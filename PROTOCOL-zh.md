[English](PROTOCOL.md)

# CO2 Sniffer — USB CDC 串口协议

Flipper 应用（`SW/co2_sniffer.c`）读取 SCD30（CO2、温度、湿度）与 BMP388
（温度、气压）传感器，并通过 USB CDC（虚拟串口）向 PC 流式发送二进制帧数据。
本文档描述这个数据流。

## 1. 传输层

- **USB 模式：**应用把 Flipper 切换到双 CDC 配置（`usb_cdc_dual`），并在
  **接口 1** 上发送数据——PC 上是**第二个**虚拟串口。第一个串口是 Flipper
  CLI 控制台。
- **波特率：**无意义。CDC 是虚拟串口；Flipper 以全速 USB 发送，上位机端设置
  的波特率会被忽略。
- **流控：**无。`furi_hal_cdc_send` 非阻塞——端点忙（上位机读取不及时）时该帧
  被**静默丢弃**。没有重传，也没有超出 USB 端点的缓冲。
- **主机打开检测：**当 DTR 置位时应用认为端口"已打开"（屏幕显示 `USB:TX`；
  `USB:wait` 表示没有主机）。
- **速率：**应用运行期间每 **333 ms**（`CO2_TICK_MS`）连续发送一帧。
- **退出时**应用恢复启动前的 USB 模式。

## 2. 帧格式

每帧 **19 字节**，载荷大端序，CRC 以小端序追加：

| 偏移 | 大小 | 字段 | 编码 |
| ---- | ---- | ---- | ---- |
| 0 | 4 | 帧头 | `DE AD FF 09`（CH552 VID `0xDEAD` + PID `0xFF09`） |
| 4 | 1 | 载荷长度 | 固定 `0x0C`（12） |
| 5 | 2 | CO2（SCD30） | uint16，**ppm**，大端序 |
| 7 | 2 | 温度（SCD30） | int16，**°C × 100**，大端序 |
| 9 | 2 | 湿度（SCD30） | uint16，**%RH × 100**，大端序 |
| 11 | 2 | 温度（BMP388） | int16，**°C × 100**，大端序 |
| 13 | 4 | 气压（BMP388） | uint32，**Pa**，大端序 |
| 17 | 2 | CRC16 | Modbus CRC，对字节 0..16 计算，**小端序** |

总计：4 + 1 + 12 + 2 = 19 字节。

解码公式：

```
co2_ppm     = uint16_be(payload[0:2])
t_scd_c     = int16_be(payload[2:4])  / 100.0
rh_scd_pct  = uint16_be(payload[4:6]) / 100.0
t_bmp_c     = int16_be(payload[6:8])  / 100.0
p_bmp_pa    = uint32_be(payload[8:12])
```

### 传感器故障哨兵值

应用每个周期都会重新初始化出故障的传感器，并保持数据流不断。故障传感器的
字段全部为 `0xFF`：

- SCD30 故障 → 字节 5..10 = `FF FF FF FF FF FF`
  （CO2 `0xFFFF`、温度 `0xFFFF`、湿度 `0xFFFF`）
- BMP388 故障 → 字节 11..16 = `FF FF FF FF FF FF`
  （温度 `0xFFFF`、气压 `0xFFFFFFFF`）

请把这些值当作**"传感器错误"**而不是数值：`0xFFFF` 温度会被解码为 −0.01 °C，
`0xFFFF` 湿度会被解码为 655.35 %RH。

## 3. CRC16

对前 17 个字节（帧头 + 长度 + 载荷）计算 Modbus CRC16：

- 多项式：`0xA001`（`0x8005` 的反射形式）
- 初值：`0xFFFF`，无最终异或
- LSB 优先（反射）
- 以小端序存储（低字节在偏移 17，高字节在偏移 18）

参考实现：

```c
/* Modbus CRC16: poly 0xA001, init 0xFFFF, LSB first */
uint16_t crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}
```

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc
```

校验方法：`crc16_modbus(frame[:17])` 必须等于 `frame[17] | frame[18] << 8`，
且对完整有效帧 `crc16_modbus(frame) == 0`。

## 4. 示例帧

| 数值 | 编码 |
| ---- | ---- |
| CO2 = 512 ppm | `02 00` |
| SCD30 温度 = 24.55 °C | `09 97` |
| SCD30 湿度 = 43.20 %RH | `10 E0` |
| BMP388 温度 = 24.62 °C | `09 9E` |
| BMP388 气压 = 100123 Pa | `00 01 87 1B` |

```
DE AD FF 09 0C 02 00 09 97 10 E0 09 9E 00 01 87 1B CA E0
└ 帧头 ┘ └长度┘ └────────── 载荷 (12) ──────────┘ └ CRC16 LE
```

字节 0..16 的 CRC16 = `0xE0CA`，以小端序存储为 `CA E0`。

## 5. 解析数据流

数据流是连续的字节流，没有空闲标记。同步方法：

1. 扫描帧头 `DE AD FF 09`。
2. 帧头后的字节是长度；应为 `0x0C`。
3. 再取 `1 + len + 2` 个字节，对 `帧头 + 长度 + 载荷` 校验 CRC
   （`crc16_modbus(frame[:17]) == crc_le`）。不匹配则丢弃并重新扫描。
4. 对齐后也可以按 19 字节分块——每块 CRC 有效即可保持对齐不被破坏。

注意：上位机未读取时 Flipper 会丢弃帧，所以解析器绝不能假设帧序号连续
（协议里本来也没有序号）。

## 6. 采样与更新速率

帧的发送周期（333 ms）**快于**传感器的更新速率：

- **SCD30：**初始化时测量间隔设为 **2 s**，因此 CO2、SCD30 温度与湿度约每
  2 s 更新一次（每 6 帧），期间数值重复。初始化时关闭了 ASC（自动自校准）。
- **BMP388：**配置为 50 Hz 输出（OSR ×32/×32，normal 模式），每个 333 ms 周期
  都读取；其温度/气压在每一帧中都是新值。
- **气压补偿：**应用根据 BMP388 气压的低通滤波值
  （`P_FIL = 0.95·P_FIL + 0.05·P_LIN`）向 SCD30 提供环境气压（mbar）。当滤波
  气压发生变化且处于 700..1200 mbar 范围内时，用新气压重启 SCD30 测量。取整
  方式为 `(uint16_t)((P_FIL + 0.5) / 100)`。
- **重新初始化：**在 Flipper 上按 **OK** 会重新初始化两个传感器（软复位；
  原来的 CH552 板是直接断电重启）。

## 7. 历史说明

原来的 CH552 嗅探板以 HID 报告的形式发送相同布局的数据。FAP 无法注册自定义
HID 描述符，所以本移植版通过 CDC 发送完全相同的帧。布局与 CH552 的 HID 报告
字节兼容，这也是帧头编码 CH552 VID/PID 的原因。
