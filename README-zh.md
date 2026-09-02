[English](README.md)

# CO2 Sniffer

Flipper Zero 的 BMP388 + SCD30 扩展板与应用。扩展板插在 Flipper 的 GPIO 排针上；
应用通过外部 I2C 总线读取两个传感器，在屏幕上显示数值，并通过 USB CDC 向 PC
流式发送帧数据。

## 当前功能

- 通过 Flipper 的外部 I2C 总线（C0 = SCL，C1 = SDA）读取 SCD30（CO2、温度、
  湿度）和 BMP388（温度、气压）。
- 屏幕实时显示：CO2 ppm、BMP388 温度与气压、SCD30 温度与湿度、每个传感器的
  OK/ERR 状态、USB 状态与帧计数。
- 每 333 ms 通过 USB CDC 向 PC 发送一帧 19 字节的二进制数据（见
  [PROTOCOL-zh.md](PROTOCOL-zh.md)）。
- 传感器故障时每个周期自动重试初始化；数据流持续运行，故障传感器上报
  `0xFF…` 哨兵值。
- **OK** 键重新初始化两个传感器，**BACK** 键退出。
- 退出时恢复应用启动前的 USB 模式。

## 硬件

`HW/` 是 KiCad 工程。板子是一个 2.54 mm 1×10 排针模块，插入 Flipper 的 GPIO
口：

| 器件 | 功能 | I2C 地址 |
| ---- | ---- | -------- |
| Sensirion SCD30 | CO2、温度、湿度 | 0x61 |
| Bosch BMP388 | 温度、气压 | 0x76 |

I2C 使用 GPIO C0（SCL）/ C1（SDA）；板子由 Flipper 的 3V3/5V 引脚供电。
`HW/Flipper-CO2-module-V10/` 内含可直接打样的 V1.0 Gerber 文件。

## 软件

`SW/` 是 Flipper 应用（FAP，appid `co2_sniffer`，分类 DIY，v0.2）。工作线程的
行为如下：

- **SCD30** — 软复位、关闭 ASC（自动自校准）、2 s 测量间隔、连续测量。环境
  气压补偿由 BMP388 提供：当滤波后的气压在允许的 700..1200 mbar 范围内变化
  时，用新值重启 SCD30 测量。
- **BMP388** — normal 模式、50 Hz ODR、OSR ×32/×32、NVM 校准。
- **USB** — 切换到 `usb_cdc_dual` 并在接口 1 上发送，即 PC 上的**第二个**
  虚拟串口（第一个是 Flipper CLI）。屏幕上的帧计数统计交给 USB 层的帧；端点
  忙（上位机未读取）时这些帧仍可能被静默丢弃。

## 目录结构

- `HW/` — KiCad 硬件工程：原理图、PCB、V1.0 Gerber 在 `Flipper-CO2-module-V10/`。
- `SW/` — Flipper 应用源码；`SW/dist/co2_sniffer.fap` 是预编译好的 FAP。
- `PROTOCOL-zh.md` — USB CDC 串口协议完整文档。
- `README.md` — 本文档的英文版。

## 许可证

GPL-3.0，见 [LICENSE](LICENSE)。

## 编译与安装

```sh
ufbt            # 编译生成 SW/co2_sniffer.fap
ufbt launch     # 安装并运行在已连接的 Flipper 上
```

应用安装到 `/ext/apps/DIY/`（FAP 分类 DIY）。

## 串口帧格式

完整协议文档（传输层、CRC 参考代码、解析指南）：**[PROTOCOL-zh.md](PROTOCOL-zh.md)**。

每 333 ms 通过 USB CDC 发送一帧，19 字节，大端序：

| 偏移 | 大小 | 字段 |
| ---- | ---- | ---- |
| 0 | 4 | 帧头 `DE AD FF 09`（CH552 的 VID 0xDEAD + PID 0xFF09） |
| 4 | 1 | 载荷长度（12） |
| 5 | 2 | CO2，uint16 ppm |
| 7 | 2 | SCD30 温度，int16 °C×100 |
| 9 | 2 | SCD30 湿度，uint16 %RH×100 |
| 11 | 2 | BMP388 温度，int16 °C×100 |
| 13 | 4 | BMP388 气压，uint32 Pa |
| 17 | 2 | CRC16（Modbus：poly 0xA001，init 0xFFFF，反射，小端序）对字节 0..16 计算 |

传感器故障时对应字段上报 `0xFFFF` / `0xFFFFFFFF`。

在 PC 上，数据流出现在**第二个**虚拟串口；第一个是 Flipper CLI 控制台。
