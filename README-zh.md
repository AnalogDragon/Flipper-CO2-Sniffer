[English](README.md)

# CO2 Sniffer

Flipper Zero 的 BMP388 + SCD30 扩展板与应用。扩展板插在 Flipper 的 GPIO 排针上；
应用通过外部 I2C 总线读取两个传感器，在屏幕上显示数值，并通过 USB CDC 向 PC
流式发送帧数据。
其中 `SW/` 软件工程由 OpenAI Codex 编写。

## 当前功能

- 通过 SCD30 读取 CO2、温度和湿度，通过 BMP388 读取温度和气压。
- 使用多种仪表布局显示实时数据，并根据气压估算海拔。
- 提供 CO2、温度、湿度、气压和海拔历史曲线，可选择不同时间尺度。
- 通过 USB CDC 将读数发送到 PC，并以可配置的采样间隔和记录时长将带时间戳的
  CSV 文件保存到 microSD 卡。
- 自动检测传感器和 I2C 总线故障并重试或恢复；只有一个传感器在线时也可继续
  工作，总线恢复失败时会提示重启 Flipper。

## 按键操作

| 按键 | 操作 |
| ---- | ---- |
| **左 / 右** | 浏览记录页、仪表页和 5 个历史曲线页；编辑记录参数时切换选项 |
| **上 / 下** | 切换仪表布局或曲线时间尺度；在记录页移动光标 |
| 短按 **OK** | 编辑或确认记录参数、开始/停止记录，或确认对话框 |
| 短按 **BACK** | 打开或取消清空历史对话框；取消退出对话框 |
| 长按 **OK / BACK** | OK 切换背光常亮；BACK 打开退出确认 |

## 数据记录

在仪表页按 **左** 打开 **Data Recording**。用 **上 / 下** 选择字段，短按
**OK** 进入编辑，再用 **左 / 右** 切换选项。选中 **Start** 后按 **OK** 开始
记录；再次按 **OK** 停止。

| 设置 | 可选值 |
| ---- | ------ |
| 采样间隔 | 2 秒、5 秒、10 秒、30 秒、1 分钟 |
| 记录时长 | 1 分钟、5 分钟、10 分钟、30 分钟、1 小时、持续记录 |

文件保存为 `/ext/co2_sniffer/YYYYMMDD_HHMMSS_<interval>_<length>.csv`。列名依次
为 `timestamp`、`elapsed_s`、`co2_ppm`、`scd30_temperature_c`、`humidity_pct`、
`bmp388_temperature_c`、`pressure_hpa` 和 `altitude_m`；离线传感器对应的字段
留空。闪烁的 `REC` 表示正在记录；`SD!` 表示文件或 microSD 卡写入失败。

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

`SW/` 是 Flipper 应用（FAP，appid `co2_sniffer`，分类 DIY，v0.4）。工作线程的
主要流程如下：

1. 检测并初始化 SCD30 和 BMP388。
2. 读取可用传感器并更新实时仪表页。
3. 保存读数供历史曲线页面显示。
4. 通过 USB CDC 发送读数，并可选择保存为 CSV。
5. 自动重试断开的传感器，退出时恢复系统设置。

## 目录结构

- `HW/` — KiCad 硬件工程：原理图、PCB、V1.0 Gerber 在 `Flipper-CO2-module-V10/`。
- `SW/` — Flipper 应用源码；构建产物写入 `SW/dist/`。
- `3d/` — 可打印的外壳文件：含拓竹打印配置的 3MF 文件和原始 STEP 模型。
- `PROTOCOL-zh.md` — USB CDC 串口协议完整文档。
- `README.md` — 本文档的英文版。

## 许可证

GPL-3.0，见 [LICENSE](LICENSE)。

## 编译与安装

```sh
cd SW
ufbt            # 编译生成 dist/co2_sniffer.fap
ufbt launch     # 安装并运行在已连接的 Flipper 上
```

应用安装到 `/ext/apps/DIY/`（FAP 分类 DIY）。

## 串口帧格式

应用通过 USB CDC 发送紧凑的二进制数据。在 PC 上应使用第二个虚拟串口，第一个
是 Flipper CLI 控制台。完整帧格式和解析方法见
**[PROTOCOL-zh.md](PROTOCOL-zh.md)**。
