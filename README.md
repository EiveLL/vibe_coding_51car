# 51car 第一次下载测试

本例程用于先验证 Keil C51 编译、生成 HEX 文件和单片机下载是否正常。它暂时不包含 RC522、循迹或电机控制代码。

## 默认硬件

- 单片机：STC89C52RC 或兼容 8052 内核的芯片
- 晶振：12 MHz
- 测试 LED：P1.0，低电平点亮

请先查看开发板原理图或丝印。如果板载 LED 不是 P1.0，请修改 `main.c`：

```c
sbit LED = P1^0;
```

例如 LED 接在 P2.0 时改为：

```c
sbit LED = P2^0;
```

## 使用 Keil C51 编译

1. 双击 `51car.uvproj`，或在 Keil uVision 中选择 `Project -> Open Project`。
2. 点击 `Project -> Build Target`，也可以按 `F7`。
3. 编译成功后，输出应显示 `0 Error(s), 0 Warning(s)`。
4. 生成的下载文件位于 `Objects\51car.hex`。

## 使用 STC-ISP 下载

Keil 负责编译，STC89 系列通常使用 STC-ISP 软件通过串口下载，不使用 Keil 的 `Download` 按钮。

1. 用 USB 转串口模块连接开发板，确保 TXD/RXD 交叉连接并共地。
2. 打开 STC-ISP，选择实际单片机型号和串口。
3. 加载 `Objects\51car.hex`。
4. 点击“下载/编程”。
5. 按 STC-ISP 提示给开发板重新上电。
6. 下载完成后，LED 应约每 0.5 秒切换一次状态。

## 常见问题

- LED 一直不亮：先确认 LED 引脚和有效电平。若 LED 高电平点亮，需要交换 `LED = 0` 和 `LED = 1`。
- STC-ISP 一直检测不到芯片：检查芯片型号、串口号、TXD/RXD、GND，并在提示后重新上电。
- 闪烁速度明显不对：在 STC-ISP 中确认内部时钟设置，或按实际晶振调整延时。
- 电机电源不要直接由单片机 I/O 供电；第一次测试只连接最小系统和 LED。
