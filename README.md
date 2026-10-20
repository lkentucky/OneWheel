# OneWheel

基于 TC264D 的独轮信标车固件，整理自中国矿业大学第 20 届全国大学生智能汽车竞赛独轮信标项目，并使用逐飞 TC264 开源库。本仓库首版将原 TFT1.8 显示改为 **IPS200 2.0 英寸屏（SPI 接口）**，保留原车的控制与视觉算法。

## 开发环境

- 单片机：Infineon AURIX TC264D / TC26x。
- IDE：**AURIX Development Studio（ADS）1.10.10**。
- 编译器：ADS 自带的 **TASKING v1.1r8**，使用工程的 `Debug` 配置。
- 底层：原项目逐飞 TC264 库及 Infineon iLLD；本次没有整体升级驱动库。

这里使用 ADS，与 AURIX Configuration Studio（ACS）及其 GCC 工具链不同。原项目按 TASKING 构建，不应直接切换成 GCC。

## 项目包含什么

- **视觉信标识别**：MT9V03X 图像采集、Otsu 阈值计算、二值化、候选信标搜索与偏差计算。主循环对阈值设置了 180 的下限。
- **姿态和平衡控制**：IMU660RB 数据采集、滤波、速度/角度/角速度控制及偏航控制。
- **电机接口**：P21.4/P21.5 双 PWM 输出、编码器计数，以及 UART3 小驱动板通信。
- **屏幕调试**：IPS200 显示二值化图像、当前/历史信标标记和偏差值。首版保持原来的半尺寸图像和标记坐标，替换对应的显示驱动接口。
- **按键及串口调试**：按键停止运行标志、无线串口数据输出。

当前代码保留原双核组织：CPU0 负责时钟、调试串口等入口，CPU1 主程序初始化外设并执行视觉/显示循环；`user/isr.c` 中的 CCU60 中断进入控制回调。控制代码以 1 ms 定时中断为基础，包含 5 ms、20 ms 分频任务和约 2 s 的初始等待。

以上描述来自当前源码；具体电机型号、机械结构和驱动板型号不能仅由代码确定。

## 目录

```text
oneWheel/
├── README.md
├── .gitignore
└── Seekfree_TC264_Opensource_Library/
    ├── .project / .cproject / .settings/  ADS 工程配置
    ├── user/                            双核入口、中断
    ├── code/                            控制、视觉、电机与通信
    ├── libraries/                       逐飞驱动、iLLD、配置静态库
    └── Lcf_Tasking_Tricore_Tc.lsl        TASKING 链接脚本
```

`code/four_posture.c` 当前内容均为注释，`code/picture_process.c` 只有说明注释。两者保留作为历史/占位文件，已在工程构建配置中排除，避免 `W508 empty source file` 警告。以后加入有效实现时，需取消对应文件的 `Exclude from Build`。

## IPS200 屏幕接线

本次参考逐飞样例 `E8_14_mt9v03x_ips200_display_demo` 的 **`IPS200_TYPE_SPI`** 模式，调用项目已有的 `zf_device_ips200` 驱动。SPI/控制引脚与原 TFT1.8 配置一致：

| IPS200 信号 | TC264 引脚/配置 |
| --- | --- |
| SCL / SCK | P15.3，SPI2 SCLK |
| SDA / MOSI | P15.5，SPI2 MOSI |
| RST | P15.1 |
| DC | P15.0 |
| CS | P15.2 |
| BL | P15.4 |
| GND、供电 | 按屏幕模块及主板的实际接口规格连接 |

显示接口选择位于 `user/cpu1_main.c` 的 `ONEWHEEL_IPS200_INTERFACE`；引脚定义位于 `libraries/zf_device/zf_device_ips200.h`。如果实际屏幕是 8 位并口版，需同时调整接口模式、接线和对应引脚，不能直接沿用本表。

## 导入与编译

1. 安装 ADS 1.10.10，选择英文路径的工作空间。
2. `File → Import → General → Existing Projects into Workspace`，选择仓库中的 `Seekfree_TC264_Opensource_Library`；建议复制到工作空间。
3. 将工程设为活动工程，选择 `Debug`，执行 `Clean Project → Build Project`。
4. 确认编译、链接成功，在工程 `Debug/` 下取得新生成的 ELF、HEX。

ADS 自带的 TASKING 工具要求通过 IDE 使用。单独在终端运行其编译器可能报 `F104: License does not support running as standalone`；使用 ADS 内的构建操作。

本仓库不提交生成的固件和构建缓存；从源码重新构建。2026-10-20，IPS200 修改后的首版已在 ADS 1.10.10 的 `Debug` 配置下完成独立副本的全量构建：**0 errors、0 warnings**，生成 165 个目标文件及新的 ELF、HEX、MAP。此次未烧录或上板验证。

## `.metadata` 是什么

如果 `oneWheel` 同时被选为 ADS/Eclipse 工作空间，IDE 会在根目录生成 `.metadata/`，保存工程关联、索引、构建日志、窗口布局、调试会话和本机偏好。它不是车辆源码，也不是项目的 `.project/.cproject` 工程定义。

`.metadata/` 已加入 `.gitignore`，保留在本机，不上传。克隆仓库后选择自己的工作空间并导入工程即可；不需要复制别人的 `.metadata`。删除它会丢失工作空间状态，因此日常无需删除。

## 来源与许可

- 原车代码：[zui811/CUMT2025-9](https://github.com/zui811/CUMT2025-9)。
- IPS200 适配参考本地逐飞仓库的 `Example/Motherboard_Demo/Motherboard_Demo/E8_camera/E8_14_mt9v03x_ips200_display_demo`。
- 保留源文件内的作者、版权和许可声明。逐飞库文件声明 GPL-3.0-or-later，相关说明位于 `libraries/doc/GPL3_permission_statement.txt`；Infineon iLLD 等第三方文件遵循各自文件中的许可。

这是软件整理与屏幕适配版本。电机、摄像头、IMU、屏幕及实际车体参数仍需按自己的硬件核对和调试；构建成功不等于整车已完成上板验证。
