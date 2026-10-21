# Qcar — TC264 独轮信标工程

IMU 姿态解算现已移植自 MCARpreparation，默认 EKF，硬件为逐飞 IMU660RB。使用方法、轴映射、控制接口单位及验证说明见 [IMU.md](IMU.md)。

MCARpreparation 的 WiFi-SPI UDP/JustFloat 传输已移植，支持通道订阅和 PID 滑块下发。从 **WiFi → Connect** 显式连接，接线、网络配置与命令见 [WIFI.md](WIFI.md)。

基于逐飞 `E13_dual_core_demo` 示例，将 [feng0511/monotroch](https://github.com/feng0511/monotroch) 的 TC377 应用移植到 TC264D。底层驱动、启动文件、TC26B iLLD 和链接脚本来自当前目录旁的 `TC264_Library`，应用算法来自 monotroch。

## 中文编码

源码及文档统一使用 UTF-8。VS Code 直接打开 **Qcar 文件夹**，会自动应用 `.vscode/settings.json`；ADS 使用 `.settings/org.eclipse.core.resources.prefs` 中的工程级 UTF-8 设置。已打开的旧标签页需关闭后重新打开，ADS 可先选中工程按 F5 刷新。

已从原始副本恢复可逆的错误转码注释；原文已丢失的注释已结合函数、变量和算法重新写成中文说明。重写说明用于解释现有实现，不代表逐字还原原作者文字。已核对 C/H 文件的非注释内容未改变，字符串和算法保持原样。

## 导入与编译

1. 打开 AURIX Development Studio，选择 **File → Import → General → Existing Projects into Workspace**。
2. 选择本目录，导入项目 **Qcar**。保持工程名为 Qcar。
3. 选中工程，按 F5 刷新，选择 **Project → Build Project**，配置为 **Debug**。
4. 产物位于 `Debug/Qcar.elf`、`Debug/Qcar.hex`，内存分配见 `Debug/Qcar.map`。

验证环境：ADS 1.10.10，内置 TASKING 1.1r8 Build 22011964，处理器 `tc26xb`。内置非商业 TASKING 授权要求从 ADS 内编译；独立调用编译器会返回 F104，不能用普通终端脚本代替 IDE 构建。

软件编译和链接通过不等于实车调试完成。本工程尚未连接开发板、烧录或验证运行周期。首次使用前需核对下表接线、IMU 坐标映射、编码器方向和电机极性；PID、相机标定及阈值保留上游参数，需要按车重新调试。

## 移植内容

| 内容 | TC264 实现 |
| --- | --- |
| CPU0 | 菜单、初始化、WiFi-SPI UDP 遥测、UART 接收中断、1 ms 级联控制中断、5 ms 姿态更新中断 |
| CPU1 | 视觉识别、轨迹规划；编译关闭 UDP 遥测后可使用原 TCP 图传预览 |
| CPU2 | TC264 不存在该核，已合并任务，无 CPU2 启动入口 |
| 无刷驱动 | 原 UART4 改为 UART2，460800 波特，保留上游 7 字节协议 |
| 逐飞助手 | 适配本地 TC264 库新版相机、示波器及参数解析接口；调试通信使用 UART0 |
| 屏幕 | IPS200 SPI 横屏，补入上游目标框、轮廓和轨迹叠加显示 |
| 视觉容量 | 原 200 项缓存统一改为 32 项；保留 188 × 120 分辨率和 40 帧历史窗口 |
| 内存 | 连通域标签工作区放 CPU0 DSPR；其余视觉状态放 CPU1 DSPR；两核用户栈各 8 KiB |
| 图像共享 | CPU1 在处理前复制一帧到工作缓存，识别与图传共用缓存，只有 CPU1 消费完成标志 |
| 参数保存 | 使用 TC26B Flash 定义，参数页保持上游布局；新增版本标记，空 Flash 使用默认参数 |

连通域最多使用 31 个非零临时标签（32 项中第 0 项保留为背景）；容量用尽时跳过新增区域，防止访问数组边界之外。此限制也可能漏掉噪声很多的图像中的后续目标，应通过阈值和光学条件控制噪声。不要只增大 `QCAR_MAX_CONTOURS`，还要重新检查 map 的 RAM 余量。

## 默认接线

这些是工程采用的默认映射，尚未获得实际主板引脚表。应用侧配置集中于 `code/qcar_config.h`；摄像头、IMU、屏幕和 Wi-Fi 的完整配置仍位于各自的 `libraries/zf_device/zf_device_*.h`。

| 设备/用途 | TC264 引脚及配置 |
| --- | --- |
| 调试/逐飞助手 | UART0 TX=P14.0、RX=P14.1，波特率见 `zf_common_debug.h`；不再占用 UART2 无线串口模块 |
| 无刷驱动 | UART2 TX=P33.9、RX=P33.8，460800；需与驱动的 RX/TX 交叉连接 |
| 遥控接收 | UART3 TX=P15.6、RX=P15.7，115200 |
| MT9V03X 配置 | UART1 TX=P02.2、RX=P02.3，9600 |
| MT9V03X 图像 | D0～D7=P00.0～P00.7，PCLK=P02.1，VSYNC=P02.0，DMA5 |
| IMU660RB | SPI0 SCK=P20.11、MOSI=P20.14、MISO=P20.12、CS=P20.13 |
| IPS200 SPI | SCK=P15.3、MOSI=P15.5、DC=P15.0、CS=P15.2、RST=P15.1、背光=P15.4 |
| 中轮驱动 | PWM=ATOM0_CH5/P02.5，方向=P02.4，20 kHz |
| 中轮编码器 | TIM6 CH1=P20.3、CH2=P20.0 |
| EC11 编码器 | TIM5 CH1=P10.3、CH2=P10.1 |
| 按键 | KEY1=P20.8、KEY2=P20.6、KEY3=P20.7、EC11 按键=P20.9 |
| 可选 Wi-Fi SPI | SPI3 SCK=P22.3、MOSI=P22.0、MISO=P22.1、CS=P22.2、INT=P15.8、RST=P23.1 |

IMU 保留原车安装映射：车体 X/Y/Z 分别取传感器 Y/Z/X。不同安装方向必须调整 `code/attitude_algorithm.c` 中的 `imu660rb_GetData()`。

Wi-Fi UDP 功能默认编译启用，开机不自动联网；网络配置集中在 `code/qcar_config.h`，沿用 MCARpreparation 的当前配置。停车并退出预览后，从 WiFi → Connect 连接，随后可开启 IMU 预览或控制运行并持续遥测。原 CPU1 TCP 图传可在 `QCAR_WIFI_ENABLED=0` 时恢复，与 UDP 遥测互斥，详见 [WIFI.md](WIFI.md)。

Flash 使用逻辑扇区 0、物理页 0～76；第 0 页保存移植版标记 `0x51434131`。不读取 TC377 设备的旧 Flash 数据。写入通过菜单 KEY3 触发，首次空片不会自动写入。标记用于区分版本，不包含校验和或掉电恢复。

## 主要文件

菜单已替换为 mcar-mailun 的树形节点结构，操作与目录见 [MENU.md](MENU.md)。

- `user/cpu0_main.c`、`user/cpu1_main.c`：双核入口。
- `user/isr.c`：控制、姿态、相机 DMA/场中断及 UART 接收。
- `code/pid.c`、`attitude_algorithm.c`：平衡、运动控制与姿态算法。
- `code/image_process.c`、`component.c`、`target_classifier.c`：信标识别与跟踪。
- `code/tragectory_planning.c`、`pixel2world.c`：轨迹规划及原车相机标定。
- `code/menu.c`、`tune.c`、`flash.c`：调参、菜单与参数保存。
- `THIRD_PARTY.md`：来源、版本及版权说明。
- `VALIDATION.md`：编译验证记录。
