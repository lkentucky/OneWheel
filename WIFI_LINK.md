# 独轮 WiFi SPI 调试链路

> **安全输出修复已更新：** 当前电机运行期输出由CPU1独立1ms中断统一管理，启动增加控制心跳/飞轮反馈新鲜度检查，新增故障需复位清除。详见 [五项修复说明](docs/SAFETY_FIXES_20261022.md)。上位机协议与硬件引脚保持原样。

本工程使用 TC264、逐飞 WiFi SPI 模块和现有 McarPVC WiFiSPI_Monitor 的 MUDP/MCAM 协议。
当前采用独轮专用上位机模式：前148字节沿用旧布局，附加OWHL v1直立遥测；默认不采集或上传图像。
使用新版 `OneWheelWiFiMonitor.exe` 或源码独轮入口。麦轮旧EXE没有完整独轮字段与参数表。
当前主循环、配置和标定步骤见 [UPRIGHT_DEBUG.md](docs/UPRIGHT_DEBUG.md)。

## 接线与网络

| 模块信号 | TC264 引脚 |
| --- | --- |
| SCK | SPI3_SCLK_P22_3 |
| MOSI | SPI3_MOSI_P22_0 |
| MISO | SPI3_MISO_P22_1 |
| CS | P22_2 |
| INT | P15_8，高为空闲、低为忙 |
| RST | P23_1 |
| 电源 / 地 | 按模块接口说明接线 |

硬件 SPI_3 配置为 30 MHz。IPS200 使用 SPI_2，参见 README 的屏幕引脚表。
接线仍需按实际主板确认。固件不会启用 WiFi INT 外部中断。

首次克隆后，将 `Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h`
复制为同目录的 `wifi_link_config.h`，再修改其中的网络宏：
`WIFI_LINK_SSID`、`WIFI_LINK_PASSWORD`、`WIFI_LINK_HOST_IP`。
本机实际配置不上传，云端只保留配置模板。
电脑和模块应处于互通的网络；目标电脑监听 UDP 8081，模块本地端口为 6666。
`MCAR SET <id> host <IPv4>` 可切换目标电脑地址；地址会校验，先向原目标发送应答，再进入 SOCKET 重连。

## 状态机与执行位置

| 状态 | 工作 | 失败处理 |
| --- | --- | --- |
| BOOT | 等待 300 ms | 到时进入 INIT |
| INIT | 原驱动初始化、连接 WiFi | BACKOFF 后重新 INIT |
| SOCKET | 原驱动建立 UDP socket | 失败等待 5 s；累计 6 次后重新 INIT |
| RUN | 分步收发、命令处理、数据/日志/图像调度 | SPI 错误或连续忙 250 ms，等待 5 s 后重新 INIT |
| BACKOFF | 按时间检查重试条件 | 不进行忙等待 |

CPU0 主循环运行 `wifi_link_service()`。**RUN 每次最多一次 CS 事务**，忙时返回；
发送分为 DATA、必要的接收排空、UDP_SEND、必要的接收排空、ACK，接收每次最多 32 字节。
`wifi_link_io.c` 直接复用本库规定的 SPI 线协议，RUN 不调用原驱动的 wait/drain 循环。
最长发送片为 1432 字节，加上 4 字节 SPI 头；底层 SPI 字节传输仍为同步轮询。

INIT 和 SOCKET 仍调用阻塞的原驱动，不能称整个链路完全非阻塞。
本工程的 CCU60_CH0 控制中断实际在 **CPU0**，优先级 30，可以抢占主循环；
初始化 PIT 的代码位于 CPU1，不代表中断在 CPU1 执行。尚未实测控制中断延迟。
`system_delay_us(300)` 是 CPU0 的忙延时节拍，不是操作系统任务切换。

CPU1 应用主循环只发布遥测、在启用图像时按需拷贝二值图像，不进行 WiFi SPI 操作。
共享遥测使用序列号和 DSYNC；图像通过请求/就绪握手发布，日志环形缓冲也有发布屏障。
控制 ISR 只读取可调目标、斜坡/死区并累加编码器，不在 ISR 内组包或发送。

名义调度周期：接收检查 5 ms，数据 50 ms，日志 20 ms，图像请求 66 ms。
调度和 SPI 忙会延后发送，66 ms 不表示已保证 15 fps。
日志出队、图像分片前进均等待模块 ACK；失败保留日志，图像从首片重试。
CRC32 每个快照仅计算一次。模块 ACK 表示 SPI 发送成功，不表示电脑已收到 UDP。

## 数据字段

MUDP 头为 `<4sBBH`：`MUDP`、版本 1、类型 1/2/3、负载长度；均为小端。
类型1负载为228字节：0～147为旧布局，148起为OWHL v1扩展。以下旧槽位名称便于核对偏移，新上位机使用独轮字段名。

| 偏移 | 上位机字段 | 独轮含义与单位 |
| --- | --- | --- |
| 0/4/8 | roll/pitch/yaw | 传感器坐标系的加速度倾角、按实际 ms 间隔积分的 Z 轴角度，单位度 |
| 12/16/20 | gx/gy/gz | 逐飞驱动换算的 °/s |
| 24/28/32 | ax/ay/az | 逐飞驱动换算的 g |
| 36 | enc_FL | 行进轮累计计数，32 位回绕，按 int32 解码 |
| 40/44/48 | 其余累计编码器 | 0，未提供 |
| 52/56 | samples/seq | 1 ms 系统计数 / 遥测包序号 |
| 60 | target_FL | 行进轮目标，编码器计数/20 ms |
| 64/68/72 | 其余目标 | 0，未提供 |
| 76 | speed_FL | 行进轮实际计数/20 ms |
| 80/84 | speed_FR/RL | 动量轮驱动返回的左/右速度，原驱动单位 |
| 88 | speed_RR | 原控制变量 car_speed，原驱动单位 |
| 92 | duty_FL | 行进轮有符号 PWM 指令，含正反向 400 补偿；停车为 0 |
| 96/100/104 | duty_FR/RL/RR | 左/右动量轮实际软件下发PWM；最后一项0 |
| 108 | vis_err_px | 原视觉误差 erro |
| 112 | omega_cmd_dps | 转向误差环给内环的角速度目标，按 IMU 比例换算 °/s |
| 116 | vx_cmd_cms | 标定后为目标 cm/s；未标定时为 0，占位表示未提供 |
| 120/124 | vy_cmd/vis_time_us | 0，未提供 |
| 128/132 | vis_interval_us/vis_frames | CPU1 观察到新摄像头帧的间隔 / 计数 |
| 136/140 | wifi_time_us/wifi_peak_us | 最近一次 / 会话内最大 SPI 事务耗时，含事务期间被中断抢占的时间，不含等待 INT 和网络传输 |
| 144 | vision_event | 当前检测结果满足信标候选坐标和像素条件时为 1 |

这些字段沿用麦轮协议槽位，FR/RL/RR 标签不代表本车有四个行进轮。
roll/pitch 是静态加速度倾角，不是原车互补滤波状态，也没有完成车体安装坐标标定。
yaw为相对积分值，会漂移。控制滤波器内部量通过下面的扩展字段单独上报，不解释为角度。

| 扩展偏移 | 含义 |
| --- | --- |
| 148/152 | 4字节OWHL标记 / uint16版本1 |
| 154 | uint16 flags：bit0运行、bit1 IMU就绪、bit2预热完成、bit3允许驱动、bit4摄像头、bit5 ROLL、bit6 PITCH、bit7 MOTOR、bit8 YAW |
| 156/160 | roll/pitch互补滤波内部量，包含机械零点 |
| 164/168 | 两轴速度环给出的平衡目标 |
| 172/176/180 | roll/pitch/yaw角速度内环目标，已含控制路径负号 |
| 184/188/192/196 | roll内环、pitch内环、额外motor环、yaw内环输出 |
| 200/204 | 两轴机械零点 |
| 208/212 | 行进目标命令 / 每1ms斜坡步长 |
| 216/220/224 | roll/pitch/yaw角速度内环原始反馈 |

除标记/版本/flags，扩展均为小端float32。控制字段直接读取各控制变量，可能跨控制节拍；不宣称硬件同步快照。

`WIFI_LINK_CM_PER_COUNT` 默认 0。只有根据实际编码器和轮周长标定后，才能设置正值并启用 cm/s 换算：
`cm/s = 计数/20ms × 50 × cm/计数`。未标定时 `cruise_cms` 写入返回 `UNCALIBRATED`，
GET ALL 不回报这个别名，独轮调参表直接使用原生 `speed`。

## 图像和日志

图传默认关闭，需按直立说明恢复摄像头及图传开关。启用后类型3使用MCAM格式1、188×120、8 位灰度；发送内容为当前 `image_binary` 二值图，
不是原始灰度图。MCAM 头为 `<4sBBHHIHHHI`，24 字节，每片最多 1400 字节，
共 17 片，末片 160 字节，整帧 CRC32 与 `zlib.crc32` 一致。
没有提供麦轮专用逐行边界/拐点标注和弯道档位扩展。

类型2文本日志包含网络状态、参数应答和运行状态变化。原有CSV调试输出默认关闭；启用APP_CSV_DEBUG_ENABLE与APP_WIRELESS_UART_ENABLE后可输出。
没有重定向所有 `printf`。CPU0 应答队列优先于 CPU1 调试日志，缓冲满时允许丢弃新日志。

## 独轮调参入口

已在 `McarPVC/upper_computer` 源码中增加独轮模式，共用原有依赖与界面框架，保留麦轮模式和已有EXE。独立程序位于其 `release/OneWheelWiFiMonitor.exe`。
在安装有该上位机依赖的 Python 环境运行：

```powershell
python tools/monitor_onewheel.py --upper-computer 'C:\Users\Lenovo\Desktop\麦轮资料\McarPVC\upper_computer'
```

本机已验证的解释器为 `C:\Users\Lenovo\Desktop\麦轮资料\WiFiSPI_Monitor\.venv\Scripts\python.exe`。
启动脚本默认识别上面的源码目录；依赖以该源码的 `requirements.txt` 为准。
独轮的设置、方案和采集文件保存至仓库 `.monitor_data/`，已忽略；可用 `--data-dir` 更改。
在界面中选择单 UDP 模式、监听 8081，将调参目标设为模块 IP、端口 6666，再点“读取全部”。

独轮有34项可调参数：speed、k1、k2、roll_zero、pitch_zero，以及下表9组PID的p/i/d，最后是ramp_ms、deadband_px。GET ALL另回读run，共35项。两项zero为机械零点内部量，不是度数；默认值从固件config.h读取或来自EXE打包快照。

| 参数前缀 | 原控制结构 |
| --- | --- |
| roll_angle | angle_cycle |
| roll_speed | speed_cycle |
| roll_rate | angular_speed_cycle |
| pitch_angle | pitch_angle_cycle |
| pitch_speed | pitch_speed_cycle |
| pitch_rate | pitch_angular_speed_cycle |
| motor | motor_cycle |
| yaw | yaw_erro_cycle |
| yaw_rate | yaw_angular_speed_cycle |

这些命名是通信映射；控制算法仍采用矿大项目的原生单位与环路。
PID、两轴零点、k1/k2和斜坡仅停车时修改。所有数字拒绝 NaN、Inf、越界和尾随垃圾字符；
超长命令整行丢弃，不执行其尾部片段。CPU1 尚未完成初始化时拒绝调参。
参数应答包含请求 ID；GET ALL 完整返回后，上位机才确认读取完成。

`ramp_ms` 定义为参考速度从 0 到 160 计数/20ms 所需的名义时间：
默认约 5333.33 ms，对应原斜坡每 1 ms 增加 0.03；设为 500 ms 时步长为 0.32。
修改目标速度后，实际斜坡时间随目标差值变化。范围 100～20000 ms。
`deadband_px` 范围 0～10；`speed` 范围 -1000～1000 计数/20ms。

固件另支持 `run` 回读和 `MCAR SET <id> run 0` 停车；不开放网络启动，非零返回 `LOCAL_START_REQUIRED`。
停车仍可使用车上按键。调参不写 Flash，重启恢复源码默认值。

## 本机验证

2026-10-21：本次直立版本ADS 1.10.10 / TASKING 1.1r8 Debug全量构建通过，168个目标文件，0错误、0警告。
原生C测试编译执行实际app/PID/control/motor/vision/WiFi源文件，覆盖默认、仅观察、仅动量轮、仅行进轮及行进方向取反五种配置。
实际C输出由新版上位机模块解析，验证228字节数据字段、图像CRC/拼接、35项参数完整确认与34项方案导入导出。
SPI等硬件由替身模拟。上位机单元测试及真实GUI回环通过：接收实际C遥测、完整读取35项回包，并确认停车请求；无网络启动。

运行可复现的主机测试：

```powershell
python tests/check_wifi_native.py --cc '路径/tcc.exe' --upper-computer '路径/upper_computer'
```

主机测试使用 TinyCC 0.9.27 Windows 的内存执行模式；它只用于测试，固件仍使用 ADS 自带的 TASKING。
没有烧录或实车测试，不能据本机通过断言无线连接、持续图传帧率、控制抖动或整车运行已经验证。
