# OneWheelUpright — TC264 直立调试工程

基于上游 `7a053b2` 精简，只保留直立控制、IMU、驱动、IPS200 菜单、参数保存和 WiFi-SPI 遥测。应用中的信标/视觉识别、轨迹规划、遥控、旧 Mode0～2、旧速度级联及未使用的 MAF/Kalman/模糊/LQR 算法已经移除。逐飞/Infineon 通用 SDK 仍保留，避免破坏底层依赖；它们的摄像头接口不再从应用初始化或中断调用。

编译与主机回归的验证记录见 [VALIDATION.md](VALIDATION.md)，尚未烧录或实车验证。

## 快速使用

1. 在 `code/config.h` 核对接线、WiFi、PC地址和控制参数；默认选择 Mode5（双轴直立），上电不自动运行。
2. 用 ADS 导入本目录的 `OneWheelUpright` 工程，选择 TASKING Debug 构建，烧录本次生成的 HEX。
3. 上电后等待 IMU 标定完成，通过 Startup 菜单选择模式；联网需单独选择 `WiFi connect`。
4. 单轴检查后，双轴直立选择 Mode5，再执行 Startup / Start；停车选择 Startup / Stop。Mode5没有速度/位置外环。
5. 参数可从 PID 菜单或已支持的上位机命令调整；停车后用 Storage / Save 保存。新格式不读取旧版 Flash 参数。

构建产物、IDE工作区和本地测试缓存不上传 Git；克隆后需在本机重新构建。

## 从哪里读代码

```text
user/cpu0_main.c         app_init(); while(1) app_poll();
code/app.c              系统初始化、应用轮询入口
code/config.h           唯一应用配置入口：参数、引脚、网络、周期、滤波、IRQ
code/balance_control.c  Mode2～8、控制状态、保护、限幅、启动斜坡
code/pid.c/.h           通用位置式 / 增量式 PID，不依赖 TC264 SDK
code/imu.c              SPI采样、静止标定、有效性检查、状态发布
code/attitude.c         EKF姿态解算、加速度低通、创新门限
code/attitude_algorithm.c  向控制层发布机体角速度和姿态
code/menu.c             菜单、显式启停、参数编辑、联网、屏幕显示
code/menu_tree.c        通用菜单树和静态节点池
code/motor.c            行进轮 PWM+DIR
code/wheel_encoder.c/.h 停车仍采样的地轮编码器观察；计数、速度、位置有效性
code/wheel_hold.c/.h    Mode2位置P→目标速度→速度PI→目标倾角修正
code/small_driver_uart_control.c  无刷驱动文本兼容发送、文本/二进制转速接收
code/wifispi.c          JustFloat遥测、订阅、RAM参数写入
code/flash.c            仅直立参数的保存、校验和恢复
user/isr.c              1ms控制入口、2ms采样、1ms按键、串口反馈中断
user/cpu1_main.c        CPU1空闲；不再处理图像或网络
```

`app_init()` 按顺序初始化时钟、调试口、IPS200、菜单/默认与保存参数，通知 CPU1，然后初始化零输出和 IMU。上电不自动启动电机、不自动联网。`app_poll()` 服务按键、IMU请求、开环测试、WiFi和显示；控制计算继续在中断中运行。

| 任务 | 默认周期 / 所在核 |
|---|---|
| 控制中断入口 CCU60_CH0 | 1ms / CPU0；每2次执行一次控制，即500Hz |
| IMU CCU60_CH1 | 2ms / CPU0；加速度/陀螺硬件ODR均833Hz |
| 按键 CCU61_CH1 | 1ms / CPU0 |
| 地轮编码器观察 | 同一按键中断内约2ms读计数，约20ms发布；速度使用STM实测间隔 |
| 主循环 | 每轮末尾等待10ms；同步网络/屏幕操作会延长一轮 |
| 屏幕 | 每5轮轮询刷新 |
| 遥测 | Start/Connect选定配置时20ms；允许命令调整 |

## 一处调参数

直接修改 `code/config.h`，重新编译、烧录。`qcar_config.h`、`imu_config.h` 已删除；驱动可调引脚/速率也引用此文件。`user/isr_config.h` 仅为 SDK 保留的包含入口，没有第二份参数值。

| 参数组 | 当前编译默认值 | 含义 |
|---|---|---|
| `BALANCE_ZERO_X_DEG` / `BALANCE_ZERO_Y_DEG` | 4.2° / 1° | 保留本机手调的行进轮 / 飞轮机械目标角 |
| `GROUND_RATE_KP` / `GROUND_ANGLE_KP` | 30 / 40 | 行进轮内环 / 外环增益 |
| `FLY_RATE_KP` / `FLY_ANGLE_KP` | 36 / 40 | 飞轮内环 / 外环增益 |
| `GROUND_RATE_LIMIT_DPS` / `FLY_RATE_LIMIT_DPS` | 100 / 160 °/s | 外环目标角速度限幅；运行期菜单值可不同 |
| `GROUND_PWM_LIMIT` / `FLY_PWM_LIMIT` | 4000 / 8000 | 输出指令限幅；不是电流限制 |
| `*_OUTPUT_SIGN` | +1 / +1 | 已验证的纠偏方向，保持不变 |
| `*_KI` / `*_KD` | 全部0 | 当前保持原 P/P 控制；启用积分还要设置相应 `*_I_LIMIT_*` |
| `BALANCE_*_PID_INCREMENTAL` | 全部0 | 0位置式，1增量式；分别选择角度环和角速度环 |
| 控制环反馈 | 直接使用姿态角和校正后的机体角速度 | 两轴角度低通旁路；飞轮角速度采用5ms低通；保留EKF及其内部加速度低通 |
| `BALANCE_STARTUP_RAMP_MS` | 1000ms | 启动PWM上限逐渐放开 |
| `BALANCE_TILT_LIMIT_DEG` / `BALANCE_IMU_STALE_MS` | 30° / 25ms | 原始角度倾倒保护 / 控制采样过期保护 |
| `APP_DEFAULT_MODE` / `DEBUG_PWM_LIMIT` | 5 / 4000 | 上电选中模式 / Mode8有符号PWM绝对上限 |

菜单和上位机修改的是 RAM 值。`QCAR_BOOT_LOAD_FLASH=1` 时，有效的新格式 Flash 记录会覆盖对应默认值；需要固定使用 config 默认值时设置为0。改 config 不会自动更新芯片里的保存记录。

## 模式与菜单

| Mode | 控制内容 | 参数菜单 |
|---|---|---|
| 2 | Mode5双轴平衡＋地轮位置P、速度PI串级外环 | PID / Speed hold RAM，及原两组姿态菜单 |
| 3 | 飞轮：角度环→角速度环 | PID / Gyro test |
| 4 | 行进轮：角度环→角速度环 | PID / Pitch balance |
| 5（默认） | 上述两轴同时启用 | 两组菜单 |
| 6 | 飞轮角速度环，目标角速度0 | PID / Gyro test |
| 7 | 行进轮角速度环，目标角速度0 | PID / Pitch balance |
| 8 | 左飞轮、右飞轮、行进轮三路独立开环PWM，不经过PID | PWM debug / Fly left PWM、Fly right PWM、Wheel PWM |

菜单允许2～8。Mode2的Start启用双轴控制和地轮位置、速度串级外环，需要IMU及新鲜有效、位置连续的编码器反馈。每次Start重建计数基准，首次IMU和编码器均就绪时锁定当前位置作为目标，不是回到上电位置。Mode6/7抑制转动，不主动回到机械零点。Mode5没有速度/位置外环，不保证原地不动。默认Mode5仅为上电选择，仍需显式Start。

注意沿用上游的轴命名：屏幕 Roll / 飞轮使用 `imu_pitch_deg` 与 `gyro.y`，机械零点是 Balance Y；屏幕 Pitch / 行进轮使用 `imu_roll_deg` 与 `gyro.x`，机械零点是 Balance X。上位机 `roll_deg` 对应 Balance X，`pitch_deg` 对应 Balance Y。

菜单为 Startup、PID、Storage、PWM debug。WiFi连接位于 **Startup / WiFi connect**；Startup保留 Start/Stop、IMU重新标定预览、旧行进轮Open loop和Wheel PWM（与Mode8参数独立）。

K1上移/加，K2下移/减，K3进入/选中/切换步进，K4返回/取消选中。K4不是停车键；停车使用 Startup / Stop。改变Mode会先停车；保存和加载要求停车且退出预览。

### Mode8 开环调试

先固定或架空车辆，在 Startup 选择8并Start（输出仍为0），再进入 PWM debug 调整 Fly left PWM / Fly right PWM / Wheel PWM，分别控制左飞轮、右飞轮、行进轮。三路参数范围均默认为-4000～4000，0停车，负数反向；10000代表100%占空比，4000为40%指令。左右飞轮使用各自独立的逻辑输入，不乘PID的Sign参数；串口发送入口分别应用 config.h 的 QCAR_FLYWHEEL_LEFT_SIGN / QCAR_FLYWHEEL_RIGHT_SIGN（当前均为-1），可独立修改任意一路的最终方向。此极性也适用于Mode3/5/6及停车串口诊断，行进轮和磁编原始反馈符号不变。

Mode8的PWM设定值和控制器兼容字段保留逻辑符号；屏幕PWM L/R及遥测motor_left_pwm/motor_right_pwm表示应用硬件极性后的最终下发指令。例如左设定+300，最终左下发-300。

Mode8不运行PID、不自动回正，也不要求IMU就绪或执行倾角/采样过期保护。它保留默认1秒启动限幅斜坡、非有限值/越界参数故障锁存、Stop立即硬件归零；任一路参数无效会停掉三路。上电、每次Start、Stop、切换模式都会清零三路Mode8设定值；不保存到Flash。斜坡只在启动时放开输出上限，后续手动修改PWM不会逐次缓变。

WiFi连接应在停车时完成。Mode8 Start后可发送 `[slider,debug_fly_left_pwm,300]`、`[slider,debug_fly_right_pwm,-700]`、`[slider,debug_ground_pwm,-500]`；可订阅字段为 `pwm_debug_enabled` 以及这三个PWM设定名。旧的 `debug_fly_pwm` 配对指令不再支持。默认使用45通道双轴帧，其中 `motor_left_pwm` / `motor_right_pwm` / `motor_ground_pwm` 为三路实际下发指令。Mode8的兼容字段 `dual_roll_pwm` 仅表示左飞轮，`dual_pitch_pwm` 表示行进轮，`dual_enabled=0`表示没有双轴闭环。没有IMU时角度/角速度字段不作为有效测量使用。原上位机若没有对应滑杆，可用指令发送框；远程Start/Stop仍不支持。

### Mode2 地轮位置、速度外环

控制链为 `参考位置 - wheel_position_count → 位置P → 目标速度 → 速度PI → 倾角修正 → Balance X + 倾角修正 → 原地轮角度环/角速度环 → PWM`；飞轮沿用Mode5。位置环累计新窗口实测时间，约100ms更新一次；速度窗口约20ms且仅新序号计算一次PI，dt使用窗口实测时间；姿态内环仍2ms。位置/速度外环没有新增低通或速度PWM并联支路，飞轮反馈使用云端5ms角速度低通。

| config默认 / 菜单 | 数值 | 单位和作用 |
|---|---|---|
| WHEEL_SPEED_KP / Speed KP | 0.001 | 度/(计数/秒)；P起始值，需实车验证 |
| WHEEL_SPEED_KI / Speed KI | 0 | 度/计数；暂不累积积分 |
| WHEEL_SPEED_TILT_LIMIT_DEG / Tilt limit deg | 1 | 目标倾角修正限幅±1°，允许菜单0～5° |
| WHEEL_SPEED_I_LIMIT_DEG / I limit deg | 0.5 | PI内部积分项限幅±0.5° |
| WHEEL_SPEED_TILT_SIGN | -1 | PI输出到目标Pitch修正的符号，结合本车实测方向 |
| WHEEL_SPEED_ENCODER_TIMEOUT_MS | 60 | 最新编码器速度窗口年龄达到60ms则故障 |
| WHEEL_POSITION_KP / Position KP | 0.5 | 单位1/秒，位置误差转换为目标速度 |
| WHEEL_POSITION_SPEED_LIMIT_CPS / Speed limit cps | 300 | 目标速度限制±300计数/秒，约4.4地轮转/分 |
| WHEEL_POSITION_PERIOD_MS | 100 | 位置环更新间隔 |
| WHEEL_POSITION_DEADBAND_COUNT | 8 | 连续死区：误差绝对值≤8输出0，超出部分乘KP |

向右位移为正时，位置误差为负，位置环要求向左的负目标速度。例如当前位置+512、参考0、速度0，扣除8计数死区后，默认目标速度为-252计数/秒，速度PI经-1产生+0.252°目标Pitch修正。目标速度为0时，当前向右+500计数/秒产生+0.5°修正。这是软件符号关系，仍需实车验证动力学。外环不能替代两轴姿态稳定性；速度KP、KI同时为0时，位置环有目标速度也不会产生倾角修正。

菜单路径为PID / Speed hold RAM，共六项。Speed KP/KI默认步进0.001，K3可切换，最小0.00001；Position KP默认步进0.1，最小0.001；屏幕标题显示实际步进。六项上位机写入名为wheel_speed_kp、wheel_speed_ki、wheel_tilt_limit、wheel_i_limit、wheel_pos_kp、wheel_speed_limit。它们只存在config默认及RAM，不属于旧Flash记录；Storage/Save不保存它们，Load或重启恢复config默认。Position KP或Speed limit为0时，退回零速度外环。

Mode2屏幕V为实际计数/秒、T为目标速度、O为目标倾角修正（度）；下一行为实际/目标Pitch，第三行显示双轴PWM/状态。运行底部P/E为当前计数/控制位置误差，OK为速度/位置有效性。Start/Stop/切模式清PID并解除参考锁定；标定时不输出、不捕获位置；启动渐增及地轮PWM饱和时冻结速度积分。短时抑制清PID但保留已锁定参考。运行中位置不连续或基准意外变化触发State=-4/F=10，不自动改变参考。编码器无效窗口为State=-4/F=7；年龄达到60ms为State=-4/F=8；外环参数或组合目标角越界为State=-3/F=9。故障后三路输出归零，位置误差保留最后控制值，需Stop→Start重建控制状态和位置基准。

WiFi连接或Start时Mode2选择48通道位置保持profile，上位机也可订阅“Mode2位置保持 · 48通道”；新增wheel_ref_count、wheel_pos_error、wheel_pos_kp、wheel_speed_limit并带wheel_position_valid。字段总表123项；旧43通道speed及39/45/28通道profile、8081/5001端口和本机IP保留。control_mode是真实已启动模式，Stop后0。配套上位机方案升级V4共21项，兼容V1/V2/V3；导入仅修改本地草稿，不自动发送。

## PID与滤波

`pid_position_update()` 计算绝对输出；`pid_incremental_update()` 累加输出增量。两个函数都接受独立状态、目标、反馈、秒为单位的 dt，带输出/积分限幅、条件抗积分饱和、首帧微分抑制和无效数值检查。

每轴的角度环、角速度环各自拥有状态，停车/重启清空历史。默认 I/D=0、位置式，数值上保持原来的 P/P 路径：

```text
角度误差 = 机械零点 - EKF姿态角
目标角速度 = clamp(Angle_KP × 角度误差, ±Rate_limit)
PID输出 = clamp(Rate_KP × (目标角速度 - 校正后机体角速度), ±PWM_limit×启动比例)
输出 = round(Sign × clamp(PID输出 + 角度补偿, ±PWM_limit×启动比例))
角度补偿 = FLY_ANGLE_FF_KP × 角度误差（仅Mode3/5飞轮轴，且Angle_KP>0）
```

增量式与位置式在未饱和、固定增益时等价；饱和后增量式从上次限幅输出继续累加，不能假设两种形式表现相同。运行中切换算法应先停车并重新启动。

保留EKF、其内部加速度低通和门限。两轴直接使用EKF姿态角（角度低通tau=0旁路）；Mode2/3/5/6飞轮角速度在进入内环前经过tau=5ms的一阶低通，2ms步长时alpha=2/7，地轮角速度不滤波。只处理新鲜且通过有效性检查的采样，旧序号不重复滤波；Stop/Start、标定和故障清滤波状态，首次有效值直接初始化。遥测飞轮角速度反映同一滤波反馈量。倾倒、非有限值、采样过期和启动渐增保护保留；Mode8仍为独立PWM调试。

## 接线与网络

| 用途 | 当前配置 |
|---|---|
| 行进轮 | P21.5，17kHz PWM；P21.4 DIR，正指令低 |
| 无刷驱动 | UART3 TX=P15.6、RX=P15.7，460800，默认文本指令；兼容A5转速回传 |
| IMU660RB | SPI0 SCK=P20.11、MOSI=P20.14、MISO=P20.12、CS=P20.13 |
| IPS200 SPI | SPI2 SCK=P15.3、MOSI=P15.5、DC=P15.0、CS=P15.2、RST=P15.1、背光=P15.4 |
| WiFi-SPI | SPI3 SCK=P22.3、MOSI=P22.0、MISO=P22.1、CS=P22.2、INT=P15.8、RST=P23.1 |
| K1/K2/K3/K4 | P20.6 / P20.7 / P11.2 / P11.3 |
| 行进轮编码器 | TIM2：计数P33.7、方向P33.6 |

编码器Stop时仍采样；Mode2使用位置、速度串级外环。`WHEEL_ENCODER_SIGN` 默认+1，仅改变观察值方向，不直接改变PWM；实测整车向右移动计数为正，屏幕Pitch减小表示右倾。编码器1024脉冲/圈、编码器4圈对应地轮1圈，当前上升沿1倍频，地轮4096计数/圈。`rpm = wheel_speed_cps * 60 / 4096`，`轮子圈数 = wheel_position_count / 4096`。无需轮径即可实现计数位置闭环；轮径未确认，不换算线速度或距离。

### 地轮编码器观察（Mode2第一步）

烧录后保持Startup / Stop，无需Start；可在Startup / Mode 2..8选2并保持Stop，屏幕提示停车手转；该模式Start会启动电机。屏幕底部`Enc d:… v:…/s E:…`为本窗口增量、计数/秒、错误次数；停车时`P:… S:… OK:a/b`为当前基准后的累计计数、发布序号、速度采样时序有效/位置连续有效。

联网后在配套上位机选择“地轮编码器 · 15通道”或点击“订阅地轮编码器（停车手转）”。新字段为`wheel_encoder_raw/delta`、`wheel_speed_cps`、`wheel_position_count`、`wheel_encoder_seq/dt_ms/valid`、`wheel_position_valid`、`wheel_encoder_errors/sign`（各名称均以`wheel_`开头）。原始16位raw不应用方向符号；增量、速度、累计位置应用符号。

先静置确认序号持续增长，再手动正反转行进轮，检查增量与速度反号、累计位置往返，并测一圈的计数。静止时为0是正常现象，`valid=1`只证明软件采样时序有效，不能证明接线正常。

读取间隔超过10ms或恰好相差半个16位计数范围时，丢弃该段增量、速度无效、错误次数增加。完整速度窗口后速度有效恢复，位置连续有效仍为0；整板复位或停车后Mode2 Start可重建位置基准，再等待完整新窗口。累计位置限制±16777215，越界也锁存位置无效。重建基准保留硬件计数器和错误历史、增加私有基准版本；运行中版本变化会停机。算法要求每次读取间位移小于32768计数，无法检测未观察到的整圈计数器回绕。

PC目标IP以本地 `config.h` 的 `QCAR_WIFI_TARGET_IP` 为准，接收端口 **8081**；模块命令端口 **5001**。WiFi名和密码仍在config中，未改动。上位机采用N个小端float32加尾标 `00 00 80 7F`，文本回执仍以MCAR开头。

支持 LIST?、GET?、SUB、RATE、STREAM 和 `[slider,名称,值]`；STREAM 0只停遥测，不停电机。当前单轴39通道、双轴/Mode8共45通道，前三十四/四十项顺序保留，末尾追加左右转速、速度帧计数、反馈年龄和新鲜度；上限48通道。上位机应通过MCAR通道名称公告同步顺序，旧固定长度34/40配置需重新同步。三轴姿态3通道及28通道诊断配置保留。旧速度环遥测和旧cascade_* PID写入名已移除，不能再用于当前直立控制。保留的可写名见 `wifispi.c` 的 `g_slider_parameters`。

## 无刷驱动通信修复

`QCAR_DRIVER_TEXT_DUTY=1` 为默认：发送 `SET-DUTY,左,右\n` 和 `GET-SPEED\n`，避开原版CYT2BL3驱动把二进制负载0x0A误作文本换行的问题。保留当前飞轮最终方向-1；PWM数值、PID和引脚不变。常见±2800指令已通过原版驱动解析器的端到端回归。驱动板未升级时也可使用这个主控兼容路径；参考驱动支持此文本协议，其他板上固件仍需确认。

每次输出时检查20ms速度请求周期，运行和停车诊断均会补发GET-SPEED，使单独复位驱动后能恢复转速回传。接收端有界读取最多32字节，支持文本 `左转速,右转速` 和A5七字节帧，拒绝格式错误、溢出数字和错误校验。独立1ms按键定时器维护反馈年龄，不依赖控制中断是否启动。

Start后默认遥测保留 `flywheel_left_speed_raw`、`flywheel_right_speed_raw`、`driver_speed_frames`、`driver_feedback_age_ms`、`driver_feedback_fresh`。反馈年龄超过100ms后fresh=0；它表示链路反馈过期，不等同于磁编码器损坏。当前新鲜度为诊断字段，不新增自动启停或改变原控制状态机。

修订的CYT2BL3驱动源码和安装脚本位于 [tools/cyt2bl3_driver](tools/cyt2bl3_driver/README.md)：二进制帧优先按长度/校验接收、逐帧清校验、兼容文本并限制数字长度；左右电机低占空比阶段清过载累计时间。堵转/过载保护及故障锁存保留，主控Stop/Start不清驱动保护。驱动侧修复需用其IAR工程重新编译并烧入驱动板；TC264的HEX只更新主控。

## 参数保存

Storage / Save保存两轴KP、角度KP、输出限幅、目标角速度限幅、方向、机械零点和启动斜坡。使用新的直立专用格式，包含校验和、最后写入的提交标记及写后回读；不再读取视觉参数或旧PID记录。旧格式/损坏记录使用config默认值。I/D及算法选择是编译配置，不在这份RAM参数记录中。

Flash写入和恢复采样必须停车执行。主机测试验证序列化和错误拒绝，不代表已验证实车DFlash硬件。

## 编译、验证

在 ADS 1.10.10 中使用 File → Import → General → Existing Projects into Workspace 导入本目录的 **OneWheelUpright**，选Debug，刷新后构建。工程元数据、TC26B链接脚本和逐飞配置静态库已随仓库保留；内置TASKING需要通过ADS调用。

产物是 `Debug/OneWheelUpright.elf`、`.hex`、`.map`。烧录选本次HEX，芯片和下载接口仍按实物；本轮没有烧录或实车验证。

主机回归：`python tests/run_all.py`（需要GCC）；也可 `python tests/run_all.py --compiler <zig.exe路径>`。测试会显式开启assert，覆盖真实C实现的PID、双轴控制、保护、IMU/EKF、Flash、按键、菜单、SPI及WiFi。验收边界见 [VALIDATION.md](VALIDATION.md)。来源与版权见 [THIRD_PARTY.md](THIRD_PARTY.md)。

## 本次云端合并

基于上游498822e，保留本机飞轮36/40/160/8000、地轮30/40/100/4000、机械零点X4.2°/Y1°、本机网络配置及Mode2位置保持。采用云端5ms飞轮角速度低通，角度低通保持旁路。保留Mode8阶梯测试和DTRACE工具；驱动记录需CYT2BL3配套固件，主控更新不等于驱动已更新。驱动实测报告来自上游其他设备。

IMU现增加观测降权、零偏学习限速、持续失信保护(-5)和停车静稳恢复；上位机新增47通道IMU可信度方案。恢复后需主动Start，细节见[IMU.md](IMU.md)。
