# OneWheel 当前代码：从上电到控制、通信与预留功能

> **版本说明：** 本页保留修复前的完整调用链讲解。新增 CPU1 独立输出、故障锁存、反馈有效期和换向状态机后，涉及输出/保护的旧描述以 [SAFETY_FIXES_20261022.md](SAFETY_FIXES_20261022.md) 为准。逐函数与宏索引已更新至修复后源码，行号也已更新。

本文对应 2026-10-22 复审的实际工作树，入口为 `Seekfree_TC264_Opensource_Library/user/`，不是旧提交。固件未在本轮修改。问题清单见 [CODE_REVIEW_20261022.md](CODE_REVIEW_20261022.md)。

覆盖应用层所有 111 个实际函数，以及配置、共享状态、中断和实际依赖的驱动机制；逐函数索引与配置值见 [CODE_MECHANISM_INDEX.md](CODE_MECHANISM_INDEX.md)。注释掉的四元数代码、未调用函数、空文件、关闭的相机路径也单独说明。未使用的整个第三方 SDK 不属于本车运行机制的承诺范围。

## 1. 文件如何分工

| 文件/目录 | 实际职责 |
| --- | --- |
| `user/cpu0_main.c` | CPU0 调用 `app_cpu0_init()`，永远循环 `app_cpu0_poll()` |
| `user/cpu1_main.c` | CPU1 调用 `app_init()`，永远循环 `app_poll()` |
| `code/app.c/.h` | 初始化排序、双核交接、按键、遥测、启动诊断、相机和显示的封装 |
| `code/config.h` | 编译开关、周期、默认速度、轴映射、PID、保护、引脚、调参范围 |
| `code/posture_control.c/.h` | IMU 处理、启动/停止请求、1ms 控制调度、两组电机的控制组合 |
| `code/balance_control.c/.h` | 控制结构体、9 个 PID、位置式/增量式计算、互补滤波、PID 状态复位 |
| `code/motor.c/.h` | 行进轮 PWM+DIR、编码器初始化、死区补偿与输出保护 |
| `code/small_driver_uart_control.c/.h` | 两个飞轮的串口占空比指令、测速请求、速度帧解析 |
| `code/wifi_link.c/.h` | 双核遥测发布、UDP 连接状态机、数据/日志/图像封包、文本调参 |
| `code/wifi_link_io.c/.h` | WiFiSPI 运行期的分步事务、INT 忙闲检查、超时与耗时统计 |
| `code/wifi_link_config.h` | 私有 WiFi 与端点配置；不在本文展开密码、不进入审查副本 |
| `code/camera.c/.h` | 视觉处理，以及当前仍在使用的速度斜坡 `retarder()` |
| `code/user_value.c/.h` | 全局变量、未接入的 Flash 参数读取、文本输出辅助函数 |
| `code/four_posture.*` | 全文件已注释的旧四元数算法，当前没有运行 |
| `code/picture_process.*` | 空预留文件，没有图像算法实现 |
| `user/isr.c` / `isr_config.h` | 中断入口、服务核和优先级；包括当前未开启的设备回调 |
| `libraries/` | 逐飞封装和 Infineon iLLD，提供实际寄存器/外设操作 |
| `tools/monitor_onewheel.py` | 启动独轮上位机模式并指定本项目配置文件 |
| `tests/host` / `tests/check_wifi_native.py` | 用真实应用 C 配合模拟外设做回归测试，不在板上执行 |

`motor.c` 里还保留“步进”的命名，但当前实现明确是 17kHz PWM+DIR；不能用文件注释推断实际电机种类。

主要结构体也有明确职责：`pid_cycle_struct` 的 p/i/d 是可跨核修改的增益，i_value/p_value_last/incremental_data 是算法状态，i_value_max/i_value_pro/out_max 是限幅与比例，out 是输出；`cascade_common_value_struct` 保存传感器指针、融合权重、调用周期、内部状态及机械零点。指针在初始化时绑定，但现行反馈调用直接传入 ROLL/PITCH 宏值；new_mechanical_zero 没有接入。`cascada_value_struct` 把这几组融合/PID组合在一起。`small_device_value_struct` 是7字节收发缓冲、接收计数、校验与左右速度；`BeaconInfo` 是主/次坐标、距离、亮度、权重与未完整实现的status/area。

## 2. 当前打开和关闭的功能

`config.h:8-48` 当前为：

```c
APP_DRIVE_ENABLE=1;
APP_WIFI_ENABLE=1;
APP_KEYS_ENABLE=1;
CONTROL_ROLL_ENABLE=1;
CONTROL_PITCH_ENABLE=1;
CONTROL_MOTOR_ENABLE=1;
CONTROL_YAW_ENABLE=0;
APP_CAMERA_ENABLE=0;
APP_CAMERA_DISPLAY_ENABLE=0;
APP_IPS200_ENABLE=0;
APP_WIFI_IMAGE_ENABLE=0;
APP_WIRELESS_UART_ENABLE=0;
APP_CSV_DEBUG_ENABLE=0;
```

以上是便于阅读的配置值摘录，不是实际 C 赋值语句；原文件用 `#define`。当前会初始化电机、飞轮串口、IMU、按键和 WiFi；不会初始化摄像头或 IPS200，也不会上传图像。部分视觉函数仍编译存在，不能据此说它们在运行。

当前不是“速度为 0 就完全不输出”：`wifi_target_speed` 默认 0，表示不主动行进；启动平衡后，姿态偏差仍可能产生飞轮和行进轮输出。

## 3. 上电：两核何时开始、为什么要交接

底层 `IfxCpu_CStart0.c:157-189` 先建 CPU0 上下文区、关闭 CPU/安全看门狗、做 C 运行时变量初始化，然后启动 CPU1，最后才调用 `core0_main()`。因此 CPU1 有可能比 CPU0 的应用初始化更早执行。

CPU0 `app_cpu0_init():62-80` 顺序：

```c
clock_init();
debug_init();
interrupt_global_enable(0);
if (APP_WIFI_ENABLE) { wifi_link_init(); }
// 设置 CPU0 诊断周期，必要时输出固件编译时间
__dsync();
app_cpu0_ready = 1;
__dsync();
```

`clock_init()` 会关中断、关看门狗、设置/查询 CPU 时钟并配置 STM 延时服务。`debug_init()` 配置 UART0，115200，TX P14.0、RX P14.1，开启其接收中断与调试缓冲。`wifi_link_init()` 初始化内存与状态机，不在此处连热点。

CPU1 `app_init():90-145` 先禁用本核看门狗，写阶段 1，等待：

```c
while (!app_cpu0_ready) { __dsync(); }
```

这个等待不调用延时函数；CPU0 尚未建立延时中断时也能等待。`volatile` 让共享标志被实际读取，`__dsync()` 用于跨核内存访问排序。它不是互斥锁，也不自动保护整个控制结构体。

两核共享数据使用全局 dsram1 地址（映射中是 `0x600...`）。中断表也共享：LSL 把 `__INTTAB_CPU0`、`__INTTAB_CPU1` 同指 INTVEC0。因此 ISR 宏中的向量表参数 0 与某个外设服务 CPU1 并不天然矛盾。

## 4. CPU1 的全部初始化阶段

| `app_cpu1_stage` | 实际步骤 | 分支或结果 |
| --- | --- | --- |
| 1 | 等待 CPU0 初始化完成 | 等不到就不会初始化电机/按键/PIT |
| 2 | `Motor_init()` | TIM6 编码器，行进 PWM=0，DIR 前向 |
| 3 | `small_driver_uart_init()` | UART3、RX 中断、清协议状态、发送飞轮零输出和测速请求 |
| 4 | `balance_cascade_init()` | 初始化滤波器、9 个 PID 及参数备份 |
| 5 | IMU 初始化，最多 3 次，失败间隔 50ms | 全失败则 `app_imu_ready=0`；成功才初次读加速度/陀螺仪并 `icm_init()` |
| 6 | `key_init(10)` | 当前按键开关为 1 |
| 7 | 可选无线串口、IPS200、摄像头初始化 | 当前均关闭；相机返回值单独记录 `camera_ready` |
| 8 | `pit_ms_init(CCU60_CH0,1)` | 由 CPU1 配置，但控制 ISR 服务核是 CPU0 |
| 9 | PIT 初始化返回，记录启动说明 | 随后才进入 CPU1 主循环 |

进入阶段 5 不表示“IMU 已失败”；可能正在等待、自检、重试或初次采样。自检失败正常返回后仍会继续到阶段 9，只是禁止 K1 启动。阶段 9 也不单独证明 CPU1 后续每轮都正常，需看 `[APP]` 日志持续变化。

预热计数从 PIT 开始自增，而非从按下 K1 开始；早于 2s 按键会被拒，之后需要再次按键，不会自动排队启动。

## 5. 三个执行位置与三套时间语义

| 执行位置 | 节拍/时间 | 做什么 |
| --- | --- | --- |
| CPU0 主循环 | 每轮工作后延时 300μs；WiFi INIT/SOCKET 仍可能阻塞 | 诊断、WiFi 状态机与 SPI 收发 |
| CPU1 主循环 | 每轮工作后延时 10ms，实际周期略长 | 按键、发布遥测快照、状态日志 |
| CPU0 的 CCU60_CH0 ISR | 请求周期 1ms，优先级 30 | `system_count`、传感器、PID、飞轮和行进输出 |

CPU0 的 WiFi 时间 `wl_now_ms`、CPU1 的后台时间 `cpu1_wall_ms` 都通过 STM0 硬件计数差计算，不依赖控制 tick。它们各自保存上次计数和不足 1ms 的余数，支持正常连续轮询下的低 32 位回绕。

`system_count` 则只在 `pit_isr_callback()` 中加 1，是“执行的控制回调次数”。中断错过/阻塞时不能把它当作精确墙上时间。它用于控制分频、2s 预热、遥测采样计数和 yaw 积分时间。

SDK 的 `system_delay_ms/us` 另有机制：根据调用核选 STM0/STM1；已开中断时设置比较器并等相应 ISR 清标志，关中断时用计数忙等。STM0 优先级 220 路由 CPU0；STM1 优先级 221 路由 CPU1。所以后台调度虽不依赖 PIT，主循环末尾的延时仍依赖 SDK 的延时机制和该核执行条件。

## 6. CPU1 主循环每一步

`app_poll():238-248` 实际代码：

```c
app_new_frame = 0;
app_buttons_poll();
// app_camera_poll();
// app_camera_display_poll();
app_telemetry_poll();
app_status_log_poll();
system_delay_ms(APP_MAIN_LOOP_DELAY_MS);
```

`app_buttons_poll()`、`app_telemetry_poll()`、`app_status_log_poll()` 分别读取 `app_cpu1_now_ms()` 并检查独立最小间隔 10ms、10ms、1000ms；到期把各自 last 时间设为当前时间，不补执行错过的多次周期。主循环耗时过大仍会增加调度延迟。

### 6.1 按键

K1=P20.6；K2=P20.7；K3=P11.2；K4=P11.3。逐飞按键默认释放高电平，扫描累计按压次数，10ms 消抖、1000ms 长按，`key_init()` 的参数用于把时长转换成扫描次数。

应用只取 `KEY_SHORT_PRESS`：K1 发启动请求，K2/K3/K4 任一发停止请求；先清按键状态，再优先处理 stop，因此同一次扫描 start+stop 同时出现时停止优先。

短按事件在释放时产生，不是电平一低立即停车。每轮 `key_clear_all_state()` 也会清除 LONG 状态，持续按下时应用不处理 LONG，释放后仍可能得到 SHORT。当前没有独立长按功能或硬件急停输入。

启动拒绝时只输出通用提示 `[APP] start refused...`，提示没有细分到具体哪个条件。

### 6.2 遥测和状态

`app_telemetry_poll()` 比较 `run_flag` 与 `last_reported_run`，变化时写 RUNNING/STOPPED 日志；然后调用 `wifi_link_cpu1_service(app_new_frame)` 更新共享快照。这一步不执行 WiFi SPI。

CSV 调试仅在 `APP_CSV_DEBUG_ENABLE && APP_WIRELESS_UART_ENABLE` 时调用 `send_two_floats()`；两个开关当前都为 0。

`app_status_log_poll()` 每约 1 秒生成：

```text
[APP] st=9 tick=1234 dtick=1000 dms=1000 imu=1 run=0
```

`st` 为 CPU1 初始化阶段，`tick` 是本次采到的控制计数，`dtick` 是上次日志以来控制计数增量，`dms` 是同区间 STM 时间。持续 `dms≈1000`、`dtick=0` 表示后台仍活着而控制心跳没推进，但不能直接指定为“核路由错误”。该日志由 CPU1 环形队列最终经 WiFi 发送，不直接输出 UART0。

## 7. 启动、运行和停车的状态交接

`control_request_start():82-90` 先检查：

```c
if (!APP_DRIVE_ENABLE || !app_imu_ready ||
    system_count < APP_CONTROL_WARMUP_MS || run_flag || stop_pending)
    return 0;
```

满足后复制当前 `stop_epoch` 到 `start_epoch`，再置 `start_pending=1`，返回“请求已接受”。这不表示电机已运行；实际启动在下一次 ISR 处理。

`control_request_stop()` 立即把软件 `run_flag` 置 0，增加 `stop_epoch` 并置 `stop_pending`。ISR 最先处理停止请求，调用 `control_stop()`：

- 清 `run_flag/start_pending/stop_pending/was_running`。
- 将 `expect_speed=0`，复位 `retarder` 内部状态。
- 清 9 个 PID 的积分、历史误差、增量历史和输出。
- 飞轮命令与记录值归零，行进 PWM 指令归零。

它不重置上位机设定的 `wifi_target_speed`，也不恢复 PID 默认参数、不清姿态滤波内部状态。再次启动时斜坡会从 0 追向上次保留的目标速度；整机重启才恢复编译默认值。

ISR 接受 `start_pending` 时先检查 epoch 是否仍匹配、是否又有停止请求；不匹配则停止。这用于拦截被新的 stop 作废的旧 start。它不是覆盖所有执行交错的完整互斥协议，最终输出仍依赖控制 ISR 执行。

## 8. IMU：到底采的是什么

`imu_data_get():18-35` 每次读 IMU660RB 的加速度和陀螺仪寄存器，然后：

```c
gyro[0] = (int16)(imu660rb_gyro_x - IMU_GYRO_BIAS_X);
gyro[1] = (int16)(imu660rb_gyro_y - IMU_GYRO_BIAS_Y);
gyro[2] = (int16)(imu660rb_gyro_z - IMU_GYRO_BIAS_Z);
// |gyro| < 5 的轴置零
alpha = 0.001f / (0.0079617834f + 0.001f);
icm_data_lpf.accX += alpha * (acc[0] - icm_data_lpf.accX);
// 其余 5 个通道同样处理
```

默认零偏 X=7、Y=-10、Z=4；减去负数等于加。低通 alpha 约 0.1116。`ICM_DATA` 字段实际为 int16，因此每步存回会截断，不能把它看成完整浮点滤波器。

`icm_init()` 在初始化成功后把首次读数写入 `icm_data_lpf` 和 `re_icm_data`；其陀螺初值是未减零偏的首次值，后续周期再逐渐收敛到校正后的数据。

SPI0=10MHz，SCLK P20.11、MOSI P20.14、MISO P20.12、CS P20.13。配置 `0x3C/0x5C` 对应加速度 ±8g、约 52Hz；陀螺 ±2000°/s、约 208Hz。驱动换算加速度除以 4098，陀螺除以 14.3。控制每毫秒读寄存器，不代表每毫秒都有一份新传感器样本。

车体控制轴映射由宏决定：

| 控制量 | 当前传感器通道 |
| --- | --- |
| ROLL_GYRO_RAW | gyroY |
| ROLL_ACCEL_RAW | accZ |
| PITCH_GYRO_RAW | gyroZ |
| PITCH_ACCEL_RAW | accY |
| YAW_GYRO_RAW | gyroX |

这些对应你的安装方向，需实测。WiFi 显示的传感器坐标系 roll/pitch/yaw 与这些车体控制通道不是同一套命名，不能直接用“上位机 roll 正向”推断车体控制反馈符号。

## 9. 互补滤波和控制器的单位

`first_order_complementary_filtering()` 不是四元数/欧拉角解算：

```c
gryo_temp = gyro_raw_data * gyro_ration;
acc_temp = (acc_raw_data - angle_temp) * acc_ration;
angle_temp += (gryo_temp + acc_temp) * call_cycle;
filtering_angle = angle_temp + mechanical_zero;
```

roll 权重为 gyro 4、acc 6；pitch 为 gyro -6、acc 4；每 5ms 调用一次，call_cycle=0.005。机械零点默认 roll=30、pitch=-100。得到的是原作者控制器内部尺度，并非“度”。保护阈值 1500/2000 也不是角度 1500°/2000°。

初始化把 `angle_temp` 和 `filtering_angle` 都设为 `-mechanical_zero`；首次滤波后再加机械零点。更改零点会改变之后计算的输出偏移，不会重新生成传感器姿态。

函数末尾正向超过 4000 改输出为机械零点的行为存在缺陷，见 F1；不能把该分支理解成可靠倾倒保护。

## 10. 1ms 控制中断：完整执行顺序

`user/isr.c:43-49` 开中断嵌套、清 CCU60_CH0 标志，再调用 `pit_isr_callback()`。服务 CPU0、优先级 30；其他更高优先级中断可打断它。

`posture_control.c:131-194` 的顺序不能调换着理解：

1. `system_count += 1`。
2. 有 `stop_pending` 时先 `control_stop()`，之后本次回调仍可继续采样。
3. `!app_imu_ready` 时，必要时停止并直接返回；不再访问 IMU。
4. `imu_data_get()`，更新原始值/低通值。
5. `control_feedback_poll()`：每 20ms 请求飞轮速度、取行进编码器、累计计数、清编码器、算 `car_speed`；每 5ms 更新 roll/pitch 互补滤波。停车时也做这些反馈更新。
6. `APP_DRIVE_ENABLE=0` 时必要时停止并返回；观察模式仍可采集反馈。
7. 有启动请求就检查 epoch、清 PID/斜坡状态并置 `run_flag=1`。
8. 没运行或尚未预热完成时返回；如果之前运行过而现在停了，补一次 `control_stop()` 清输出。
9. 置 `was_running=1`，更新 `expect_speed=retarder(wifi_target_speed,wifi_ramp_step)`。
10. 每 20ms 更新速度环，每 5ms 更新角度/视觉误差环；角速度环每次调用。
11. 先检查 roll/pitch 倾倒与行进组合输出，超限则停止并返回。
12. `dynamic_value_control()` 输出飞轮，再 `bujin_motor_contron()` 输出行进轮；两个函数内部还有自身的停止/限幅/保护分支。

PID 分频通过 `system_count % 周期 == 0`，不是分别创建 5ms 和 20ms 硬件中断。

## 11. 9 个 PID 与最终输出关系

### 11.1 两种 PID 算法

位置式 `pid_control():124-135`：

```c
error = target - real;
i_value += error * i_value_pro;
i_value = limit(i_value, -i_value_max, i_value_max);
difference = error - p_value_last;
out = limit(P*error + I*i_value + D*difference, -out_max, out_max);
p_value_last = error;
```

增量式 `pid_control_incremental():162-171`：

```c
error = target - real;
delta_p = error - previous_error;
delta_d = error - 2*previous_error + older_error;
out += P*delta_p + I*error + D*delta_d;
out = limit(out, -out_max, out_max);
// 更新两次误差历史
```

增量式的 `i_value=error` 只是记录，不使用位置式的积分限幅和 `i_value_pro`。因此即使配置了所有 `PID_*_I_LIMIT/I_SCALE`，角速度增量环也不会按这些值计算积分。两个函数都没有自动按真实 dt 缩放 I/D，采样周期是参数含义的一部分。

`balance_cascade_reset()` 只清状态，不清 P/I/D 与限幅参数；`balance_cascade_resave` 是初始化时的结构体副本，当前没发现恢复调用。

### 11.2 各环串联与并联

| 环 | 周期 | target / real | 输出交给谁 | 当前开关 |
| --- | --- | --- | --- | --- |
| roll speed | 20ms | 0 / `car_speed` | roll angle 的目标 | 开 |
| roll angle | 5ms | speed.out / `roll_filter - (相机开启?k1*erro_yawan:0)` | roll rate 目标 | 开 |
| roll rate | 1ms，增量式 | angle.out / `ROLL_GYRO_RAW` | 两飞轮的 roll 分量 | 开 |
| pitch speed | 20ms | `expect_speed` / `encoder_data_1` | pitch angle 目标 | 开 |
| pitch angle | 5ms | pitch_speed.out / pitch_filter | 取负后作为 pitch rate 目标 | 开 |
| pitch rate | 1ms，增量式 | `-pitch_angle.out` / `PITCH_GYRO_RAW` | 行进输出的一部分 | 开 |
| motor | 20ms，位置式 | `expect_speed` / `encoder_data_1` | 与 pitch_rate.out 相加 | 开 |
| yaw error | 5ms | 0 / `wifi_vision_erro(erro)`，相机关则反馈 0 | 取负后作为 yaw rate 目标 | 关 |
| yaw rate | 1ms，位置式 | `-yaw_error.out` / `YAW_GYRO_RAW` | 两飞轮的 yaw 分量 | 关 |

pitch 三环和独立 motor 速度 PID 并行推动同一个行进轮，输出最终相加。这不是互斥的两种模式；两套增益可能互相影响，调参应分别验证方向与响应。

完整默认值（位置式最后两列分别为积分存储限幅与积分累加比例；增量式不用这两列）：

| 环 | P | I | D | I_LIMIT | I_SCALE | OUT_LIMIT |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| roll rate | 0 | 0.13 | 0 | 1000 | 0.1 | 10000 |
| roll angle | 0.85 | 0 | 0.08 | 4000 | 0.05 | 8000 |
| roll speed | 0.2 | 0.001 | 0 | 4000 | 0.5 | 1500 |
| pitch rate | 0 | 0.023 | 0 | 1000 | 0.1 | 10000 |
| pitch angle | 0.58 | 0 | 0.003 | 4000 | 0.05 | 10000 |
| pitch speed | 5.8 | 0 | 0.001 | 4000 | 0.5 | 3500 |
| motor | 11.1 | 0 | 0.29 | 5500 | 0.05 | 8000 |
| yaw rate | 0.62 | 0 | 0 | 1000 | 0.1 | 10000 |
| yaw error | 9.4 | 1.55 | 0 | 8500 | 0.05 | 8000 |

上表是代码默认值，不代表已经适合你的车。当前 yaw 相关默认参数虽然初始化、可回读，但其计算和输出分量关闭。

## 12. 斜坡、编码器与速度单位

`camera.c` 的 `retarder()` 仍是现行控制功能：每次把内部 `retarder_output` 向输入移动一个 `unit`，接近后限制为输入，避免越过目标。正负方向都支持。`retarder_reset()` 清这个持久状态。

默认：`wifi_ramp_step = 160 / 5333.333333 ≈ 0.03`，每次 1ms 控制调用改变约 0.03 个“编码器计数/20ms”。到目标 160 约需 5333 次调用；`ramp_ms` 是以参考值 160 定义的时间，并非到任意目标都固定用该时长。

行进编码器是 TIM6 的脉冲+方向接口：P20.3 计数，P20.0 方向。`control_feedback_poll()` 每 20ms：

```c
encoder_data_1 = (int16)(encoder_get_count(ENCODER_1) * ENCODER_DIRECTION_SIGN);
wifi_encoder_total += (uint32)(int32)encoder_data_1;
encoder_clear_count(ENCODER_1);
car_speed = (left_flywheel_speed - right_flywheel_speed) / 2;
```

`encoder_data_1` 单位是计数/20ms；`car_speed` 则是飞轮驱动反馈的组合量，不能当作行进轮 cm/s 或默认等同 RPM。串口测速先发请求再读最近一次接收值，不会同步等待这一次回复。

`wifi_encoder_total` 用 uint32 保存有符号增量的模运算累计值，上位机按有符号 32 位解释。`WIFI_LINK_CM_PER_COUNT=0` 表示没有距离标定；因此 cm/s 换算为零，`cruise_cms` 命令被拒为 UNCALIBRATED。

标定后 `cm/s = 计数/20ms × 50 × cm/count`；固件反算 `cruise_cms` 也是这个关系。

## 13. 两组执行器、死区和保护

### 13.1 飞轮

`dynamic_value_control():93-111`：

```c
roll = CONTROL_ROLL_ENABLE ? roll_rate.out : 0;
yaw  = CONTROL_YAW_ENABLE  ? yaw_rate.out  : 0;
left  = limit(-roll + yaw,-10000,10000) * MOMENTUM_LEFT_SIGN;
right = limit(-roll - yaw,-10000,10000) * MOMENTUM_RIGHT_SIGN;
```

符号默认 left=+1、right=-1；当前 yaw 关闭，所以两飞轮用相反的驱动方向表达同一 roll 控制作用。没有运行时直接发左右零输出。

若 `abs(roll_filter)>1500` 或 `abs(roll)>10000` 则停止。roll PID 本身上限也是 10000，所以正常饱和恰好等于 10000 时不会触发后一条件。

### 13.2 行进轮

```c
travel_command = pitch_rate.out + motor.out;
bujin_motor_contron((int)travel_command);
```

`run_flag=0`、`run_moter=0` 或 pitch/motor 两个开关都为 0 时，行进输出归零；`run_moter` 只是行进轮的本地使能，不关飞轮。

保护先检查 pitch 2000、roll 1500 与原始 duty 8000。然后乘 `MOTOR_DIRECTION_SIGN`；正数加 400，负数减 400，零不加；正方向 DIR=低，负方向 DIR=高。最终幅值写 P21.5 的 ATOM0_CH3，DIR 写 P21.4。例：+1000→PWM 1400/DIR低，-1000→PWM 1400/DIR高，0→PWM0。

`bujin_last_duty` 保存应用了方向符号和补偿后的带符号指令，用于遥测，不是编码器实测。

`motor_write_output()` 只在方向改变时先写零 PWM、改 GPIO，再写新幅值；没有实际等待 PWM 影子寄存器生效，见 F4。`MAX_DUTY=50` 是头文件遗留宏，未在当前输出链使用，不能理解成已经有 50% 总限幅。

### 13.3 保护的完整层次与缺项

- 编译总开关 `APP_DRIVE_ENABLE`：阻止本地启动，ISR 仍可采样。
- 上电 IMU 就绪与 2s 预热：未满足则不接受启动。
- 停止请求/epoch：停止优先，旧启动请求可被作废。
- ISR 组合输出前的 roll/pitch/行进指令检查：避免已输出一组后才发现另一组超限。
- 各执行器函数内部检查与飞轮限幅、行进死区补偿。
- PID 输出限幅及位置式积分限幅。

代码没有完成控制心跳超时禁能、飞轮反馈过期处理、WiFi ALIVE 超时停车，也没有电流/温度/驱动故障引脚保护。`app_imu_ready` 是初始化结果，不是持续运行健康度。不要把这些未实现机制当作已有功能。

## 14. 无刷驱动串口的全部协议

UART3，460800，TC264 TX P15.6 / RX P15.7；UART3 RX ISR 调 `uart_control_callback()`。头文件 `SMALL_DRIVER_RX/TX` 名称看似反向，但它们作为 `uart_init` 的第三、第四实参传入后，实际仍是 TX=P15.6、RX=P15.7，不应仅凭宏名改线。

每帧 7 字节：

| 偏移 | 内容 |
| --- | --- |
| 0 | 0xA5 帧头 |
| 1 | 0x01 设置占空比，0x02 查询/回复速度 |
| 2、3 | 左侧 int16 高、低字节，大端 |
| 4、5 | 右侧 int16 高、低字节，大端 |
| 6 | 前 6 字节求和并保留低 8 位 |

`small_driver_set_duty()` 组 0x01 帧，没有底层自动限幅，上游负责范围；`small_driver_get_speed()` 发送固定 `A5 02 00 00 00 00 A7`。

RX 每次取一个字节、累计 7 字节后检查 A5、和校验和功能字。0x02 的两个大端数写回 int16，因此能表达负值；错帧或处理完成后清接收数组/计数。只有当前缓冲首字节不是 A5 时，新 A5 才重置计数，丢字节恢复不是完整滑窗搜索，也没有超时或反馈新鲜度判断。

`small_driver_init()` 清收发缓冲、接收计数、校验和、两个速度。串口初始化打开 RX 中断后再清这些数据，随后发零占空比和测速请求。

逐飞 UART 发送逐字节等待 TX FIFO 清空，属于轮询发送；控制 ISR 里每 1ms 会发送飞轮占空比，20ms 时还发测速请求。不存在已实现的无刷发送队列/DMA；实际 ISR 耗时与接收延迟应上板测量。

## 15. CPU0 主循环与 WiFi 五状态机

`app_cpu0_poll()` 先加 `app_cpu0_poll_count`，运行启动诊断，再服务 WiFi；WiFi 关闭时只延时 300μs。

`app_boot_diagnostics_poll()` 直接读 STM0，每约 1000ms 写 UART0，并追加同一行到 CPU0 WiFi 日志：

```text
[BOOT] c0_polls=... c1_stage=... tick=... imu=... run=... wifi=...
```

第一轮也会输出。`wifi=1` 只表示状态机处于 RUN，不证明电脑已收包。固件编译日期/时间的 CPU0 ready 横幅只直接走串口；`[NET]` 日志也会镜像到 UART0。CPU1 的 `[APP]` 行不直接写 UART0。

`wifi_link_service():873-959`：

| 状态 | 行为 | 转移 |
| --- | --- | --- |
| BOOT | 等 STM 时间达到 300ms | INIT |
| INIT | 重新初始化运行期 IO 状态；调用 SDK `wifi_spi_init(SSID,PASSWORD)` | 成功 SOCKET；失败 BACKOFF→INIT |
| SOCKET | SDK 配置 UDP，远端主机/端口与本地端口来自私有配置 | 成功 RUN；失败计数，6 次以内重试 SOCKET，耗尽改重试 INIT |
| RUN | 分步运行 `wl_run()` | IO 超时/错误或 INT 长忙则 BACKOFF→INIT；切换 host 则 SOCKET |
| BACKOFF | 等 5000ms | 回到记录的下一状态 |

每轮末尾延时 300μs。INIT 和 SOCKET 仍是 SDK 阻塞函数，RUN 才是协作分步；初始化卡住时 CPU0 的启动诊断也可能暂停，但高优先级中断是否继续取决于卡住位置。

WiFi SPI3=30MHz，SCK P22.3、MOSI P22.0、MISO P22.1、CS P22.2、INT P15.8、RST P23.1。INT 高表示模块可处理事务；并不是通过 P15.8 的 ERU ISR 接收数据，当前 RUN 在主循环轮询它。

私有配置含 SSID、密码、HOST_IP、HOST_PORT、LOCAL_PORT。HOST 是电脑目的地址；LOCAL 是模块端口，两者没有必须相同的要求。网络配置与应用 `config.h` 分离是为了私有凭据不提交 Git。

## 16. RUN 状态的优先级、队列和提交机制

`wl_run():804-839` 优先级：

1. IO 已忙：只推进 `wl_io_service()`，本轮返回，避免复用正在发送的 `wl_packet`。
2. 有新 host，且 CPU0 日志队列清空：切主机、进入 SOCKET。先让旧地址有机会收到 host 设置应答。
3. 模块 INT 低：记录首次忙时间；持续 250ms 则重连；否则直接让出。
4. 到 5ms 接收周期时先启动一次 RX。
5. 否则，有日志且距上次日志 20ms 时发日志。
6. 否则，距上次数据 50ms 时发遥测，即理想约 20Hz。
7. 否则，图像开关开启才推进一片图像。

CPU1 每约 10ms 更新快照，不等于网络每 10ms 发包；网络优先级、SPI 握手、日志流量还会降低实际发送频率。

`wl_send_wire()` 只向 IO 提交指向 `wl_packet` 的发送请求，不是已成功。IO 报 SENT 后 `wl_tx_committed()` 才：

- LOG0：从 CPU0 队列移除本次长度并把余下字节前移。
- LOG1：推进 CPU1 环形队列 tail。
- IMAGE：推进片号，整帧结束才释放 `frame_ready`、更新时间。
- DATA：没有需要消费的日志或图像状态。

错误时不会提前消费日志；会清 pending、重置图像片号/命令半行并退避重连。因此失败后可以重发日志、从第一片重发相同图像，而不是丢掉尚未确认的内容。

CPU0 日志是 2048 字节线性队列，满后追加截断；每条格式化临时文本上限 128 字节。CPU1 日志是 1024 字节单生产者/消费者环形队列，写 head、读 tail 用 DSYNC 排序，满时丢弃后续文本；自动补 `\n`。单次日志包最多 900 字节，CPU0 应答优先，CPU1 日志可能延迟；没有无损日志保证。

`wl_host_seen` 在 ALIVE 时被置位，但没有参与超时判定；`wl_tx_fail` 没有形成有效的运行期失败计数逻辑。不能把变量存在解释成“已有心跳断连停车”。

## 17. WiFiSPI 的分步事务与超时边界

`wifi_link_io.c` 有 IDLE、DATA、FLUSH、ACK、RX、DRAIN 六状态。

- `wifi_link_io_send()`：只接受空闲状态、非空数据与合法长度，记住地址/长度/起点，进入 DATA。
- DATA：以 `WIFI_SPI_DATA` 提交载荷，通常进入 FLUSH。
- FLUSH：以 `WIFI_SPI_UDP_SEND` 触发模块发送，通常进入 ACK。
- ACK：发一次无载荷查询，收到 OK 才返回 SENT。
- RX：查询接收；START 表示还有数据，继续 RX；END 或 OK 才完成。
- DATA/FLUSH 若同时带回接收 START，进入 DRAIN，排出收到的多片后回到此前记录的下一发送阶段。

每次 `poll()` 最多一个拉低/拉高 CS 的 SPI 事务；INT 低就让出。每阶段或 RX/DRAIN 连续过程超过 250ms 返回 ERROR；发送正常跨阶段可更新超时起点，连续接收流保留原期限，防止无限收包拖住发送。

`io_exchange()` 先全双工交换头，再按协议交换固定 32 字节接收窗口；发送长度超过 32 时，再写余下发送数据。模块返回长度超过 32 被当错误，实际收到的命令字节立即交给 `wl_command_bytes()`。

last/peak SPI 时间由 STM 计数差算成 μs，发进遥测。它测的是当前 SPI 事务成本，不是完整控制 ISR 最坏耗时，也不是 UDP 到电脑的往返延迟。

250ms 超时在两次 poll 之间判断。底层 `spi_transfer_8bit()` 本身仍有无超时的寄存器等待；若卡在该底层调用里，不能声称外层 250ms 一定会救回来。

## 18. 双核遥测快照与物理姿态显示

CPU1 的 `wifi_link_cpu1_service():962-1016`：

1. 序列号加 1 变奇数，DSYNC。
2. 将低通陀螺转 °/s、低通加速度转 g。
3. 加速度模长不接近零时，按传感器坐标算 `roll=atan2(ay,az)`、`pitch=atan2(-ax,sqrt(ay²+az²))`，转度。
4. 首次之后按 `gyroZ × system_count时间差` 积分 yaw，并 `fmod(...,360)`；负值可保留为负角。
5. 更新视觉误差、指令展示值、视觉事件与帧间隔/帧数。当前关闭视觉，相关误差/事件为 0。
6. DSYNC、序列号加 1 成偶数、DSYNC，再置 `cpu1_ready=1`。
7. 如果打开图像且 CPU0 请求、CPU1 有新图，就复制整幅二值图到 `wl_frame`，发布帧号并释放请求。

CPU0 `wl_send_data()` 开始先检查 `cpu1_ready` 与序列号偶数；读完数据后再比较序列号，变化就放弃本次封包。保护覆盖共享发布结构，不自动覆盖 CPU0 ISR 同时变化的所有控制字段，也不保证 CPU1 从 IMU 六轴取得完全同一瞬间的值。

`cpu1_ready` 一旦置1，CPU1 后来停止更新时不会自动清零。CPU0 仍可能不断重发最后的快照、递增包序号；所以“有UDP包”不等于“控制与采样还在推进”，应结合 tick、dtick 和状态日志判断。WiFi 运行期重连也不会自动清 PID、目标速度或发布停止请求。

上位机的物理姿态角、传感器 XYZ，与扩展区的车体 raw 控制量并列存在：调平衡应看对应 raw 反馈/零点/输出，不要把两者当同单位。特别是显示 yaw 积分的是传感器 gyroZ，而当前车体 `YAW_GYRO_RAW` 映射 gyroX。

## 19. 数据包、每个字段和状态位

所有整数/float 字节编码为小端。`wl_put_u16/u32/float()` 完成编码；float 先 memcpy 到 uint32，避免类型别名读取。

外层固定 8 字节：`MUDP`（4）、版本 1（1）、类型（1）、负载长度 u16（2）。类型 1=遥测，2=日志，3=图像。遥测负载是 **228 字节**，整个 UDP 载荷 236 字节；头文件写“148 字节参数包”的旧注释没有跟上 OWHL 扩展。

以下偏移相对负载起点：

| 偏移 | 字段与含义 |
| --- | --- |
| 0/4/8 | float：传感器坐标 roll/pitch/yaw，度 |
| 12/16/20 | float：gx/gy/gz，°/s |
| 24/28/32 | float：ax/ay/az，g |
| 36/40/44/48 | 四个累计编码器槽位；第一个是行进轮，其余置 0 |
| 52/56 | uint32：system_count、包序号 |
| 60/64/68/72 | float：目标速度槽；第一个为 expect_speed，其余 0 |
| 76/80/84/88 | float：行进速度、左飞轮速度、右飞轮速度、car_speed |
| 92/96/100/104 | float：行进带符号输出、左飞轮输出、右飞轮输出、0 |
| 108/112/116/120 | float：视觉像素误差、yaw 目标换算的角速度展示值、标定后的 vx 指令、0 |
| 124/128/132 | uint32：视觉耗时（当前固定 0）、帧间隔 μs、帧数 |
| 136/140/144 | uint32：最近 SPI μs、峰值 SPI μs、视觉事件 |
| 148 | `OWHL` 魔数，4 字节 |
| 152/154 | u16：扩展版本 1、控制状态位 |
| 156/160 | float：roll/pitch raw 互补滤波结果 |
| 164/168 | float：roll/pitch 速度环输出 |
| 172/176/180 | float：roll/pitch/yaw 角速度目标；pitch/yaw 已取负 |
| 184/188/192/196 | float：roll rate、pitch rate、motor、yaw rate 输出 |
| 200/204 | float：roll/pitch 机械零点 |
| 208/212 | float：wifi_target_speed、wifi_ramp_step |
| 216/220/224 | float：车体映射后的 ROLL/PITCH/YAW_GYRO_RAW |

状态位在负载偏移 154（不是“154 字节状态包”）：bit0运行、bit1 IMU ready、bit2 tick≥2s、bit3 drive开关、bit4相机开关、bit5 roll开关、bit6 pitch开关、bit7 motor开关、bit8 yaw开关。它没有 stop_pending、倾倒故障原因、反馈过期原因等字段。

## 20. 下行调参协议、全部命令和限制

文本逐行处理，命令区分大小写：

```text
MCAR ALIVE 1
MCAR GET 2 ALL
MCAR SET 3 speed 0
MCAR SET 4 roll_rate_i 0.13
MCAR SET 5 run 0
MCAR SET 6 host 192.168.0.113
```

最后一个 IP 仅为语法示例，不表示本机当前网络地址。每条应有换行结尾。

`wl_command_bytes()` 支持串口/SPI 数据跨片拼行；忽略 CR，允许 ASCII 可打印字符和 tab，64 字节缓冲最多保留 63 字节内容；非法字符或超长后丢弃直到下一换行，防止尾部被当新命令。

`wl_split()` 按空格/tab 分成最多 6 个 token；命令要求精确参数个数。ID 用 strtoul，范围到 2147483647；`wl_number()` 用 strtod，拒绝尾随垃圾、ERANGE、NaN 和 Inf，再转 float。主机地址只收可容纳的 IPv4，首段 1..223，其余 0..255。

应答经日志通道发送：`[PARAM] id OK key value`、`ERR reason`、`DONE`。`GET ALL` 输出当前参数，再输出 run/ramp/deadband，距离标定有效时另含 cruise_cms，最后 DONE。

| 参数组 | 写入对象/作用 | 允许范围 | 运行时限制 |
| --- | --- | --- | --- |
| speed | wifi_target_speed，计数/20ms | -1000..1000 | 可运行中改，斜坡跟随 |
| roll_zero/pitch_zero | 互补滤波机械零点 | -4000..4000 | 先停 |
| k1/k2 | 视觉压弯增益；k2 当前无实际调用 | -10..10 | 先停 |
| roll_angle/speed/rate 的 p/i/d | 对应 3 环，共 9 个参数 | -10000..10000 | 先停 |
| pitch_angle/speed/rate 的 p/i/d | 对应 3 环，共 9 个参数 | -10000..10000 | 先停 |
| motor_p/i/d | 直接速度 PID | -10000..10000 | 先停 |
| yaw_p/i/d、yaw_rate_p/i/d | 两个 yaw PID；当前控制开关关闭 | -10000..10000 | 先停 |
| ramp_ms | 重新计算 ramp_step=160/value | 100..20000ms | 先停 |
| deadband_px | yaw 视觉反馈死区 | 0..10像素 | 可运行中改 |
| cruise_cms | 标定有效后反算 speed | 0..200，且反算速度≤1000 | 无标定拒绝；可运行中改 |
| run | 仅允许写 0，请求停车 | 只允许 0 | 禁止远程启动，非零返回 LOCAL_START_REQUIRED |
| host | 设置下次 UDP 目的 IP | IPv4 | 旧日志/应答清空后重连 |

静态 `wl_params` 共 32 项，加 ramp_ms/deadband_px 是常规上位机 34 个可调参数；GET ALL 另读 run 共 35 项。cruise_cms 为标定后的附加项，host 不在常规参数表。

正常参数修改需要 `cpu1_ready`，否则 NOT_READY；远程停止可以在该门槛之前处理，但仍受前面的日志空间门限影响（F3）。普通参数中只有 speed 可在 run=1 时改，其他项 STOP_FIRST；ramp 单独检查。批量参数是逐条写入，没有整体事务或掉电保存。

`wifi_vision_erro()` 仅在 `abs(erro)<deadband` 时返回 0；边界等于死区时不归零。它当前只接入 yaw 误差反馈，不会把 roll 的 `k1*erro_yawan` 同时归零。

## 21. 图像上传：虽然关闭，保留机制是什么

CPU0 在图像间隔到 66ms 后通过 `frame_want` 请求新快照。CPU1 只有拿到新处理帧且尚未占用 `frame_ready` 才复制 188×120=22560 字节；CPU0 整帧发完后释放。请求超过 300ms 无响应可撤回，不会一直等同一请求。

每帧分 17 片：前 16 片各 1400 字节，最后 160 字节。每片带 24 字节 MCAM 头：魔数、版本、8位灰度类型、宽高、帧号、片号、总片数、当前长度、整幅图 CRC32；外层再包 MUDP。

CRC32 由 `wl_crc32()` 使用 16 项半字节查表计算，整帧只算一次；错误重连重新从片 0 发。发送成功后才推进片号，全部片完成才把 `frame_ready` 清零。

传的是 `image_binary` 二值图，以 8bit 灰度格式承载，不是压缩 JPEG。当前相关开关为 0，CPU1 相机调用也注释，没有这一条数据流。

## 22. 相机、信标和 IPS200 的完整预留路径

恢复视觉需要既设置相应开关，又取消 `app_poll()` 中的两行调用注释；仅改宏不会开始处理/显示图像。

### 22.1 新帧门槛和处理次序

`app_camera_poll()` 先清 `app_new_frame`，只有相机开关开、初始化成功、`mt9v03x_finish_flag` 为真才继续。消费标志后复制原图到 `base_image`，Otsu 求阈值，小于 180 则抬到 180，再二值化、检测信标、记录误差、置新帧标志。

相机初始化默认自动曝光关闭、曝光512、帧率50、增益32，参数通过库配置送给模块；UART1 接收参数通信，P02.0 场同步与 DMA 完成回调更新采集过程/完成标志。它们只有相机初始化成功后才构成实际采集链。

`Image_Binarization()` 实际读的是 `mt9v03x_image`，不是阈值所用的快照；≥阈值写 255，否则 0。最后把顶部 30 行的二值图与 base_image 清零。相机 DMA 可能同时修改输入，风险见 V3。

### 22.2 Otsu

`Ostu()` 清 256 桶直方图及 P/PK/MK 数组；统计 188×120 像素，计算灰度概率、累计概率和累计灰度均值，再在 5..244 搜索类间方差最大阈值。当前没有处理类概率为 0/1、最大方差始终未变等退化情况（V1）。`hist` 是 uint16，默认 22560 像素不超过其范围。

### 22.3 信标检测

`detect_beacon()` 清主坐标、状态和 area；按 y=39..109、x=10..187 扫描白点，对每点向左上统计 10×10 窗口像素和值；保存最大候选，只有 y 距离超过 20 才认为可作为另一候选。

然后计算主/次候选到画面中部的曼哈顿距离，以及横向距离、到底部距离和窗口亮度组合权重。主候选亮度项为减，次候选亮度项为加，代码并不使用同一对称评分公式；主权重大于次权重时交换坐标和亮度和。

`erro = 94-centerX`，正值表示信标在中心左侧。`erro_yawan=erro` 供旧压弯/视觉控制使用。没有主坐标时按上次误差决定保持或给 ±65 的丢标误差；`last_erro=-1` 存在 0/0（V2）。`status` 始终没有改成 FOUND/CENTERED，`area` 不统计，sum/次候选也没有完整逐帧重置（V4）。

`MIN_BEACON_AREA`、CENTER_THRESHOLD_X/Y、grayscale、guding_erro 等头文件常量是遗留/未接入项；不能据名称说已经有最小面积过滤或居中判定。

### 22.4 IPS200

`app_camera_display_poll()` 需要显示开关、IPS200 开关和新帧都成立：显示 188×120 二值图，以 2 倍缩小到 94×60；主候选画红点/红框，次候选画绿框；框边长默认 5 个显示像素；在 (0,60) 显示 `erro`，整数宽 4、小数 2。当前没有显示 CPU 状态、调参菜单或实际运行中的摄像头画面。

IPS200 SPI2=60MHz，SCLK P15.3、MOSI P15.5、MISO P15.4、RST P15.1、DC P15.0、CS P15.2，BLK 宏也是 P15.4。具体驱动怎样使用 MISO/BLK 需要按屏幕接线与驱动模式核对，当前不开启它们。

## 23. 所有未调用的旧接口、空模块与全局状态

| 名称 | 实现与当前地位 |
| --- | --- |
| `change_mechanical_zero()` | 用 k1×erro_yawan 算 new_mechanical_zero，再用 encoder×k2×erro 限幅；未调用，new_mechanical_zero 未被现行滤波读取。表达式可能形成反向上下限，恢复前需复查 |
| `change_the_speed()` | erro<60 给 expect_speed=300，否则二次式 -0.05e²+6e+53；当前未调用，现行速度由 retarder 跟随 WiFi 目标 |
| `en_retarder()` | 独立静态 en_output 的另一份双向斜坡；未调用，也没有 reset 接口 |
| `speed_expect()` | 独立静态 speed 的双向斜坡，返回 int 会截断；未调用 |
| `find_ma()` | 旧包装，直接调用 imu_data_get；当前未调用 |
| `imu_filiter()` | 仅把 icm_data_lpf 复制给 re_icm_data；当前未调用，不是另一套运行中滤波器 |
| `flash_init()` | 检查扇区0/页8，存在数据就读入 pitch_rate.i、pitch_speed.i/d；主入口没调用，没有范围校验，也没有现行保存命令 |
| `send_int16_string()` | snprintf 十进制整数；无线 UART 开关开启才发并加 CRLF；未接入主循环 |
| `send_float_string()` | snprintf 保留6位小数；无线 UART 开才发；未接入主循环 |
| `send_two_floats()` | 实际是三个 float 的 CSV；CPU1 且 WiFi 开时进 WiFi 日志，无线UART开时也发UART。当前调用总开关关闭 |
| `four_posture.c/.h` | 全文件注释：旧 IMU 去偏/低通、加速度归一化、重力方向误差、PI校正、四元数积分/归一化、asin/atan2欧拉角与牛顿开方；所有类型与函数均未编译成现行实现 |
| `picture_process.c/.h` | 空文件/保护宏，没有处理机制 |

`acc[3]/gyro[3]` 是校正后的原始数组；`icm_data_lpf` 是六通道 int16 低通状态；`re_icm_data` 只有初值或旧复制接口会更新。`pitch_filiter` 是未接入的旧变量。

`balance_cascade` 保存当前控制状态；`balance_cascade_resave` 仅初始化备份；`motor_pid` 和全局 `new_mechanical_zero` 没有现行消费者。`bujin_car_speed`、`count_speed`、`insert_value1/2` 也不参与当前输出。

视觉全局 `erro/error_back/erro_yawan/erro_yawan_back/last_erro` 保留旧流程；当前视觉关闭时主要保持初值，不能认为后台正在识别信标。`BeaconInfo` 的坐标/候选/权重字段来自检测，但 status/area 如前所述没有完整实现。

## 24. user/isr.c 全部中断分派

所有实际 ISR 入口都开启嵌套。当前没有在主循环主动轮询替代它们的通用中断处理器。

| ISR 组 | 优先级 / 当前处理 |
| --- | --- |
| CCU60_CH0 | 30，CPU0，清标志并执行实际控制回调 |
| CCU60_CH1、CCU61_CH0/CH1 | 31/32/33，CPU0，只清标志，未配置为当前控制时钟 |
| EXTI CH0/4 | 40，检查并清 P15.4/P15.5 对应标志，无应用动作 |
| EXTI CH1/5 | 41，P14.3 调 ToF 回调，P15.8 只清标志；ToF 未在当前 app 初始化 |
| EXTI CH2/6 | 配置为 DMA、优先级值5；对应普通 ISR 实现在源码中已注释，不是另一条正在运行的处理 |
| EXTI CH3/7 | 43，P02.0 调摄像头场同步；P15.1 只清标志；当前摄像头关闭 |
| DMA CH5 | 60，调摄像头采集完成回调；当前相机未启用 |
| UART0 TX/RX/ER | 11/10/12；TX空，RX在 DEBUG_UART_USE_INTERRUPT 时进调试环形缓冲，ER交 iLLD 错误处理 |
| UART1 TX/RX/ER | 13/14/15；TX空，RX摄像头参数回调，ER交库 |
| UART2 TX/RX/ER | 16/17/18；TX空，RX无线模块统一回调，ER交库 |
| UART3 TX/RX/ER | 19/20/21；TX空，RX飞轮协议，ER交库 |
| STM0/STM1（delay驱动） | 220/221，分别服务CPU0/CPU1，清延时比较标志并释放等待 |

写了 ISR 不代表外设已打开；相机/ToF/无线 UART 等需要初始化和中断使能才会触发相应路径。UART3 当前固定 RX 回调，因此以后改 `MOMENTUM_UART` 不仅要改引脚，也要迁移中断处理，否则接收路径仍在 UART3。

`zf_assert()` 失败会调用调试断言处理：关闭 PIT、PWM、外部中断并进入诊断等待；底层 trap hook 也会打印/配置中断后停住。这里的 PWM 关闭不等于已经向串口飞轮发送零输出，不能把库断言自动当成整车完整急停。

## 25. config.h 的其他机制和编译检查

参数表见第11/20节，全部宏原值在索引。编译检查目前覆盖：

- MOTOR_DIRECTION_SIGN 必须 ±1。
- DIR 正/反电平都为 0/1 且不同。
- IMU 重试 1..255，避免 uint8 次数溢出或零次尝试。
- 按键/遥测/状态/主循环周期为正。
- 控制基本周期必须 1ms；角度/速度周期必须是基本周期倍数。
- 开相机显示要求相机与 IPS200 开；开图像上传要求相机与 WiFi 开。
- 相机宽高不得超过 255，以适应 uint8 索引循环。

这些检查没有覆盖所有组合：例如角度/速度周期不能为0、所有执行器符号是否为±1、死区补偿与最终占空比上限、扫描窗口边界等仍需人工维护。部分 APP 开关用 `#ifndef` 允许构建覆盖，其他宏是直接定义，不能假定全部支持命令行覆盖。

CPU 主频仍在逐飞 `zf_common_clock.h` 中设置为 200MHz；UART0 调试参数、库按键阈值、协议魔数、Flash扇区等也不全在 config.h。config.h 集中了应用调参，不等于替换了所有库常量。

当前地图文件还保留 `image_binary`、`wl_frame`、`mt9v03x_image` 三个各22560B数组，而未引用的 base_image 被移除。关闭视觉代表不执行，不保证所有视觉相关 RAM 都已经释放。

## 26. 上位机入口与验证机制

`tools/monitor_onewheel.py` 支持 `--upper-computer`、`--data-dir`，检查指定目录有 onewheel_profile.py，再把 `--profile onewheel --firmware-config <本项目config.h>` 交给外部 app.py。`apply_profile()` 用同一配置文件构建调参默认值/范围。

外部上位机独轮解码要求负载 228B、148处 OWHL，解析各字段和状态位；网络 MUDP 分流、文本参数 ACK/ERR/DONE、MCAM 图像重组仍由上位机协议层完成。普通运行 SET 是 RAM 修改，没有固件写 Flash或自动保存进config.h；电脑保存调参方案也不等于固件上电自恢复该方案。

`check_wifi_native.py` 临时复制真实应用源码和公开 WiFi 示例配置；模拟 SPI/串口/编码器/按键/时间，执行真实 C，再让真实上位机解码生成的字节。它检查参数回读、命令合法性、日志失败保留、SPI握手、CRC与17片图像，以及6组控制配置。

主机测试无法证明硬件引脚、接线、IMU方向、真实 PWM 换向窗口、中断最坏耗时或实车直立。新增审查复现证明旧测试没覆盖的边界仍有缺陷；编译0错误/0警告也不代表逻辑无问题。

## 27. 按什么顺序阅读与标定

先读 `config.h` 判断开启的模块，再看两个 main 和 app 的上电/循环；接着沿 `pit_isr_callback()` 看每个分支，再对照 PID 与执行器，最后读通信和关闭的视觉部分。

标定对象必须明确单位：IMU 静态零偏、车体轴与正负方向、roll/pitch raw 机械零点、行进/飞轮/编码器方向、死区补偿、计数到距离、各环增益与输出阈值。先确认保护和信号方向，再讨论增益。当前代码默认值不能证明电机响应方向已经闭环正确，更不能证明你的车已具备稳定直立条件。
