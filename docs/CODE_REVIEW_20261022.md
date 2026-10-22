# 当前工作树代码复审：2026-10-22

> **修复状态更新：** 下文是修复前复审记录。F1—F5 已实现并通过本机回归与 ADS 编译，修复后机制和边界见 [SAFETY_FIXES_20261022.md](SAFETY_FIXES_20261022.md)；V1—V4 视觉路径仍关闭，尚未修改。历史复现结果保留作为依据。

本轮只读审查固件，新增说明文档及工作区外的复现用例，没有修改固件源码、网络配置、HEX，也没有烧录。审查快照与哈希在 `build_validation/full_review_20261022/`。

范围：`code/` 的应用代码、`user/` 入口与中断，以及实际使用的逐飞/iLLD 驱动机制。第三方库中未使用的设备驱动不代表当前车辆功能；没有把全部第三方 SDK 逐行审计作为本轮结论。私有 WiFi 文件未复制进审查材料。

下面区分“代码与模拟复现确认”和“硬件上是否触发”。这些问题不能自动解释此前 WiFi 故障的真实原因。

## 需要优先处理的现行路径问题

### F1：互补滤波把正向越界输出伪装成机械零点（高）

位置：`code/balance_control.c:146-157`；消费者是 `code/posture_control.c:103-106、184-188` 和 `code/motor.c:64-68`。

```c
filter_value->angle_temp += ((gryo_temp + acc_temp) * filter_value->call_cycle);
filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero;
if (filter_value->filtering_angle > FILTER_STATE_LIMIT_RAW)
{
    filter_value->filtering_angle = filter_value->mechanical_zero;
}
```

默认 `FILTER_STATE_LIMIT_RAW=4000`，roll 保护阈值只有 1500。结果超过 4000 后却被写成机械零点 30，后续绝对值保护可能不停车。内部 `angle_temp` 未被清理；负方向也没有同样逻辑，因此处理不对称。

复现：向真实函数输入内部状态 6000、对应加速度 6000、角速度 0，输出变为 30；随后调用真实 `dynamic_value_control()`，`run_flag` 仍为 1。不是说正常缓慢倾倒必然绕过保护：逐渐越过 1500 时通常会先停车；问题发生于启动前已有大状态、突变或异常输入等直接跨过 4000 的情况。

建议：超限必须保留为故障依据，或显式锁存传感器/姿态故障后停车；不要替换成一个看似正常的姿态值。还需要定义非有限数值、滤波状态恢复和重新启动条件。

### F2：停车请求依赖下一次控制回调，PIT 停止时输出可保持（高）

位置：`code/posture_control.c:73-79、131-134`，实际输出清零在 `control_stop():58-70`。

```c
void control_request_stop(void)
{
    run_flag = 0;
    __dsync();
    stop_epoch++;
    __dsync();
    stop_pending = 1;
}
// pit_isr_callback():
if (stop_pending) { control_stop(); }
```

这样把电机操作集中给控制中断，正常情况下请求在下一次 1ms 回调被处理。但若回调不再运行，变量 `run_flag=0` 不会自动清硬件 PWM，也不会给无刷驱动发零指令。

复现：已输出 PWM=1400、飞轮左=-500/右=500 时调用真实 `control_request_stop()`，模拟输出保持不变；运行下一次真实 `pit_isr_callback()` 后才归零。此复现是保留硬件最后命令的模拟，不是实车电平测量。

启动代码与应用关闭看门狗，未找到独立的控制心跳故障处置。因此新增“tick 停止时遥测仍可发送”的能力，不等于“tick 停止时车辆仍能可靠停车”。

建议：设计独立于控制回调的故障禁能/看门狗处置。不要未经同步直接让两核同时写 UART 共用发送缓冲和 PWM，否则会引入新的并发问题。

### F3：日志队列仅积压 249 字节，就会静默丢弃远程 STOP（高）

位置：`code/wifi_link.c:688-689` 位于 `SET run 0` 分支（721-726）之前。

```c
if (wl_c0_len > sizeof(wl_c0_log) - 1800u) { return; }
```

`wl_c0_log` 为 2048 字节，所以上述门槛是 248 字节。它为 GET ALL 的大批应答预留空间，但也无差别拒绝 ALIVE、STOP 等短命令。GET ALL、网络诊断等使队列短暂积压就能触发，远未达到缓冲区真正写满。

复现：真实解析器收到 `MCAR SET 42 run 0` 时，设置 `wl_c0_len=249`、`run_flag=1`，命令被直接返回且 `run_flag` 仍为 1，未生成错误应答。电脑可能重试，但固件不能保证这次停止请求被执行。

建议：先处理安全停止，再决定应答能否排队；GET ALL 单独预留空间，普通应答按实际大小判断。

### F4：PWM 清零调用不等于电平已经清零，换向没有保证零输出窗口（中）

位置：`code/motor.c:30-39`、`libraries/zf_driver/zf_driver_pwm.c:307-325`；iLLD `IfxGtm_Atom_Ch_setCompareOneShadow()` 仅写 `SR1`。

```c
pwm_set_duty(MOTOR_PWM_PIN, 0);
gpio_set_level(MOTOR_DIR_PIN, direction);
...
pwm_set_duty(MOTOR_PWM_PIN, magnitude);
```

逐飞 `pwm_set_duty()` 写影子比较寄存器，不等待生效。零值可能在周期更新前被新幅值覆盖，DIR 则已立即改变。因此上次模拟测试只证明“软件调用先写零再换方向”，不能证明示波器上出现过真实零 PWM 窗口。17kHz 周期约 58.8μs。

这是代码与寄存器实现确认的保证缺口；是否违反实际驱动器的 DIR 建立/保持时间，尚需其规格与波形验证。

建议：确认驱动器要求，再用分阶段换向、可靠输出禁能或等待零占空比实际生效实现时序；不能只在 1ms ISR 中随意加入长阻塞延时。

### F5：飞轮测速断线后仍永久使用上一次值（中）

位置：`code/small_driver_uart_control.c:13-67`、`code/posture_control.c:114-122`。

有效 7 字节速度帧会更新左右速度，但没有接收时间戳、有效标志或过期判断。`small_driver_get_speed()` 只发请求，随后读到的是最近一次接收值；无回复时一直保留旧值。roll 速度外环会继续使用 `(left-right)/2`。

建议：增加“最近有效帧时间”和协议错误计数，规定反馈过期时的控制策略；不能把旧反馈当作持续有效测量。具体是降级还是停车需结合平衡控制安全策略决定。

## 当前关闭路径的确定问题

### V1：均匀图像的 Otsu 阈值可能未初始化

`code/camera.c:122-163` 中 `threshold` 没有初值，只有 `vartmp>var` 才赋值；均匀图像可能令分母 `PK*(1-PK)` 为零，比较不成立，最终返回未确定值。当前摄像头关闭，不影响现行直立链路。

### V2：丢标分支存在确定的 0/0

`code/camera.c:99-109` 的 `abs(last_erro+1)/(last_erro+1)` 在上一误差为 -1 时产生 NaN。真实 `detect_beacon()` 对全黑图、上一误差 -1 的复现已确认。恢复视觉控制前必须处理。

### V3：阈值计算与二值化可能使用不同帧

`app_camera_poll()` 把摄像头数组复制到 `base_image` 后计算阈值；`Image_Binarization()` 却仍读会被 DMA 更新的 `mt9v03x_image`。因此复制快照没有贯穿整条处理链，存在跨帧读取风险。是否实际重叠取决于 DMA 时序。

### V4：信标候选状态未完整重置、枚举没有更新

`detect_beacon()` 每帧只清 `status/centerX/centerY/area`；`sum/sum_last/last_center*` 等可能保留上帧状态。它也始终未把 `status` 设为 FOUND/CENTERED，`area` 没有统计。WiFi 目前用 `sum+坐标范围` 判断事件，未用这个枚举，不能把未实现的状态机制当作已有功能。

此外，改变扫描最小坐标/窗口尺寸需要保证 `image[y-i][x-j]` 不越界，现有默认值满足，但配置编译检查未覆盖这一关系。

## 需要理解但不能直接判成 bug 的机制与边界

- 中间电机输出的保护判断发生在死区补偿前。输入 8000 会输出 8400，而硬件上限是 10000。这是“原指令阈值”和“最终占空比”的区别，不能仅据数值不相同认定当前输出超出硬件范围。
- `CONTROL_PITCH_ENABLE || CONTROL_MOTOR_ENABLE` 是因为 pitch 平衡和独立速度 PID 都驱动同一个行进电机，关闭其中一个不应顺带关闭另一个；两者都关闭时才关行进输出。
- 编码器并非一直为零，真实读取在 `control_feedback_poll()`，每 20ms 读计数、乘方向符号、累加后清零。
- `small_driver_set_duty()` 自身不限幅，但现行 `dynamic_value_control()` 上游会限幅，不能说现行链路任意绕过 10000 上限。它仍是需要调用者遵守范围的底层接口。
- 两核共享中断表：LSL 的 `__INTTAB_CPU0` 与 `__INTTAB_CPU1` 同指 INTVEC0；STM1 路由 CPU1、向量写 0 在本工程并非自动构成错配。
- 上位机 SET 的停车检查只读 `run_flag`；CPU1 可能同时发布启动请求。当前不是完整的跨核“停车后事务调参”协议。批量修改多个参数也没有整体原子性，需要明确调参/启动互斥策略，不能仅靠逐个 float 的单次写入证明事务安全。
- 遥测序列号保护 CPU1 发布的 `wl_share`，不把全部控制器字段和 IMU 采样变成同一瞬时快照。诊断可见状态可能跨控制周期，不能以每个字段时间完全一致作为前提。
- K2/K3/K4 处理的是短按事件，通常在释放按键后产生，不是按下即硬件急停。应用每轮清状态，长按状态也被清掉，长按释放仍可能转成短按。现有主机测试模拟的是按键事件，未模拟完整 GPIO 消抖过程。
- roll 输出上限与 `PROTECT_ROLL_DUTY` 同为 10000，且保护比较为 `>`，因此“达到饱和上限”本身不会触发这条保护；倾倒保护仍看姿态值。
- 原始 IMU 数据低通存回 int16，存在逐步截断；读取频率 1kHz 不代表传感器每毫秒产生新样本，当前配置加速度约 52Hz、陀螺仪约 208Hz。
- WiFi 断连没有自动停车策略；`ALIVE` 只记录一个标志，没有超时失联保护。传感器运行时故障也没有独立健康度判定，`app_imu_ready` 仅代表上电初始化结果。
- STM 低 32 位差值支持正常轮询期间的回绕，不代表暂停任意长时间都能恢复完整时间。控制计数为 uint32，长期运行回绕后还会重新经过 2s 预热条件。

## 验收与审查边界

1. 现有 6 组本机回归全部通过：默认、观察模式、仅飞轮、仅行进、行进反向、后台周期覆盖。
2. 新增独立复现用例确认 F1/F2/F3/V2，输出在 `build_validation/full_review_20261022/cases-results.json`。这些用例证明缺陷存在，不能表述为测试证明代码无缺陷。
3. F4 通过逐飞驱动和 iLLD 实际寄存器写入路径确认；未用示波器验证。
4. 只把外设协议的小范围检查委派给 DeepSeek Flash（9 次工具调用），合同验收只新增报告。已独立核实其建议：采纳反馈无过期检测，拒绝“OR 条件错误”“编码器无读取”等脱离调用方上下文的判断。
5. 已核对应用源码快照保持不变。上次 ADS 编译结果不能替代本轮边界审查，更不能替代实车验证。

完整运行机制、代码片段和变量单位见 [CODE_WALKTHROUGH.md](CODE_WALKTHROUGH.md)，每个实际函数与配置宏的索引见 [CODE_MECHANISM_INDEX.md](CODE_MECHANISM_INDEX.md)。
