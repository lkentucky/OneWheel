# 横滚角速度闭环测试

CPU0 已恢复菜单入口，上电不驱动电机。`Startup / Mode 3=gyro` 默认值为 3，模式 0、1、2 仍为原来的完整控制模式；当前测试请保持 3。

1. 编译并烧录。在 `PID / Gyro test` 确认 `KP=5`、`PWM limit=1000`、`Sign +/-1=1`。
2. 固定或扶稳车架，使飞轮能自由旋转，进入 `Startup / Start`，设为 On。先保持静止，等待 IMU 标定完成。
3. 状态变为 2 后，轻轻向两侧转动车身，观察是否产生阻止转动的反作用力矩。若加剧转动，立即 Stop；停稳后把 Sign 改为 -1，再重新 Start。
4. 测试不设时间限制，使用 `Startup / Stop` 或将 `Start` 设为 Off 手动停止。测试只抑制角速度，不会主动回到直立。
5. 方向确认正确后，只逐步增加本页 KP。PWM limit 有效范围为 0～2000，KP 为 0～100；超出范围会停机。测试参数只在 RAM 中生效，不纳入 Flash 保存。

控制为 `error = 0 - gyro.x`（度/秒），`PWM = Sign × KP × error`，限幅后左右驱动指令分别为 `PWM`、`-PWM`，沿用原横滚混合器的安装方向约定。每 5 ms 执行一次；I、D、速度外环、角度外环、偏航和地轮驱动均不参与。

屏幕底部显示 Rate、PWM、State、Roll、Pitch。ADS 可查看 `roll_rate_test_state`、`roll_rate_test_actual`、`roll_rate_test_error`、`roll_rate_test_pwm`。不要在电机运行期间暂停 CPU，主控暂停不保证驱动板停止。

| State | 含义 |
| --- | --- |
| 0 | 关闭 |
| 1 | 等待静止标定 |
| 2 | 正在闭环 |
| -1 | IMU 异常、无有效样本或样本连续 25 ms 未更新 |
| -2 | 横滚或俯仰绝对值超过 30° |
| -3 | KP、PWM limit 或 Sign 无效 |

手动停止或故障后输出保持零，必须 Stop / Start 才重新测试。

之前的飞轮脉冲测试入口保存在 `tests/motor_uart_pulse.c.in`，不会编入固件。
