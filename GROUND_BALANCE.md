# 地轮 Pitch 角度与角速度串级测试

Startup / Mode=4，菜单 PID / Pitch balance。这里的 Pitch 是已交换显示名称后的前后俯仰；底层使用 imu_roll_deg 和 attitude_state.gyro.x。飞轮输出始终为零，不接速度外环。

默认内环 KP=100、PWM limit=4000、Sign=+1；新增外环 Angle KP=1、Rate limit=10°/s，目标角度为地轮自身的 Balance X（mono_params.balance_angle_x），不使用飞轮的 -0.3°。

控制关系：

```text
角度误差 = Balance X - 当前显示的 Pitch
目标角速度 = clamp(Angle KP × 角度误差, ±Rate limit)
角速度误差 = 目标角速度 - gyro.x
地轮 PWM = clamp(Sign × KP × 角速度误差, ±PWM limit)
```

先 Stop，核对 Balance X 对应真实平衡姿态，再 Start，等待 IMU 标定/新样本就绪。扶稳车架，用约 2～3° 的小偏角检查是否朝目标回正。确认方向正确但回正太慢时，再逐步提高地轮 Angle KP；不将飞轮 Angle KP=50 直接用于地轮。Angle KP=0 则只抑制角速度，不主动回正。

Angle KP 有效范围0～100，Rate limit大于0且不超过100°/s，Balance X绝对值不超过20°；参数越界为State=-3。被测Pitch超过±30°为State=-2；IMU异常/样本陈旧为State=-1。故障锁存到Stop/Start。

地轮菜单 Save to Flash 同时保存地轮内环与外环参数，旧存档缺少外环字段时采用默认1/10；Balance X沿用原有车辆参数存档。已有有效Flash会覆盖默认值，烧录后核对菜单。

WiFi的balance_target_deg、balance_actual_deg、balance_error_deg、balance_kp、balance_rate_limit_dps，以及rate_target_dps、rate_actual_dps、rate_error_dps、rate_pwm，均对应当前被测地轮串级环。rate_axis=1、motor_ground_pwm为实际地轮指令，左右飞轮指令为零。默认角度诊断字段仍可使用；单独地轮采集配置在tools/ground_telemetry.json。

Mode=5???????????????Pitch???Roll???DUAL_BALANCE.md?
