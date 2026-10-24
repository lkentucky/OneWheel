?????????Mode3 Roll???4 Pitch???5 ???6 Roll?????7 Pitch?????????Pitch????Balance X=3.2????PWM?1??????????????MODES_STARTUP.md???Flash?????????

???????Mode=5?????Pitch??Angle KP=10?Rate limit=30?PWM limit=4000?Roll????????Mode=3/4??????????Stop??????DUAL_BALANCE.md?Flash??????????????

# ??????

- ?? Startup / Mode=4??? Pitch ????????????????????P???Angle KP??1?Rate limit??10?/s????Balance X??Angle KP=0?????????????????KP=100?PWM limit=2000?Sign=+1?
- ????? PID / Pitch balance ????Stop ?? Save to Flash?????????????????
- Mode=3 ????????Angle KP ??50?????? PID / Gyro test???????Stop?
- ??????IMU????????State=-2????????????30???????Stop/Start?
- ??????????????????????????????????Stop????Sign??-1??????????4000???????
- Mode4???WiFi??????????PWM?????????????????0??????GROUND_BALANCE.md?
- ????rate_axis?1???0????motor_ground_pwm????????tools/ground_telemetry.json??????python tools/wifi_rate_capture.py --profile tools/ground_telemetry.json --seconds 30?

# Mode 3: Pitch balance tuning (current behavior)

Mode 3 now runs Pitch angle P -> Y rate P every 5 ms. The old gyro-only notes below describe the Angle KP=0 setting.

1. Stop. Select Startup / Mode 3=gyro.
2. Under PID / Gyro test, set KP=50 and Sign +/-1=-1 (existing Flash may override defaults). Keep the previously validated PWM limit; start conservatively.
3. Set Balance Y to measured Pitch at the mechanical balance point. Valid offset range: -20..20 degrees.
4. Set Angle KP=0.5 and Rate limit=20 deg/s. Start with support and small 2..3 degree disturbances; increase Angle KP gradually only if return is slow. Stop if motion grows.
5. Angle KP=0 disables the outer loop for rate-only checks. The ground wheel, speed outer loop and yaw stay off in Mode 3.
6. Stop before Save to Flash. New angle KP/rate limit records use a separate checked Flash extension; old saved rate KP/limit/sign remain readable. Balance Y uses the existing vehicle parameter storage.

Formula: target_rate = clamp(Angle KP * (Balance Y - Pitch), +/-Rate limit); PWM = clamp(Sign * KP * (target_rate - gyro.y), +/-PWM limit).

Defaults: rate KP=50, Sign=-1, Angle KP=0.5, Rate limit=20, PWM limit=3000. Flash values override defaults. These changes apply to Mode 3; Mode 0 still uses its original incremental PID.

Telemetry: rate_target_dps now reports the computed target, rate_error_dps is target minus actual Y rate. balance_error_deg, balance_kp, balance_target_deg and balance_rate_limit_dps are also available. IMU failure/staleness, tilt or invalid parameters latch a zero-output fault until Stop/Start.

---

# 横滚角速度闭环测试

当前为菜单入口，`code/qcar_config.h` 中 `QCAR_BOOT_IMU_WIFI=0`。上电自动初始化 IMU，WiFi 必须停车后手动将 `WiFi / Connect` 设为 On；成功后才上传数据。没有自动连接或后台重连，发送报错后停止后台收发，等待手动重连。电机控制仍需显式 Start。Stop 后 IMU 和已经连接的遥测继续运行，Flash 保存时暂停采样、保存后恢复。自动 IMU/WiFi 独立诊断入口仍保留在源码中，设为 1 可恢复。

CPU0 已恢复菜单入口，上电不驱动电机。`Startup / Mode 3=gyro` 默认值为 3，模式 0、1、2 仍为原来的完整控制模式；当前测试请保持 3。

1. 编译并烧录。在 `PID / Gyro test` 确认 `KP=5`、`PWM limit=3000`、`Sign +/-1=1`。
2. 固定或扶稳车架，使飞轮能自由旋转，进入 `Startup / Start`，设为 On。先保持静止，等待 IMU 标定完成。
3. 状态变为 2 后，轻轻向两侧转动车身，观察是否产生阻止转动的反作用力矩。若加剧转动，立即 Stop；停稳后把 Sign 改为 -1，再重新 Start。
4. 测试不设时间限制，使用 `Startup / Stop` 或将 `Start` 设为 Off 手动停止。测试只抑制角速度，不会主动回到直立。
5. 方向确认正确后，只逐步增加本页 KP。PWM limit 有效范围为 0～2000，KP 为 0～500；超出范围会停机。默认 PWM limit 仍为 1000；已经达到该限幅时，继续增加 KP 不会增大最大输出。
6. 先 Stop，再将 `PID / Gyro test / Save to Flash` 设为 On，可保存 KP、PWM limit、Sign（同时保存原菜单的其他参数）。也可使用原有 `Storage / Save`。下次上电自动读取；`Storage / Load` 可重新读取。运行或预览期间会拒绝保存，提示 `Stop before saving`。旧 Flash 没有这三个参数、校验失败或数值越界时使用默认值。

IMU 首次静止标定改为约 1 秒，允许较小的手扶抖动；明显晃动仍会重置窗口，不能边转动车身边标定。本次上电已有有效标定时，普通 Stop / Start 复用已有零偏；`Startup / IMU` 用于重新标定。也可先平放进行 IMU 预览标定，保持预览采样时再扶正，之后 Stop / Start 进入闭环，无需在架子上直立标定。

控制为 `error = 0 - gyro.y`（度/秒），`PWM = Sign × KP × error`，限幅后左右驱动指令分别为 `PWM`、`-PWM`，沿用原横滚混合器的安装方向约定。每 5 ms 执行一次；I、D、速度外环、角度外环、偏航和地轮驱动均不参与。

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


## 2026-10-23 actuator/IMU axis mapping

IMU coordinates are unchanged. Lateral balance uses Pitch/Y and gyro.y (flywheels); fore/aft balance uses Roll/X and gyro.x (ground wheel).

- Mode 3 Gyro test now reads gyro.y. Historical roll_rate_test_* symbols remain for Flash/host compatibility.
- Flywheels use Pitch speed/angle/gyro PID parameters and Balance Y fly (balance_angle_y).
- Ground wheel uses Roll speed/angle/gyro PID parameters and Balance X wheel (balance_angle_x).
- Telemetry pitch_*_out describes flywheels; roll_*_out describes the ground wheel. rate_actual_dps is Y rate. Resend subscriptions by name after updating firmware.
- Legacy internal cascade_*_out variables retain the flywheel role; cascade_*_out_p retain the ground wheel role. dynamic_zero_roll/turn_k*_l/r retain the flywheel role; dynamic_zero_pitch/turn_k*_p_l/r retain the wheel role.

Old Flash values are NOT migrated: saved PID fields still load by axis name and may be unsuitable for their new actuator. Reconfigure both PID groups and measure X/Y balance offsets. Recheck test KP and Sign with low PWM limits before saving while stopped. Defaults follow actuator roles, but existing Flash overrides defaults. Mode 3 uses cascaded angle P and rate P (Angle KP=0 selects rate-only); normal mode remains incremental PID, so gains cannot be copied directly. Verify normal-mode actuator polarity separately: Gyro test Sign affects only Mode 3.

Angle KP ?????? 0?100?PWM limit ??? 0?3000??? 3000???? Flash ?????????????? PWM limit ?? 3000 ????


??????????? Connect ??????? 28 ?????????????????? ANGLE_TUNING_DATA.md?

?? PWM limit ????????? 4000?Flash ????????? Flash ??????????????? 4000 ????
