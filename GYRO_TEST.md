# 飞轮角速度测试

闭环参数允许设置的最大范围为 Rate 200°/s、PWM 8000，统一由
`BALANCE_RATE_LIMIT_MAX_DPS` 和 `BALANCE_PWM_LIMIT_MAX` 定义。
飞轮默认值：Rate 100°/s、PWM 8000、机械零点 -0.5°、角速度 Kp 43。
地轮仍为 Rate 100°/s、PWM 4000；修改编译默认值不会自动改动板上 RAM。
控制器校验、WiFi 滑杆和 Flash 保存/加载采用同一范围；旧 Flash 中的有效值仍会加载。

飞轮角速度反馈在进入 PID 前增加一阶低通，适用于 Mode3/5/6。
`code/config.h` 中 `FLY_RATE_LPF_TAU_S=0.005f` 表示 5ms 时间常数；
当前 5ms 控制周期下系数为 0.5，改为 `0.0f` 可旁路，需重新编译。
地轮反馈和 Mode8 原始 PWM 不经过该滤波。

滤波首帧以有效实测值初始化，只在 IMU 样本序号更新时推进；启停、标定和
故障时清除状态。NaN/Inf、倾角和样本过期保护仍检查未滤波数据。
正常运行时，单轴 `rate_actual_dps`、Mode5 的 `dual_roll_rate_dps` 为 PID
实际使用的滤后反馈，误差和 PWM 使用同一值。`gyro_y_dps` 可作未经本低通的
传感器参考，但其零偏处理与控制反馈不同，不应直接当成完全相同的输入。
滤波引入额外相位延迟，主机测试通过不代表实车已消振；本次不调整 PID 增益。

当前版本说明已集中至 [README](README.md)、[模式说明](MODES_STARTUP.md)、[菜单](MENU.md) 和 [遥测](WIFI.md)。

仅保留Mode3～8（8为PWM开环调试）；参数以code/config.h及运行时回执为准，旧版速度环与信标说明不再适用。
