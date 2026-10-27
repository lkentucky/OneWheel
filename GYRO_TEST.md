# 飞轮角速度测试

闭环参数允许设置的最大范围为 Rate 200°/s、PWM 8000，统一由
`BALANCE_RATE_LIMIT_MAX_DPS` 和 `BALANCE_PWM_LIMIT_MAX` 定义。
飞轮默认值：Rate 100°/s、PWM 8000、机械零点 -0.5°、角速度 Kp 36。
地轮仍为 Rate 100°/s、PWM 4000；修改编译默认值不会自动改动板上 RAM。
控制器校验、WiFi滑杆和Flash记录校验采用同一范围；仅在`QCAR_BOOT_LOAD_FLASH=1`时读取有效Flash参数，当前默认0使用编译默认值。

控制环额外低通已删除：飞轮、地轮角度直接使用EKF解算值，角速度直接使用校正后的机体陀螺仪反馈。EKF及其内部加速度低通保留。

单轴 `rate_actual_dps`、Mode5的 `dual_roll_rate_dps` 为PID实际使用的角速度，误差和PWM使用同一值；不要与未扣除全部估计零偏的原始诊断量混淆。NaN/Inf、倾角、样本过期、故障锁存和启动渐增保护保持有效。地轮角速度KP编译默认值20，飞轮36；主机测试通过不代表实车稳定性已验证。

当前版本说明已集中至 [README](README.md)、[模式说明](MODES_STARTUP.md)、[菜单](MENU.md) 和 [遥测](WIFI.md)。

仅保留Mode3～8（8为PWM开环调试）；参数以code/config.h及运行时回执为准，旧版速度环与信标说明不再适用。
