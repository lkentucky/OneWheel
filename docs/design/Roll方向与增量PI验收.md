# Roll方向与增量PI验收（2026-11-09）

基线：master，462b924。本轮工作仅保存在本地，未提交、未推送、未烧录。原有`.cproject`本地修改保持原样。

## 最终修改与复审

- Roll Rate调用独立的增量式PI函数，D固定停用；P项符号、Angle位置式PID及现有调度保持不变。
- 新增左右独立的原始编码器极性配置；文本和二进制RPM回传统一归一化，边界负数取反不会溢出。
- 在“正本地PWM对应正本地RPM、正Roll控制量产生负Roll角加速度”的约定下，Speed回收改为零点加偏移。
- Balance与单轴Speed测试共用目标角方向、±10°限幅和实时目标遥测。
- Roll菜单按参数名称判断环归属；移除Rate KD编辑行，保留旧协议键但拒绝非零KD。
- 旧Flash按名称加载，原KD归零，其他参数及机械零点不清空；没有自动写入或擦除Flash。
- 保持原Stop、故障退出、测试启动/退出、Balance重启的清历史路径。
- 二次复审修正了简单增量限幅可能制造P饱和偏置的问题：独立保留P历史与有界I输出，限制新增积分；KI=0时P饱和后误差归零，输出归零。

## 已通过

20项相关回归，执行实际C函数；使用本机TCC内存运行模式，不访问车辆。

```powershell
$env:ONEWHEEL_HOST_TCC='C:\Users\Lenovo\Desktop\独轮\build_validation\host_tools\tcc\tcc.exe'
python -m pytest -c tools/unicycle_station/pyproject.toml tools/unicycle_station/tests/test_roll_pi_direction.py tools/unicycle_station/tests/test_direction_baseline.py tools/unicycle_station/tests/test_angle_timing.py tools/unicycle_station/tests/test_pitch_coordinates.py tools/unicycle_station/tests/test_imu_forward.py tests/test_qcar_driver.py -q -k 'not real_imu_mapping_bias_acceleration_and_restart'
```

结果：20 passed、1 deselected；另有一条已安装pytest插件的弃用提示。覆盖PI累加、P饱和回零、积分饱和退出、禁用KI、实际reset、三环正反向输入、20ms回收保持、单轴与Balance目标一致性、Flash加载/保存兼容、菜单权限、四组编码器极性×四组输出方向及UART双格式。被排除的一项是未修改的全套IMU原生GCC测试，其脚本仍要求本机可写输出文件的GCC。

ADS最终全量编译：`python tools/ads/build.py --clean --timeout 240`。

- STATUS=OK，ERRORS=0，完整构建0 errors / 0 warnings。
- 最终日志完成时间：14:34:57；流程用时40.2s。
- 当前构建输入SHA256：`44ae824bd4a28b2e0ab6b35da7fcc4adb2a64442c7fc24ea01f7a3aac2cbf3ce`，已与当前输入重新比对。
- 已确认链接MAP包含`pid_inc_pi_calc_limited`。
- `git diff --check`通过。

已将产物复制到工程`Debug`，复制前后SHA256一致：

| 文件 | 字节数 | SHA256 |
|---|---:|---|
| OneWheelUpright.hex | 492786 | 2e0517b85de186264e06a3fce2e183468be1f8e9ec3a3451a6376d8072c4d28b |
| OneWheelUpright.elf | 12223528 | 040935594ba72e9baeeb1bb856eb006b7f8282d6697182d2983e108ba667473c |
| OneWheelUpright.map | 3172137 | 1fa296a92028e8ffcb2d1a2c9b304eb7e7e36846ea563f083b03607ea0b5bef6 |

完整机器构建记录：`.ads/build/latest.json`。

## 扩大检查发现的限制

没有宣称整个仓库的历史测试套件全部通过：

- HEAD本来已有78项固件参数，旧profile/mock及部分测试仍按75项检查；已用`git show HEAD`确认本轮未增加参数数量。这类全表一致性断言仍失败，没有为使测试变绿而修改历史实车快照。
- 本机GCC可运行但创建临时文件/输出文件报Permission denied；部分独立测试硬编码GCC，不能通过现有TCC入口运行。
- 扩大检查中的旧压弯函数测试未通过；本轮没有更改压弯函数，未将其列为已通过项目。
- 当前Python未安装PySide6，完整GUI测试无法收集；本轮仅同步仓库内PI参数说明，没有替换桌面上位机程序。
- 先前TCC数学头与Pitch测试自定义函数的兼容问题已修正，最终Pitch相关测试通过。

## 仍需实车确认

两路`FLY_ENCODER_POLARITY`默认+1只保留旧回读极性，不等于已读取并确认驱动板内的标定符号。先核对两轮各自正负PWM与RPM同号，再依次验证Rate阻尼、Angle回正、Speed回收。不能把编译与符号模型的通过，当作当前实车一定能直立的证据。

默认Rate KP=20、KI=0；已有Flash中的KP/KI仍优先加载。KI每1ms累加一次，需按新PI重新试调。IMU融合和滤波没有在本轮更改。
