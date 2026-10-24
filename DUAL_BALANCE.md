# Pitch + Roll 双轴串级测试

新增 Mode=5 并设为菜单默认值，上电不自动驱动电机。Mode=3 保留飞轮单轴，Mode=4 保留地轮单轴。先 Stop 再切换模式。

Pitch 指当前屏幕前后俯仰，由地轮控制，底层为 imu_roll_deg/gyro.x；Roll 指左右侧倾，由飞轮控制，底层为 imu_pitch_deg/gyro.y。显示名称保持现状，未更改 IMU 轴映射。

地轮默认 Angle KP=10、Rate limit=30°/s、角速度 KP=100、PWM limit=4000、Sign=+1；菜单 PID / Pitch balance，零点为 Balance X。飞轮沿用独立参数（代码默认 Angle KP=50、Rate limit=80、角速度 KP=35、PWM limit=4000、Sign=+1），菜单 PID / Gyro test，零点为 Balance Y。已有 Flash 存档优先，烧录后核对实际菜单值，修改后 Save to Flash。

两轴都采用目标角度减实际角度、角度 P 输出限幅后的目标角速度、角速度 P 输出 PWM；I/D、速度外环和偏航混入不参与。每 5 ms 同时更新。飞轮左右指令为 U_roll/-U_roll，地轮指令为 U_pitch。

标定期间输出为零。双轴模式任一轴绝对倾角超过30°、IMU故障/25ms样本陈旧、参数无效，都同时清零所有输出并锁存到Stop/Start。单轴模式仍只检查所测轴倾角。

## 操作

1. 分别确认两个单轴环的方向正确，再在可靠支撑或限位下测试。需要地轮有合适接地条件，防止车模窜动；不要直接无支撑松手。
2. 如需记录，停车状态下先 Connect。选择 Mode=5，核对两组参数及各自零点，再 Start。
3. 小幅、分轴施加约2～3°扰动，观察两个方向是否都朝各自零点恢复。若任一方向加剧倾倒或明显振荡，Stop，回单轴模式定位。程序测试通过不能代替实车稳定性验证。

## WiFi

Mode=5 Start 或在此模式 Connect 后自动选择40通道、请求20ms周期，每帧164字节，发送MCAR SUB通知以更新Route_Run字段。切回Mode=3/4时恢复原28通道。

字段分为dual_roll_*和dual_pitch_*，包含目标/实际角度、误差、目标/实际角速度、角速度误差、PWM。还记录两轴KP/Sign/限幅、三个电机指令、飞轮原始转速、时序/状态和四个饱和标志。完整顺序见tools/dual_telemetry.json。

GUI可以导出CSV；若用脚本接收，不要同时占用8081：

```powershell
python tools/wifi_rate_capture.py --profile tools/dual_telemetry.json --seconds 30
```

控制隔离、联动停机和Flash边界由主机测试覆盖；40通道已通过固件实际打包→本机UDP→Route_Run接收解析测试。实车无线收包及双轴自主稳定性仍需烧录后确认。
