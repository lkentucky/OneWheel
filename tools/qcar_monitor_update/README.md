# 上位机兼容更新

与主控本次驱动通信修复配套：最大订阅数从40改为48，完整字段表含反馈年龄/新鲜度，startup为39通道、dual为45通道，旧字段在帧中的位置保留。Mode8独立15通道调试配置保留。

这些Python文件来自本机 `McarPVC/upper_computer`，覆盖到该上位机源码目录的同名文件即可；它们依赖原上位机的app、transport、qcar_parameters等模块，不是独立上位机发行包。本机源码已同步，桌面程序将使用该源码重新生成。

- `qcar_firmware.py`：与固件 `TELEMETRY_VARIABLES` 同步，包含固件当前可订阅字段；追加地轮编码器15通道profile，原profile顺序保留。
- `qcar_protocol.py`：订阅上限48，仍拒绝未知/重复字段与坏帧。
- `qcar_gui.py`：显示39/45通道配置名称和48通道上限；新增“地轮编码器 · 15通道”及停车手转订阅按钮，显示计数、计数/秒、累计位置和连续性标志。
- `tests/test_qcar_protocol.py` 和 `tests/fixtures`：覆盖原上位机同名测试/样本；样本由本次真实C遥测实现输出。
- `check_qcar_layout.py`：覆盖原上位机 `tools/check_qcar_layout.py`，验证45通道及编码器布局、等待/过期显示和曲线。
- `check_qcar_packaged.py`：覆盖原上位机 `tools/check_qcar_packaged.py`，以回环UDP验证打包EXE的双轴、Mode8和编码器profile。

在完整上位机目录执行 `python -m pytest -q tests`，再运行 `tools/build_qcar.ps1` 生成运行程序。主控WiFi配置不复制到此目录，程序端口沿用电脑8081、模块5001。

更新上位机后启动监听，等待MCAR字段公告或发送GET?，不要继续套用旧34/40字段固定帧长。`driver_feedback_fresh=0` 表示超过100ms没有有效速度帧，或尚未收到第一帧；`motor_left_pwm` / `motor_right_pwm` 仍是主控最终下发指令，不是驱动已应用指令确认。

Mode2位置保持适配：新增48通道hold profile，保留43通道speed。显示当前位置、参考位置、控制位置误差、目标/实际速度、倾角修正及速度/位置有效性。按Mode2 Start重建位置基准，IMU及编码器就绪时锁定参考；F10表示位置连续性或基准丢失，Stop→Start重新建基准。State=-4也可能是速度窗口无效或过期，需结合F码。

必须一起覆盖qcar_parameters.py；六项外环参数只存RAM/config，不保存旧板载Flash。Position KP默认0.5、目标速度限幅300计数/秒，位置更新约100ms、速度约20ms、姿态2ms；Speed KP默认0.001、KI=0，二者同时为0时不会有倾角修正。Position KP或目标速度限幅为0仅关闭位置环，仍有零速度控制。

Scheme V4共21项，兼容旧V1/V2/V3；导入只修改本地草稿，不自动发送。新增test_qcar_position.py、test_qcar_speed.py、test_qcar_pwm.py及qcar_hold.bin/qcar_speed.bin，check_qcar_packaged.py验证五种profile。网络端口8081/5001和原用户数据目录不变。

2026-10-29：IMU质量字段继续保留，用于显示拒绝时长、接受比例、加速度波动和R倍数等诊断；不再由质量拒绝/弱观测累计触发主控`imu_status=-5`停车，也不含停车静稳恢复或预览/Start互锁。既有字段顺序和“IMU可信度 · 47通道”方案保留；新增8字段`imu_pre_gx_dps/gy_dps/gz_dps`、`imu_pre_ax_g/ay_g/az_g`、`imu_pre_norm_g`和`imu_reacquiring`，配套“IMU原始/预处理 · 48通道”方案。Start后重新请求该方案，原始gyro_y与预处理gyro_y默认绘图对照。预处理陀螺仪已经扣启动标定零偏，控制再扣在线残余零偏；具体单位和质量码见IMU.md。旧恢复进度/次数字段固定0。

本轮软件验收：完整上位机223项pytest通过，7种打包程序方案回环通过；新增48通道方案读取真实C打包样本，30帧、0解析错误，并验证原始/预处理对比曲线和CSV。固件145字段与Python字段表顺序一致；所有SUB方案不超48通道且命令小于1024字节。上述结果不代表实车抗振效果。
