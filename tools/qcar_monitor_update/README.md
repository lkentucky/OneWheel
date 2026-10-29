# 上位机兼容更新

与主控本次驱动通信修复配套：最大订阅数从40改为48，完整字段表含反馈年龄/新鲜度，startup为39通道、dual为45通道，旧字段在帧中的位置保留。Mode8独立15通道调试配置保留。

这些Python文件来自本机 `McarPVC/upper_computer`，覆盖到该上位机源码目录的同名文件即可；它们依赖原上位机的app、transport、qcar_parameters等模块，不是独立上位机发行包。本机源码已同步，桌面程序将使用该源码重新生成。

- `qcar_firmware.py`：与固件 `TELEMETRY_VARIABLES` 同步，含116个可订阅字段；追加地轮编码器15通道profile，原profile顺序保留。
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

2026-10-29：追加14个IMU质量字段，保留既有字段顺序与profile。新增“IMU可信度 · 47通道”，含实际C生成的qcar_quality.bin及解析回归；完整上位机221项测试通过。LIST元数据缓冲扩至4096字节，仅主循环使用静态存储；订阅最大48通道和SUB命令缓冲保持兼容。Start会切换固件默认方案，需要在Start后重新请求IMU方案。
