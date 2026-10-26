# 上位机兼容更新

与主控本次驱动通信修复配套：最大订阅数从40改为48，完整字段表含反馈年龄/新鲜度，startup为39通道、dual为45通道，旧字段在帧中的位置保留。Mode8独立15通道调试配置保留。

这三个Python文件来自本机 `McarPVC/upper_computer`，覆盖到该上位机源码目录的同名文件即可；它们依赖原上位机的app、transport、qcar_parameters等模块，不是独立上位机发行包。本机源码已同步，桌面程序将使用该源码重新生成。

- `qcar_firmware.py`：与固件 `TELEMETRY_VARIABLES` 及三个遥测profile同步，含94个可订阅字段。
- `qcar_protocol.py`：订阅上限48，仍拒绝未知/重复字段与坏帧。
- `qcar_gui.py`：显示39/45通道配置名称和48通道上限。
- `tests/test_qcar_protocol.py` 和 `tests/fixtures`：覆盖原上位机同名测试/样本；样本由本次真实C遥测实现输出。
- `check_qcar_layout.py`：覆盖原上位机 `tools/check_qcar_layout.py`，验证45通道布局。

在完整上位机目录执行 `python -m pytest -q tests`，再运行 `tools/build_qcar.ps1` 生成运行程序。主控WiFi配置不复制到此目录，程序端口沿用电脑8081、模块5001。

更新上位机后启动监听，等待MCAR字段公告或发送GET?，不要继续套用旧34/40字段固定帧长。`driver_feedback_fresh=0` 表示超过100ms没有有效速度帧，或尚未收到第一帧；`motor_left_pwm` / `motor_right_pwm` 仍是主控最终下发指令，不是驱动已应用指令确认。
