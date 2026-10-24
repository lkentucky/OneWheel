Current Mode 3 firmware includes the Pitch angle outer loop. rate_target_dps is the limited angle-loop target, not a constant zero; rate_error_dps = target - gyro.y. New variables: balance_error_deg, balance_kp, balance_target_deg, balance_rate_limit_dps. See GYRO_TEST.md for tuning steps. Angle KP=0 selects the historical rate-only behavior.

# 角速度环 WiFi 遥测

当前为菜单启动，`QCAR_BOOT_IMU_WIFI=0`；上电只自动初始化 IMU，必须停车后手动将 WiFi / Connect 设为 On。连接成功后才收发和发布通道顺序；未连接、连接失败或发送报错时不执行后台 WiFi 轮询，也不自动重连，需手动再连。只有显式 Start 才启动电机闭环。Stop 后 IMU 与已连接的遥测继续运行。菜单底部显示 WiFi 状态和发送计数。独立自动诊断入口仍保留，说明如下。

菜单模式的初始化及运行期收发等待均会处理按键导航、数值编辑和刷新。Start/Stop/Flash 等涉及硬件的操作先请求结束等待，当前 SPI 事务退出后由外层菜单执行一次；有待执行操作时跳过新的网络轮询，避免 Stop 被下一次等待延后。运行期 K4 普通返回不会直接取消网络。IO 返回后清除取消标志，避免一次操作令后续 WiFi 一直失败。该实现仍是协作式同步 IO，不宣称硬实时或独立网络线程；按键响应及实际收包仍需实板复核。

## 可选：上电自动发送 IMU

`code/qcar_config.h` 中 `QCAR_BOOT_IMU_WIFI=1` 时，CPU0 使用独立自动诊断入口，不进入菜单，不依赖 Connect 或 Start。上电先将电机输出置零，再初始化 IMU、启动 5 ms IMU 采样，随后连接 WiFi。只启动 IMU 定时器 CH1，不启动闭环定时器 CH0。

屏幕标题为 `AUTO IMU UDP 1023-A`。Boot=3 表示正在初始化 IMU，4 等待双核同步，5 正在连接 WiFi，6 初始化连接已成功，7 等待重试。屏幕始终显示 IMU 状态、标定进度、姿态、样本序号、WiFi 状态、RC/FW、Tx、底层 Fail/Ack、电脑地址和模块地址。请保持车身尽量静止约一秒完成首次标定；网络失败时 IMU 采样仍继续。

连接成功后立即自动发送 `roll_deg,pitch_deg,yaw_deg` 三个小端 float32 加 JustFloat 帧尾，每个数据报 16 字节，默认周期请求 10 ms。设备每秒发送一次当前通道、周期、STREAM 配置通知，现有 Route_Run 上位机开启 MCAR 模式和 UDP 8081 监听后可按 SUB 通知自动更新解析字段。没有收到实车有效帧前，不能把本机协议测试当作实板通过。

初始化失败后，每轮尝试完成再等 5 秒自动重试；持续发送失败也会重连。自动诊断入口不支持用菜单 K4 取消连接。若 IMU 故障，屏幕显示负的 IMU 状态；收到全零角度也不能视为有效姿态。设置 `QCAR_BOOT_IMU_WIFI=0` 并重新构建、烧录，可恢复下面的原菜单和控制测试流程。

验证：`tests/check_imu_wifi_boot.py` 覆盖自动初始化顺序、仅 IMU 定时器、零电机输出、重试/时钟回绕及菜单入口恢复；`tests/check_auto_imu_host.py` 使用固件实际生成的三通道报文和现有 Route_Run 接收/解析代码做本机 UDP 验证。

## 原菜单模式：QCAR_BOOT_IMU_WIFI=0

当前固件目标为电脑 WLAN 地址 `192.168.0.106:8081`，模块本地端口为 5001。电脑地址改变后必须同步更新 `code/qcar_config.h` 并重新构建、烧录。WiFi / Connect 手动连接成功后才发送；Start/Stop 不会断开已经建立的 UDP 遥测。

## Connect 保持 Off 时

先执行 Startup / Stop，退出 IMU、Camera、Open loop 等预览，再进入 WiFi / Connect。运行或预览期间不会执行模块初始化，因此 Connect 不会变为 On。WiFi 页面底部现在优先显示 `WiFi / Err / Try`、模块固件版本和发送帧数，以及拒绝或失败原因，不再被角速度或 IMU 状态覆盖。

- `WiFi=0, Try=0` 且提示 `Stop before WiFi setup`：初始化未执行，先 Stop。
- `WiFi=-1`：SPI 模块初始化失败或返回版本不符合当前 V2 检查；同时观察 FW。
- `WiFi=-2`：热点连接失败。
- `WiFi=-3`：UDP socket 建立失败。
- `WiFi=1`：初始化完成，再看 Tx 和上位机接收计数。

热点或 UDP 建立失败时，底部新增 `Cmd / Fail / Ack / Len`。Cmd=10（十六进制）表示连接热点，Cmd=11 表示设置 socket。Fail=1 为发命令前 INT 未就绪；2 为 SPI 发送超时；3 为发命令后等待应答就绪超时；4 为应答传输失败或长度异常；5 为收到非成功应答。Ack=81 表示模块返回 ERROR；Ack=80 表示 OK。不要仅凭 WiFi=-2 就认定密码错误，它也可能是该步骤的传输或等待失败。

已确认本机 HDUASC 是 2.4GHz/WPA2 网络，固件密码与电脑当前连接配置一致；参考 MCARpreparation 配置的 SSID 是 ASC_Circuit，因此没有照搬其网络凭据。WiFi SPI 已从 30MHz 降至 10MHz，以增加接线余量；这不代表信号问题已被实板确认。

对照队友仓库 `OneWheel/branchwan` 的提交 `cc55824552ed202858dc63ed3412a2eba7ca70bb`，SPI 驱动和引脚定义一致，应用初始化顺序也一致。不能仅凭该分支能收包就确定本机失败原因；需要上述实际状态。对比记录见 `validation/wifi-branchwan-compare.json`。

### 按 MCARpreparation 修正 SPI 和等待

进一步对照本机 `C:\Users\csz18\Desktop\mcar-mailun\MCARpreparation`，发现其 RT1064 工程使用 Mode 3，而 TC264 OneWheel/branchwan 仍使用 Mode 0。曾尝试 Mode 3，但没有实板成功证据；重新核对同平台分支后，当前恢复为 TC264 参考的 Mode 0，保留 SPI3 引脚和新增超时保护。两种模式的差异不能当成当前实板失败原因已被确认。

WiFi 第二行新增 `RC`：记录底层 `wifi_spi_init` 原始返回值。`WiFi=-1, RC!=0` 表示模块初始化没有通过；`WiFi=-1, RC=0` 表示底层返回成功，但 FW 没有通过 `V2` 前缀检查。`RC=-1` 表示尚未调用或正在调用底层初始化。版本字段可能按固定宽度填充，现按实际缓冲区容量安全截断并补结束符，不因恰好等于字段容量而直接拒绝。

WiFi 收发改用有 STM 截止时间的 SPI API，每次传输最多等待 20 ms；主控显示屏、IMU 等其他 SPI 调用保持原 API。模块 INT 握手等待按真实 STM 时间计时，连接期间每 50 ms 有机会刷新 WiFi 页面；K4 可取消当前连接，显示 `WiFi=-6`，取消后不会继续重试。这里的连接仍是同步调用，但等待中可显示状态和取消，不宣称完全异步。

UDP socket 单次握手等待从 50 秒缩短为 5 秒，热点单次等待仍为 10 秒，最大尝试次数仍为 3。回复长度会先校验再复制；版本、MAC、IP 字符串会检查目标缓冲区大小并补字符串结束符。无响应和异常回复不会再通过原有无界 SPI FIFO 等待或未校验长度处理。

验证：`tests/check_spi_timeout.py`、`tests/check_wifi_spi_driver.py`、`tests/check_wifi.py` 覆盖 SPI 接收/结束事件超时、时钟回绕、片选释放、Mode3、异常长度/命令、握手超时和取消重试；MCAR 19 通道接收/解析回环测试保持通过。仍需实板确认模块版本、联网和实际 UDP 收包。

保留默认三通道 roll/pitch/yaw，兼容原有上位机。判断横滚反馈时使用 `tools/gyro_telemetry.json` 中的 19 通道配置；每帧 80 字节，19 个小端 float32 加 `00 00 80 7F`。上传周期请求为 20 ms，实际周期取决于主循环、屏幕刷新和网络，应看记录中的时间戳和序号。

| 通道 | 含义 |
| --- | --- |
| telemetry_ms | 本次 WiFi 初始化后的 MCU 时间，毫秒 |
| rate_seq | 5 ms 角速度控制更新累计序号 |
| rate_target_dps | 目标角速度，当前固定为 0 |
| rate_actual_dps | 控制器实际使用的扣零偏横滚角速度，°/s |
| rate_error_dps | 目标减实际角速度 |
| rate_pwm | 限幅后的横滚驱动指令 |
| motor_left_pwm / motor_right_pwm | 按安装方向混合后传给驱动板的左右指令；右侧含反号，不是实际电流反馈 |
| roll_deg / pitch_deg | 车身倾角，度 |
| rate_state / imu_status | 独立角速度测试状态 / IMU 状态 |
| rate_kp / rate_sign / rate_pwm_limit | 实际参数 |
| flywheel_left_speed_raw / flywheel_right_speed_raw | 驱动板回传的有符号原始转速值，不擅自标成 RPM |
| driver_speed_frames / imu_seq | 驱动板有效速度帧累计数 / IMU 有效样本序号 |

MCAR 上位机先监听 `0.0.0.0:8081`，选择模块实际 IP、端口 5001，在自定义上传列表粘贴下面的列表，周期填 20 ms，应用上传配置：

```text
telemetry_ms,rate_seq,rate_target_dps,rate_actual_dps,rate_error_dps,rate_pwm,motor_left_pwm,motor_right_pwm,roll_deg,pitch_deg,rate_state,imu_status,rate_kp,rate_sign,rate_pwm_limit,flywheel_left_speed_raw,flywheel_right_speed_raw,driver_speed_frames,imu_seq
```

若直接发送命令，须逐条等待对应 MCAR 确认：`STREAM 0`、`SUB 上述列表`、`RATE 20`、`STREAM 1`，均以换行结束。上位机收到 SUB 确认后按该顺序设置 float32 解析表。

## 自动收包

以下脚本直接使用现有 MCAR 上位机的 `UdpReceiver`、订阅状态机、JustFloat 校验和字段解析器；不发送电机启动或参数修改命令：

```powershell
python tools/wifi_rate_capture.py --wait 120 --seconds 20
```

脚本占用 8081 时不要让 GUI 同时监听同一端口。收到模块首包后自动订阅，确认成功后采集 20 秒，输出 `validation/wifi-rate-live-*.csv/.json`。脚本自己不启动电机，测试动作由菜单 Start 控制。可通过 `--host-dir` 指定上位机源码目录、`--module-ip` 指定模块地址。

先在停车状态验证状态/参数和计数能收到，再记录扶持车架左右转动时的数据。数据中只有 State=2 且有实际运动的片段才适合分析角速度环。手的力矩、重力和外界扰动都会影响角加速度，不能仅因 PWM 与角速度同号或异号就断言反馈方向正确。

## 验证边界

`python tests/check_wifi_host.py`：由固件实际打包/订阅代码生成测试帧，通过本机 UDP 发送给现有 MCAR 上位机接收和解析代码，验证 19 项数值及正负号。它是模拟模块条件下的本机协议测试，不代表实车无线链路已经收到数据。

实车接收证据只看 `wifi-rate-live-*.json` 的 `real_board_frames_verified` 和对应 CSV；尚未收到有效实车帧时不能报告实车验证通过。反馈方向也不由收包成功自动证明。


Axis update: rate_actual_dps is flywheel Pitch/Y (gyro.y); pitch_*_out is flywheel output and roll_*_out is ground-wheel output. Historical roll_rate_test_* symbols remain. Recheck old KP/Sign on the new axis; see GYRO_TEST.md.
