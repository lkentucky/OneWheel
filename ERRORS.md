# Qcar 报错与诊断信息速查

全部条目来自本工程代码，不是文档推测。改动代码后请同步本表。

主要来源：`code/pid.c`（故障锁存）、`code/imu.c`（IMU 状态与计数）、`code/menu.c`（屏显文本）、
`code/wifispi.c`（WiFi 状态与遥测通道）、`code/flash.c`（参数存储）。

## 0. 三个前提

1. 底部 `F / IMU / dt` 是**故障瞬间的快照**，`SPI / NR / Reg` 是**实时值**（`menu.c:845-847`）。
2. `SPI / NR` 从上电起累计，**Stop/Start 不清零**，`imu_reset_pipeline()` 也不清零（`imu.c:27,144`）。
3. 故障锁存后输出归零，必须 Stop/Start 才复位；**Stop 会清掉 `F / IMU / dt` 快照**，故障发生后先记录再动。

## 1. 控制故障码（底部 `F:` 与 `State:`）

`F` 是故障原因，`State` 是单轴测试状态机，两者一一对应（`pid.c:715-736`，文本见 `menu.c:479-492`）。

| F | State | 屏上文本 | 含义 | 触发条件 |
| --- | --- | --- | --- | --- |
| 1 | -1 | `ERR -1: IMU ...` | IMU 已成负状态，屏上文本继续显示 IMU 子码 | 检查时 `imu_attitude_status<0` |
| 2 | -1 | `ERR -1: IMU no sample` | 没有有效样本序号 | `imu_sample_sequence==0` |
| 3 | -1 | `ERR -1: IMU invalid data` | 角度/角速度出现 NaN 或 Inf | `!isfinite()` |
| 4 | -1 | `ERR -1: IMU stale >25ms` | 样本连续 25 ms 未更新，此时 IMU 状态仍为 1 | 5 次 ×5 ms 序号不变 |
| 5 | -2 | `ERR -2: Tilt >30; Stop/Start` | 被测轴倾角超 30° | 地轮看 `roll`，飞轮看 `pitch`（`pid.c:724`） |
| 6 | -3 | `ERR -3: KP/limit/sign/zero invalid` | 参数越界 | KP>500、limit>4000、Sign≠±1、\|零点\|>20° 等 |

State 的正常值：`0` 关闭、`1` 等待静止标定、`2` 闭环中。

## 2. IMU 状态码（底部 `IMU:`）

快照，不是实时值。枚举见 `imu.h:9-12`，文本见 `menu.c:586-590`。

| 值 | 枚举 | 屏上文本 | 含义 |
| --- | --- | --- | --- |
| 1 | RUNNING | `IMU:RUN` | 正常 |
| 0 | CALIBRATING | `IMU:CAL xx%` / `IMU init OK - keep still` | 静止标定中（200 帧，约 1 s） |
| -1 | INIT_FAILED | `INIT_FAIL` | 硬件初始化失败（WHO_AM_I 或寄存器回读不符） |
| -2 | TIMEOUT | `TIMEOUT` | 连续 100 ms 没有成功采样（`IMU_TIMEOUT_MS`） |
| -3 | BAD_DT | `BAD_DT` | 采样间隔 >20 ms，**锁止**，需重新标定或 `imu_recover_after_stall()` |
| -4 | FILTER_ERROR | `FILTER_ERR` | EKF 更新失败 |

`IMU:-2/-3` 时控制会先报 `F:1`；`F:4` 出现在 `IMU:1` 是正常的，因为 25 ms 的停机判断比 100 ms 的 IMU 超时更早。

## 3. 底部诊断字段

| 字段 | 刷新方式 | 含义 | 异常指向 |
| --- | --- | --- | --- |
| `F` | 故障时快照 | 故障原因码（见第 1 节） | 0 表示无故障 |
| `IMU` | 故障时快照 | 锁存瞬间的 IMU 状态（见第 2 节） | 负值说明 IMU 已失效 |
| `dt` | 故障时快照 | 最后一帧成功采样的间隔 | 只反映故障前最后一帧，不是故障时的间隔 |
| `SPI` | 上电累计 | SPI 传输超时次数（单次 1 ms 超时） | >0 = 总线卡死、CS 异常 |
| `NR` | 上电累计 | STATUS 寄存器里 XLDA/GDA 未同时就位的次数 | 增长快 = 传感器没数据或字节被干扰 |
| `Reg` | 实时 | 最后一次读到的 STATUS（0x1E） | `07` 正常；`00/01/02` 缺 ready 位；`FF` 读失败 |

STATUS（0x1E）位定义：bit0 XLDA 加速度就绪、bit1 GDA 陀螺就绪、bit2 TDA 温度就绪。

## 4. WiFi 状态（`WiFi:` 字段）

来源 `wifispi.c:385-466`。

| 值 | 含义 | 处理 |
| --- | --- | --- |
| 1 | 已连接，可发遥测 | 看 `Tx` 是否在涨 |
| 0 | 未初始化（烧录后或手动断开） | 到 WiFi / Connect 设为 On |
| 2 / 3 / 4 | 初始化过程阶段：模块复位 / 连热点 / 建 socket | 过渡值，不应停留 |
| -1 | 模块 SPI 初始化失败，或固件版本不是 V2 | 看 `FW`、`RC` |
| -2 | 连热点失败 | 查 SSID/密码/2.4G；也可能只是该步传输超时 |
| -3 | UDP socket 建立失败 | 查对端 IP/端口 |
| -4 | 发送失败或断开失败，后台收发已停止 | 手动重连 |
| -5 | 编译时 `QCAR_WIFI_ENABLED=0` | WiFi 功能被关掉 |
| -6 | 用户按 K4 取消 | — |

## 5. WiFi 底层子码（`Cmd / Fail / Ack / Len`）

只在 `WiFi=-2/-3` 时显示。

| 字段 | 值 | 含义 |
| --- | --- | --- |
| `Cmd` | `10` | 连接热点 |
| `Cmd` | `11` | 设置 socket |
| `Fail` | 1 | 发命令前 INT 未就绪 |
| `Fail` | 2 | SPI 发送超时 |
| `Fail` | 3 | 发命令后等待应答就绪超时 |
| `Fail` | 4 | 应答传输失败或长度异常 |
| `Fail` | 5 | 收到非成功应答 |
| `Ack` | `80` | 模块回 OK |
| `Ack` | `81` | 模块回 ERROR |

## 6. 菜单文字提示

| 文本 | 触发 |
| --- | --- |
| `IMU init failed` | Start 或 IMU 预览时 `ensure_imu()` 失败 |
| `Camera init failed` | 模式 1 或相机预览初始化失败 |
| `Remote UART used by motor` | 模式 2 与 `QCAR_DRIVER_UART==UART_3` 冲突 |
| `Stop before saving` / `Stop before loading` | 运行或预览中操作 Flash |
| `Stop before WiFi setup` | 运行或预览中开 WiFi |
| `Wait for WiFi then change Mode` | WiFi 事务中改 Mode |
| `WiFi connecting; K4 cancels` | 正在连接，可取消 |
| `WiFi failed; retry Connect` / `WiFi connection cancelled` | 连接失败 / 被取消 |
| `IMU error - toggle IMU Off/On` | IMU 负状态时的提示 |

## 7. 读数据时必须记住的坑

1. **屏幕角度标签是交换的**：`menu.c:574,585` 中 `"Pitch:"` 打印的是 `imu_roll_deg`，`"Roll:"` 打印的是
   `imu_pitch_deg`。模式 7（地轮）闭环和 F=5 判断用的是 `roll`，也就是屏幕上标着 `Pitch:` 的那个数。
2. **Flash 保存失败没有提示**：`write_flash()` 返回 `void`（`flash.c:682`），失败也显示 `Parameters saved`。
3. **`RB ID=0x.. expected 0x6B`** 只走 `zf_log`（`zf_device_imu660rb.c:258`），不上屏。
4. **默认值改动无效**：Flash 里已存的参数会覆盖代码默认值，改增益要在菜单里改并 Save。
5. **`F=4` 与 `F=1` 的区别**：`F=4` 是 25 ms 无新样本但 IMU 状态还是 1；`F=1` 说明 IMU 自身已经报错。
6. **`S=-1`（State=-1）不是超调标志**，是 IMU/样本故障。

## 8. Mode 7 诊断采集

分析 F=4 / F=5 需要的是"时间对齐的一整段数据"，单看锁存后的几个数不够。通道表在
`tools/diagnose_telemetry.json`（40 通道，10 ms），全部是 `wifispi.c` 已有的变量，**不需要重新编译固件**，
主机用 SUB 命令下发即可。

前置：PC 地址 `192.168.0.107`，模块本地口 `5001`，接收口 `8081`（`qcar_config.h:55-63`）。

```bash
# 终端里先运行，脚本会等板子发第一包
python tools/wifi_rate_capture.py --profile tools/diagnose_telemetry.json --seconds 30
```

在车上的顺序：

1. WiFi / Connect 设为 On，确认 `WiFi:1`、`Tx` 在涨。
2. 回到 Startup，`Start` 设为 On。
3. 保持 30 s 左右（先静止 2 s → 手动轻推 → 松手观察，或按需要提高占空比）。
4. `Stop` 记录 `F / IMU / dt` 后，再 Ctrl-C 或等脚本超时结束。

产出 `validation/wifi-rate-live-<时间>.csv` 和同名 `.json`。

### 判读要点

| 现象 | 结论 |
| --- | --- |
| `imu_seq` 在 25 ms 内不变，`imu_not_ready` 同步增加 | 传感器真的没有新数据（或 STATUS 字节被干扰）→ 电气/供电 |
| `imu_seq` 不变但 `imu_not_ready` 不增 | 采样中断没在跑，或卡在 EKF/别处 → 软件 |
| `fault_reason` 从 0 变 5 时 `roll_deg` 只到几度 | 姿态估计发散，不是真倾角 → 查 `imu_accel_used`、`accel_norm_g` |
| `rate_pwm` 在两个大值之间每 10 ms 跳变 | 环路极限环 → 增益过高 / 符号错 |
| `rate_actual_dps` 与 `rate_pwm` 同相增长 | 正反馈 → Sign 反了 |
| `accel_norm_g` 长期超出 0.85~1.15 | 剧烈振动，加速度被拒，姿态只剩陀螺积分 |

注意：`imu_not_ready` 和 `imu_spi_timeouts` 是累计值，CSV 里要**看相邻两行的差值**，不能看绝对值。

## 9. 上位机实时调参（Route_Run）

上位机：`C:\Users\csz18\Desktop\imu\WiFiSPI_UDP_MCAR_Route_Run.exe`，工作区文件
`C:\Users\csz18\Desktop\imu\WiFiSPI_UDP_MCAR.workspace.json`（退出时自动保存）。

已预置（原文件备份为 `WiFiSPI_UDP_MCAR.workspace.json.bak-before-qcar-mode7`）：

- 订阅通道：第 8 节的 40 路诊断通道，周期 10 ms；
- 滑杆：`rate_kp`（0~500）、`rate_pwm_limit`（0~4000）、`rate_sign`（-1 或 1）。

`rate_*` 三个滑杆是**新增的固件功能**（`wifispi.c` 的 `g_slider_parameters`），
**必须重新编译烧录后才会生效**；否则上位机会收到 `MCAR ERR unknown parameter`。

使用顺序：

1. 车上先 `WiFi / Connect` 设为 On。
2. 上位机「开始监听」（`bind 0.0.0.0:8081`），`remote_ip` 填模块地址（当前 `192.168.0.109`），`remote_port` 5001。
3. 点「订阅」——`mcar_names` 已填好 40 个通道，设备回 `SUB` 后解析字段自动更新，曲线开始。
4. 车上 `Start`。先静止看基线，再轻推车身。
5. 需要时拖动 `rate_kp` / `rate_pwm_limit` 滑杆（拖动即发送，立即生效），观察同一屏的曲线。

调参时的判读：`rate_pwm` 与 `rate_actual_dps` **同相**增大 = 符号反了，把 `rate_sign` 拨到另一档；
`rate_pwm` 等幅高频跳变 = 增益过大，先降 `rate_kp`；`roll_deg` 缓慢漂移而 `accel_norm_g` 一直超界 =
振动把加速度全部拒掉，姿态在靠陀螺积分漂。

注意：

- `rate_sign` 只有 -1 和 1 两档，拖到 0 会锁存 `F=6`（输出清零，Stop/Start 恢复）。
- 滑杆只改 RAM，**不写 Flash**。调好后要 Stop，在菜单里 `Save to Flash`，否则下次上电回到旧值。
- 滑杆写入的是 `ground_rate_test_kp / ground_rate_test_limit / ground_rate_test_sign`（`pid.c`），
  只影响模式 4/5/7 的地轮；模式 3/6 的飞轮用的是 `roll_rate_test_*`，没有对应滑杆。
- 工作区改动只影响上位机默认值；退出时会被上位机按当前界面重新保存。
