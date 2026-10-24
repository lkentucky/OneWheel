# 验证记录

## 联网等待期间按键响应（2026-10-23）

- 定位到普通菜单启动的自动联网及 5 秒重连路径同步调用 WiFi 初始化，旧等待回调只检查 K4，K1/K2/K3 的菜单处理要等连接返回。
- `code/menu.c` 的等待回调改为处理菜单；硬件动作先请求取消网络，再由外层主循环执行。发起动作前消费按键事件，避免等待回调再次读取同一事件；有待执行动作时不抢先重连。
- 新增 `python tests/check_menu_wifi_responsiveness.py`，在真实菜单函数内注入延迟网络回调，验证等待期间导航、数值编辑、K4 取消、Start/Stop/Flash 延后执行、动作前不重连，以及发起联网的按键不会重复执行。
- `check_key_debounce.py`、`check_menu_keys.py`、`check_menu_tree.py`、`check_wifi_menu.py`、`check_rate_menu.py`、`check_imu_wifi_boot.py` 通过。`check_wifi_menu.py` 清理了当前界面实现已不使用的两个测试桩变量，保留严格警告检查。输出记录见 `validation/key-responsiveness-host-tests.log`。
- ADS Debug 增量构建通过：`22:16:47 Build Finished. 0 errors, 0 warnings.` 日志为 `validation/ads-key-wifi-responsive-build.log`；map 中已确认新增延后执行函数和状态被链接。
- 本次未烧录，未验证实物按键电平、扫描中断及联网等待期间的实际响应延迟。该修复针对已定位的联网阻塞；若无联网操作也失灵，需要结合屏幕是否刷新继续检查硬件和调试暂停状态。

## 仅保留 EKF（2026-10-23）

删除 Mahony、Madgwick 实现和选型宏，以及未使用的旧 `code/icm.c/.h`。运行入口直接使用 `Ekf6` 和 `ekf6_*`；控制输出统一命名为 `attitude_state`。保留当前 IMU660RB 轴映射、标定门限、200 Hz 采样、故障处理和遥测协议值 3。

主机回归通过：`check_imu.py`、`check_calibration.py`、`check_balance_axes.py`、`check_roll_rate_loop.py`、`check_rate_menu.py`、`check_rate_flash.py`、`check_auto_imu_host.py`、`check_imu_display.py`、`check_imu_wifi_boot.py`、`check_menu_keys.py`、`check_wifi_host.py`、`check_wifi.py`、`check_spi_timeout.py`。三轴复合转动改用独立解析轨迹验证，不再依赖另一种姿态滤波算法。

本次未重新生成 ADS 板端固件，以下历史固件的构建记录和哈希不代表本次源码。需要在 ADS 刷新工程后重新编译、烧录。

## WiFi-SPI UDP 移植（2026-10-21）

- 来源：MCARpreparation 当前工作区 `project/code/wifispi.c/.h` 与网络配置。TC264 保留本项目 SPI3 驱动；CPU0 主循环收发，时间基准改为 STM0，CPU1 TCP 图传与 UDP 遥测按编译开关互斥。
- ADS 1.10.10 / TASKING 1.1r8，TC26B Debug，IDE Clean 后完整重编译 193 个 C 翻译单元，结果：`16:00:37 Build Finished. 0 errors, 3 warnings.` 新 `code/wifispi.c` 已编译和链接，日志见 `validation/ads-wifi-spi-build.log`。
- 3 条警告均为现有 `code/attitude.c` 第 117、174、306 行的 W542（聚合初始化缺少嵌套括号）；本次未修改该文件。新增 WiFi 代码无编译警告。
- ROM：118847 字节；RAM：182429 字节。map 显示 CPU0 DSPR 剩余 10868 字节，CPU1 DSPR 剩余 3311 字节，两核用户栈已分别预留 8 KiB。
- `python tests/check_wifi.py` 通过：使用真实传输源文件，模拟 STM 和 SPI；验证 JustFloat 小端字节及容量、UDP 数据报、订阅替换、滑块分段／连续／引号解析、错误及超长输入、暂停流仍可接收命令、关中断边界、STM 回绕、发送失败、初始化分阶段失败／重试和显式断开。禁用配置不链接任何 SPI/STM/IMU/控制硬件符号。
- `python tests/check_menu_tree.py`、`python tests/check_menu_keys.py` 通过。测试输出记录见 `validation/wifi-host-tests.log`，主机 C 测试使用 `.c.in` 后缀，不会编入固件。
- 尚未烧录，未验证板上 WiFi-SPI 2.0 通信、UDP 联机、实际发送周期和控制实时性。网络和操作说明见 [WIFI.md](WIFI.md)。

| 文件 | SHA-256 |
| --- | --- |
| `Debug/Qcar.elf` | `30cd1baf0eae2c5163c456efc506e3e49a4d43c4179be4526a493e68140e493e` |
| `Debug/Qcar.hex` | `3ced7e0a1da9a57969f46ac2ac5a4c10c91cd227f5de4128c0241000550bfd97` |

## IMU660RB / EKF 移植（2026-10-20）

- ADS 1.10.10 / TASKING 1.1r8，TC26B Debug（O0）：`22:48:20 Build Finished. 0 errors, 0 warnings.`
- 编译并链接新增 `code/attitude.c` 和 `code/imu.c`，测试代码未链接进固件。控制台记录：`validation/ads-imu-rb-gui-build.log`。
- ROM：110025 字节；RAM：181863 字节。
- `python tests/check_imu.py`：Mahony、Madgwick、EKF 三种配置的 RB 采样流水线测试及原项目 EKF 数值回归全部通过，记录见 `validation/imu-host-tests.log`。主机测试源文件使用 `.c.in` 后缀，避免 ADS 自动扫描为固件源文件。
- 菜单树主机检查 `python tests/check_menu_tree.py` 通过。
- 未烧录，未测 RB 实际通信、安装轴方向、板上运行时间和闭环控制效果。200 Hz 角速度发布及统一为 °/s 后，需要重新确认控制参数。详见 `IMU.md`。

本次产物 SHA-256：

| 文件 | SHA-256 |
| --- | --- |
| `Debug/Qcar.elf` | `6518be9a5eda03bd271012dfbb12352ba7fdcd361058812dda3715838b9ad1f6` |
| `Debug/Qcar.hex` | `5ff3e2fc7c84d2fb772a7ef5fbd9eadacb4d325c27d4a39da465aa92f0f792e9` |

日期：2026-10-20。目标：TC264D / `tc26xb`。

## ADS 干净构建

使用单独工作区 `../.ads-qcar-gui` 导入 Qcar，清除本次生成的 Debug 构建目录，F5 刷新后在 ADS 中重新构建。没有使用上游 TC377 的预生成目标文件。

- ADS：1.10.10。
- 编译器：TASKING 1.1r8 Build 22011964。
- 配置：Debug，示例原有 O0 设置。
- 实际编译 189 个 C 翻译单元，汇编及 TC26B 链接完成。
- 控制台结果：`17:44:44 Build Finished. 0 errors, 0 warnings. (took 4s.650ms)`。
- 完整控制台输出保存在 `validation/ads-clean-build.log`。
- `elfsize` 报告：ROM 123807 字节，RAM 177595 字节。
- map 报告：CPU0 DSPR 剩余 `0x2a74`（10868）字节，CPU1 DSPR 剩余 `0x1fd1`（8145）字节；两核各预留 8 KiB 用户栈。

产物 SHA-256：

| 文件 | SHA-256 |
| --- | --- |
| `Debug/Qcar.elf` | `fa39ec92b6778a3ee0d248f974fcbad126a6a5aad5c1f24c43c14bda321e4118` |
| `Debug/Qcar.hex` | `81cb9ed7b36df2dcdbdb7d69213b774c78def16a3a007761fb8a2a5ceecc1f9f` |

## 连通域边界检查

运行 `python tests/check_labels.py`（需要本机 GCC）。测试使用实际 `code/component.c` 与实际容量配置，仅为主机测试替换硬件头文件和像素到世界坐标的函数。

对 `label_components`、`label_components_change` 分别检查：空图像无目标、全白图像单个目标的面积与均值、远超 32 个孤立亮点时限制临时标签数，且输出数组两侧哨兵未被覆盖。共六组场景通过：

```text
PASS: empty, full-frame and label-overflow images, both algorithms
```

该检查不验证像素标定、视觉识别准确率或实车控制性能。

## 尚未验证

未连接 TC264 开发板，未烧录，未做 IMU/相机/驱动通信、控制中断执行时间、栈峰值、Flash 掉电、实车平衡和赛道验证。实际主板型号及引脚表尚未提供；接线采用 README 中列出的默认方案。TC264 上视觉处理帧率需实测。

## UTF-8 ???????

2026-10-20????? UTF-8????????????????? C/H ??????ADS ??????18:36:28?0 errors?0 warnings???? `validation/ads-utf8-build.log`????????????????????????

| ?? | SHA-256 |
| --- | --- |
| `Debug/Qcar.elf` | `17e9f524bc48a0a732f65cfbeddd8252bd072996bcbad21df3ff3dc106ea045c` |
| `Debug/Qcar.hex` | `81cb9ed7b36df2dcdbdb7d69213b774c78def16a3a007761fb8a2a5ceecc1f9f` |

## ?????????

2026-10-20??????? 353 ?????????? 19 ?????????? 1 ?????? C/H ????? UTF-8 ????????????????????????????????????????????ADS ?????0 errors?0 warnings???? `validation/ads-comments-build.log`??? ADS ???????????????????

???????

- `Debug/Qcar.elf`?`1d98eab31900e42809e6850b21a8aa82e0aae7a4936281bef91c1f5720fb755b`
- `Debug/Qcar.hex`?`81cb9ed7b36df2dcdbdb7d69213b774c78def16a3a007761fb8a2a5ceecc1f9f`

## Tree menu replacement

ADS Debug build: 21:50:20, 0 errors, 0 warnings. Log: `validation/ads-tree-menu-build.log`. ROM: 100785 bytes; RAM: 181496 bytes. CPU1 DSPR free: 4244 bytes, with stacks already reserved. Host test `tests/check_menu_tree.py`: PASS (empty/single/nested circular trees, repeat initialization, pool capacity). Hardware input, screen refresh and vehicle modes have not been tested on a board.

- `Debug/Qcar.elf`: `db5fc983ec6262fbc539a5d8404c3123c094128a20b71cda1f8a668fb6eb281a`
- `Debug/Qcar.hex`: `784e3e9ad33fe93349460e997edf6f4f33ce7b4b384e3ec1f9140ddfa2ed1911`

## Reference menu key scheme (2026-10-21)

All files are created directly in Menu_Create with Create_Menu_File_dynamic(parent, name, address, kind). PID entries are explicit; add_* and MENU_Action removed. Actual Key_* / Menu_Switch behavior matches the reference key mapping, event priority, clearing and step cycle. Typed edit and tree tests pass. ADS build: 15:32:08, 0 errors, 0 warnings; log: validation/ads-strict-menu-build.log. No firmware download or hardware key test performed.

## Four-key remapping (2026-10-21)

KEY_1 up/increase; KEY_2 down/decrease; KEY_3 enter/select/cycle step down; KEY_4 deselect/back. Actual key implementation host tests pass. ADS build passes with 0 errors and 0 warnings; log: validation/ads-key-remap-build.log. No firmware download performed.

## Key debounce and lost-event fix (2026-10-21)

The old short-press flag lasted only one 1 ms scan; the 10 ms+ main loop could miss it. Replaced with stable-edge debounce in milliseconds and per-key queued events, consumed atomically. The menu now consumes only the selected key and leaves lower-priority events pending. Short-press actions occur after 10 ms of stable press; release is debounced and holding does not repeat navigation.

Actual-driver host test check_key_debounce.py and actual-menu test check_menu_keys.py pass: 1/5/10 ms scan periods, delayed polling, press/release bounce, rapid presses, long hold, concurrent keys and interrupt-state restoration. ADS build: 15:58:59, 0 errors, 0 warnings. Log: validation/ads-key-debounce-build.log. No hardware timing measurements or firmware download performed.

- `Debug/Qcar.elf`: `5db5e6d1c189e8976aeb7d7d49b7edff77e9b17e578f6d9e343e3ef1f2f0df7f`
- `Debug/Qcar.hex`: `3ced7e0a1da9a57969f46ac2ac5a4c10c91cd227f5de4128c0241000550bfd97`
