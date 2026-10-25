# Mode8左右独立PWM验收（2026-10-25）

基于编码器修正提交94091b1，将Mode8拆成左飞轮、右飞轮、行进轮三路独立PWM。Mode3～7闭环和TIM2/P33.7/P33.6编码器配置保留。本轮仅执行编译，未由Codex触发烧录；随后ADS控制台记录Flash Pass，实车运行和电机方向尚未验证。

- ADS 1.10.10 / TASKING 1.1r8 Build 22011964，TC26B Debug，本轮增量构建：**0 errors / 0 warnings**。
- ROM 101969 B，RAM 30450 B（工具输出data+bss，不是全部运行期栈/堆峰值）。
- 直接在英文路径C:/Users/Lenovo/Desktop/oneWheel编译，记录613个C/H/静态库SHA256；编译后源码与清单一致。
- 已生成Debug/OneWheelUpright.hex、.elf、.map。
- 本轮14组主机回归全部通过，断言显式启用；使用Zig 0.13.0自带Clang编译真实C实现。
- 确认balance_control.c、menu.c、wifispi.c重新编译，目标汇编包含左右独立PWM变量。

覆盖：位置式/增量式PID、变dt/溢出/复位、双轴P/P原公式对照、限幅/启动斜坡/倾倒/IMU过期/停机；Flash往返、校验、部分写入和提交标记；IMU采样/轴映射/标定/时间回绕/超时及EKF轨迹；WiFi订阅/参数/坏报文/重试；SPI超时；按键消抖、菜单导航/延后动作及开环测试。

新增覆盖：Mode8单独左转/右转、左右不同值、正反向和限幅、任一路参数异常停三路并锁存、启动斜坡、无IMU时启动、默认Mode5、三个实际菜单绑定、Start/Stop/切换模式清零、闭环IMU门槛；WiFi左右独立写入/订阅、拒绝旧配对指令以及保持右路实际下发符号。

主机测试使用假的硬件边界，不验证真实DFlash擦写、无线连通、驱动电气响应或控制稳定性。当前仍是角度/角速度直立控制，没有速度/位置外环。

编码器当前仅完成初始化，未接入周期计数读取和遥测；引脚编译验证不代表实车已经有计数。

参数和本地IP已保留：地轮KP30、角度KP40；飞轮KP35、角度KP50；机械零点X3.9°/Y-0.3°；角度低通tau0.06s；PC192.168.0.112:8081，模块命令端口5001。

HEX SHA256：`b165cfa8083354cf867f9486e7a5c1f1f21c68b278f413fad87ae84cd32921f8`。

复现主机测试：`python tests/run_all.py`（GCC）或追加`--compiler <zig.exe>`。ADS工程名OneWheelUpright，仅保留TASKING Debug配置。初始化与入口见README。
