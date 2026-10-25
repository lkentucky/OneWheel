# 直立模式验收（2026-10-25）

基于直立精简提交3a0e79c，增加上电默认Mode5、Mode8双轴PWM开环调试、菜单及WiFi参数。未烧录、未实车验证。

- ADS 1.10.10 / TASKING 1.1r8 Build 22011964，TC26B Debug，本轮增量构建：**0 errors / 0 warnings**。
- ROM 101722 B，RAM 30446 B（工具输出data+bss，不是全部运行期栈/堆峰值）。
- 编译副本的613个C/H/静态库与交付源码逐文件SHA256一致。
- 已生成并放回Debug/OneWheelUpright.hex、.elf、.map。
- 主机14组测试全部通过，断言显式启用；使用Zig 0.13.0自带Clang编译真实C实现。

覆盖：位置式/增量式PID、变dt/溢出/复位、双轴P/P原公式对照、限幅/启动斜坡/倾倒/IMU过期/停机；Flash往返、校验、部分写入和提交标记；IMU采样/轴映射/标定/时间回绕/超时及EKF轨迹；WiFi订阅/参数/坏报文/重试；SPI超时；按键消抖、菜单导航/延后动作及开环测试。

新增覆盖：Mode8独立有符号输出、正反转、PWM范围/NaN故障锁存、启动限幅、无IMU时启动、默认Mode5、实际菜单参数绑定、重复Start/Stop/切换模式清零、回到闭环后仍要求IMU；WiFi新增参数写入/拒绝和调试字段订阅。

主机测试使用假的硬件边界，不验证真实DFlash擦写、无线连通、驱动电气响应或控制稳定性。当前仍是角度/角速度直立控制，没有速度/位置外环。

参数和本地IP已保留：地轮KP30、角度KP40；飞轮KP35、角度KP50；机械零点X3.9°/Y-0.3°；角度低通tau0.06s；PC192.168.0.112:8081，模块命令端口5001。

HEX SHA256：`9f619cc1aff4c360baeb1c1ef8e1d024c72c0ab47355fdfb874c01ffc2848e4b`。

复现主机测试：`python tests/run_all.py`（GCC）或追加`--compiler <zig.exe>`。ADS工程名OneWheelUpright，仅保留TASKING Debug配置。初始化与入口见README。
