# 直立精简验收（2026-10-25）

基于上游7a053b2的直立精简版本，未烧录、未实车验证。

- ADS 1.10.10 / TASKING 1.1r8 Build 22011964，TC26B Debug：**0 errors / 0 warnings**。
- ROM 100385 B，RAM 30437 B（工具输出data+bss，不是全部运行期栈/堆峰值）。
- 编译副本的613个C/H/静态库与交付源码逐文件SHA256一致。
- 已生成并放回Debug/OneWheelUpright.hex、.elf、.map。
- 主机13组测试全部通过，断言显式启用；使用Zig 0.13.0自带Clang编译真实C实现。

覆盖：位置式/增量式PID、变dt/溢出/复位、双轴P/P原公式对照、限幅/启动斜坡/倾倒/IMU过期/停机；Flash往返、校验、部分写入和提交标记；IMU采样/轴映射/标定/时间回绕/超时及EKF轨迹；WiFi订阅/参数/坏报文/重试；SPI超时；按键消抖、菜单导航/延后动作及开环测试。

主机测试使用假的硬件边界，不验证真实DFlash擦写、无线连通、驱动电气响应或控制稳定性。当前仍是角度/角速度直立控制，没有速度/位置外环。

参数和本地IP已保留：地轮KP30、角度KP40；飞轮KP35、角度KP50；机械零点X3.9°/Y-0.3°；角度低通tau0.06s；PC192.168.0.112:8081，模块命令端口5001。

HEX SHA256：`d8e7ed5fb28f3655b05b7ab582592e9ac292bef0e948fb0eea060870447e80ac`。

复现主机测试：`python tests/run_all.py`（GCC）或追加`--compiler <zig.exe>`。ADS工程名OneWheelUpright，仅保留TASKING Debug配置。初始化与入口见README。
