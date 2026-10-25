# 直立菜单

参数值集中在 `code/config.h`，菜单绑定相应RAM变量。

- Startup：Mode3～8、Start、Stop、IMU、Open loop、Wheel PWM、WiFi connect；上电默认Mode5但不自动Start。
- PID / Gyro test：飞轮KP、PWM limit、Sign、Angle KP、Rate limit、Balance Y。
- PID / Pitch balance：行进轮同类参数、Balance X。
- Storage：Save、Load；要求停车并退出预览。
- PWM debug：Fly left PWM、Fly right PWM、Wheel PWM，分别独立控制Mode8左飞轮、右飞轮和行进轮，默认±4000；左右不自动同步或取反。先在Startup选择8并Start，再设置PWM；每次Start/Stop/切换模式都会清零三路。不依赖IMU，无PID/倾角保护，需架空调试。

独立WiFi页面、Vehicle、Vision、Trajectory及旧速度级联PID页面已移除。WiFi连接功能仍在Startup中。

K1上移/加，K2下移/减，K3进入/选中/循环步进，K4返回/取消选中。K4不是紧急停车键；使用Startup/Stop。WiFi等待中继续处理导航；硬件动作在SPI事务退出后执行。运行时改变模式先停车，再显式Start。

状态码、控制链和保存机制见 [README](README.md)。菜单树最多64节点，源代码见 `menu_tree.c`；按键消抖及周期也在config中。
