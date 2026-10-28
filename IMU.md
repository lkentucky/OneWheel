# IMU660RB 与 EKF

`imu.c`使用SPI0读取RB，硬件加速度/陀螺ODR均为833Hz，软件每2ms采样一次（500Hz）。CTRL1_XL/CTRL2_G均配置0x7C，保留±8g、±2000°/s量程。机体X/Y/Z=传感器Z/-Y/X；角度单位度，控制角速度单位度/秒。所有可调阈值、寄存器配置、轴映射与SPI接线在 `code/config.h`。

833Hz ODR编码0111见[ST LSM6DSR数据手册](https://www.st.com/resource/en/datasheet/lsm6dsr.pdf)。上电会回读配置寄存器，主机测试不能代替实板ODR和实际执行耗时确认。

启动时采集500个连续合格的静止样本（约1秒），完成零偏校准。运动、加速度范围、峰峰值或平均角速度越界会拒绝标定。EKF保留加速度低通、创新门限和X/Y残余零偏估计；未接入、无调用的静止Z零偏跟踪已删除。六轴系统没有绝对航向参考，Yaw长期会漂移。

SPI访问有超时；无ready/总线坏帧不发布有效样本。超过MAX_SAMPLE_DT但未超过MAX_SKIP_DT的间隔丢帧，更大间隔报dt错误；无成功样本达到IMU_TIMEOUT_MS报超时。控制层另有25ms采样过期保护。Flash保存后重新建立采样时间基准。

状态：0标定，1正常，-1初始化失败，-2超时，-3采样间隔异常，-4滤波失败。上位机需同时看imu_status、imu_seq、imu_dt_ms和控制fault_reason。

屏幕Roll/飞轮使用imu_pitch_deg、gyro.y；屏幕Pitch/行进轮使用imu_roll_deg、gyro.x。控制环直接使用EKF姿态角和校正后的陀螺仪角速度，无额外角度/角速度低通；保留EKF内部加速度低通和有效性门限。

`tests/check_imu.py`编译实际采样/解算代码验证标定、映射、STM回绕、超时/恢复和航向跨界，并运行独立EKF数值轨迹。该测试不代替实物安装方向和运行耗时测量。
