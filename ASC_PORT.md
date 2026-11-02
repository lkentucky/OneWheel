# ASC 控制结构与 A+ 姿态模块移植

分支：port/asc-control-r10h。来源：Kongmengwuwei/ASC_SummerTraining_Unicycle，固定提交 0eed51b8fd0f4e72ab92f737642ab1900f60cbd3。原 Qcar 工作区保持 main，本分支位于独立工作区。

## 控制结构

Mode10：飞轮回收→位置式角度→位置式角速度；地轮速度→位置式角度→增量式角速度；yaw 为位置式航向→位置式角速度。Roll优先，yaw仅使用剩余输出余量。

IMU、姿态发布、角度与角速度环均为1ms；速度与飞轮回收环为20ms。Mode10为零速度保持，不含位置环；原Mode2保留位置保持。没有引入视觉、压弯或路径导航。保留Qcar引脚、菜单、WiFi和驱动协议，不自动启动。

## 姿态模块移植注意事项

依据参考仓库 docs/design/Aplus-r10h移植说明.md、方向基准整理.md、角度外环1kHz试验说明.md 与 docs/guides/控制调试指南.md。

- 两传感器ODR为1667Hz（CTRL1_XL/CTRL2_G=0x8c），±8g、±2000°/s；4098 counts/g、14.3 counts/(°/s)。保留本车SPI、BDU/递增及其他既有寄存器配置。
- 一次六轴突发读取，每次SPI事务100µs截止，检测DRDY、身份、坏包和饱和端点。
- A+输入原始物理量，不在外部重复扣零偏或滤波；控制使用单独的静止零偏修正陀螺仪。核心仍用未标定单位矩阵，不能冒称本车已完成矩阵标定。
- 静止采样6000个有效点，运动检查失败重新累计；之后成功更新3000拍才允许电机闭环，合计约9秒。
- 固定1ms调用，750～1250µs以外或解算耗时≥1000µs锁止；故障不自动追帧/恢复，停机后重新静止初始化。
- CPU0中断栈增为4KB，状态放在cpu0_dsram；实际最坏时延与栈高水位仍需实车测量。

## 坐标与方向

先按原 IMU_BODY_* 得到Qcar车体系B，再将加速度、陀螺仪和启动均值统一旋转到参考控制体系R：

    R.x = B.y
    R.y = -B.x
    R.z = B.z

这是行列式+1的右手旋转。A+四元数转换回B后继续供旧界面使用；Mode10使用R中的角度。机械零点从旧roll/pitch对应的重力方向做同一旋转，再计算参考零点，没有直接套用参考车零点。

Pitch角速度反馈为 q*cos(roll)-r*sin(roll)，使用绝对Roll，不扣机械零点。asc_roll_actual_deg、asc_pitch_actual_deg、asc_yaw_actual_deg 是参考控制反馈，yaw连续累计。六轴yaw不能提供绝对北向。

原Qcar两路最终极性默认均为-1；适配后实现参考混控 A=-Roll+Yaw、B=Roll+Yaw。地轮通过原方向配置换算并保留正反向摩擦补偿。安装方向或电机极性变更后仍需核对物理负反馈。

## 参数单位

角度PID仍按5ms参考：内部Ki×0.2、Kd×5、积分累计限幅×5。内环和速度环为每拍算法，不能与旧秒制Kd直接互换。

| WiFi参数 | 默认值 | 说明 |
|---|---:|---|
| fly_angle_kp / fly_angle_kd | 36.00006 / 0 | 位置式角度环，Kd按5ms参考 |
| fly_rate_kp / fly_rate_ki / fly_rate_kd | 20 / 0 / 8 | 位置式内环，Kd内部有效系数为负，每拍差分 |
| fly_vel_kp / fly_vel_kd | 0.0015 / 0 | 直接输入左右RPM差，不再乘0.005 |
| ground_angle_kp | 8.999999 | 地轮角度环 |
| ground_rate_kp / ground_rate_kd | 20 / 0 | 增量式内环；rate_kp是地轮Kp写别名 |
| asc_ground_rate_ki | 0.6400001 | 地轮内环每拍Ki |
| asc_ground_speed_kp / ki / kd | 0.06 / 0.002 / 0 | 输入counts/20ms，由本车counts/s×0.020转换 |
| asc_yaw_angle_kp / asc_yaw_rate_kp | 18.00003 / 56 | yaw内环有效比例为负Kp |
| asc_yaw_enabled | 1 | 只能写0/1，每次Start捕获当前航向 |

旧 wheel_speed_*、wheel_pos_* 属于Mode2。默认值来自参考源码，不是本车已完成的调参结果。

参考限制在本分支恢复：飞轮目标相对零点±10°、地轮速度环倾角±8°、飞轮PWM±10000、地轮控制PWM±8000、Roll/Pitch保护20°/50°、飞轮8000RPM保护，以及IMU/驱动/编码器异常停机。

只测飞轮时，保持Mode10，地轮角度Kp、角速度Kp/Kd、asc_ground_rate_ki、asc_ground_speed_kp/ki/kd及两向死区补偿归零，再将asc_yaw_enabled设0。单测内环还需关闭飞轮角度及转速外环，并始终支撑车身。

## 保存、编译与验证

使用新Flash标识ASC1，保存新内环D、独立速度参数、yaw和摩擦补偿；拒绝旧OWB2记录，不默默迁移不同单位。标记最后写入、CRC及回读验证仍有效。

保留 QCAR_BOOT_LOAD_FLASH=0：Save可写入，但启动/Load使用编译默认值。需要自动加载新记录时改为1并重新编译。旧Flash里的现场机械零点不会迁移，须先记录并设置到新配置。

保留460800占空比串口，默认文本SET-DUTY以兼容既有驱动。未替换驱动板FOC/六步换相固件，该接口不等于已标定电流或转矩指令。

    python tests/run_all.py --report .ads/asc-full-final.json
    python tools/ads/build.py --ads C:/Infineon/AURIX-Studio-1.10.10

32项回归与ADS构建通过，未烧录、未做本分支实车平衡验证。首次使用需先静止标定、核对轴向与1ms时序，再在支撑条件下逐环验证。详见ASC_PORT_VALIDATION.md和docs/asc-port-manifest.json。