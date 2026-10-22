# 逐函数与逐宏机制索引

由五项修复后的源码快照生成；新增/变更机制优先见 [SAFETY_FIXES_20261022.md](SAFETY_FIXES_20261022.md)。每个函数均有说明。修复前主讲解见 [CODE_WALKTHROUGH.md](CODE_WALKTHROUGH.md)，问题见 [CODE_REVIEW_20261022.md](CODE_REVIEW_20261022.md)。

## 全部 128 个实际函数

下表不把全文件注释的 four_posture 算作执行函数；其旧算法和空模块在主讲解第23节说明。行号对应当前固件文件。

| 源码位置 | 函数 | 作用及主讲解章节 |
| --- | --- | --- |
| [code/app.c:13](../Seekfree_TC264_Opensource_Library/code/app.c#L13) | `app_boot_diagnostics_poll()` | 第5、15节：CPU0 STM节流，UART0与WiFi日志输出BOOT；不依赖控制tick |
| [code/app.c:48](../Seekfree_TC264_Opensource_Library/code/app.c#L48) | `app_cpu1_now_ms()` | 第5、6节：STM低32位差值及余数累计，供CPU1后台调度 |
| [code/app.c:63](../Seekfree_TC264_Opensource_Library/code/app.c#L63) | `app_cpu0_init()` | 第3节：时钟/延时、debug、通信内存初始化，发布CPU0 ready |
| [code/app.c:83](../Seekfree_TC264_Opensource_Library/code/app.c#L83) | `app_cpu0_poll()` | 第15节：累计CPU0循环，输出诊断，服务WiFi或延时 |
| [code/app.c:91](../Seekfree_TC264_Opensource_Library/code/app.c#L91) | `app_init()` | 第4、修复说明节：双核交接和外设初始化，先独立输出PIT再控制PIT |
| [code/app.c:150](../Seekfree_TC264_Opensource_Library/code/app.c#L150) | `app_buttons_poll()` | 第6.1、7节：独立周期扫描，停止优先，K1启动请求，拒绝日志 |
| [code/app.c:169](../Seekfree_TC264_Opensource_Library/code/app.c#L169) | `app_camera_poll()` | 第22.1节：关闭/未ready/无新帧时返回；复制、阈值、二值、信标、新帧发布；当前调用注释 |
| [code/app.c:184](../Seekfree_TC264_Opensource_Library/code/app.c#L184) | `app_camera_display_poll()` | 第22.4节：多个开关和新帧门槛；二值缩图、红绿候选框、误差；当前调用注释 |
| [code/app.c:206](../Seekfree_TC264_Opensource_Library/code/app.c#L206) | `app_telemetry_poll()` | 第6.2节：run变化日志，CPU1快照发布，可选CSV |
| [code/app.c:224](../Seekfree_TC264_Opensource_Library/code/app.c#L224) | `app_status_log_poll()` | 第6.2节：按STM时间输出CPU1阶段/tick/dtick/dms/IMU/run |
| [code/app.c:242](../Seekfree_TC264_Opensource_Library/code/app.c#L242) | `app_poll()` | 第6节：清新帧，扫描键，预留视觉，遥测、状态、10ms延时 |
| [code/balance_control.c:22](../Seekfree_TC264_Opensource_Library/code/balance_control.c#L22) | `balance_cascade_init()` | 第9、11节：配置两路滤波和9个PID，保存初始结构体备份 |
| [code/balance_control.c:125](../Seekfree_TC264_Opensource_Library/code/balance_control.c#L125) | `pid_control()` | 第11.1节：位置式PID，积分与输出限幅，保存前次误差 |
| [code/balance_control.c:147](../Seekfree_TC264_Opensource_Library/code/balance_control.c#L147) | `first_order_complementary_filtering()` | 第9、修复说明节：互补滤波；双向越界/非有限状态锁存故障并保留异常符号 |
| [code/balance_control.c:169](../Seekfree_TC264_Opensource_Library/code/balance_control.c#L169) | `pid_control_incremental()` | 第11.1节：增量式PID输出累加、误差历史、输出限幅；不用I_SCALE/I_LIMIT |
| [code/balance_control.c:180](../Seekfree_TC264_Opensource_Library/code/balance_control.c#L180) | `change_mechanical_zero()` | 第23节：未调用；视觉压弯零点候选与编码器相关限幅，未接入真实滤波 |
| [code/balance_control.c:187](../Seekfree_TC264_Opensource_Library/code/balance_control.c#L187) | `balance_cascade_reset()` | 第7、11.1节：清9环状态、保留增益和限幅 |
| [code/camera.c:16](../Seekfree_TC264_Opensource_Library/code/camera.c#L16) | `detect_beacon()` | 第22.3节：窗口亮度主/次候选、位置权重交换、像素误差、丢标回退；当前关闭 |
| [code/camera.c:122](../Seekfree_TC264_Opensource_Library/code/camera.c#L122) | `Ostu()` | 第22.2节：直方图/概率/累计均值/类间方差求阈值，退化帧存在V1缺陷 |
| [code/camera.c:170](../Seekfree_TC264_Opensource_Library/code/camera.c#L170) | `Image_Binarization()` | 第22.1节：原摄像头数组按阈值二值化，顶部清零；当前关闭 |
| [code/camera.c:192](../Seekfree_TC264_Opensource_Library/code/camera.c#L192) | `retarder_reset()` | 第7、12节：清现行目标速度斜坡的持久状态 |
| [code/camera.c:193](../Seekfree_TC264_Opensource_Library/code/camera.c#L193) | `retarder()` | 第12节：现行1ms双向定步长速度斜坡，不越过目标 |
| [code/camera.c:217](../Seekfree_TC264_Opensource_Library/code/camera.c#L217) | `en_retarder()` | 第23节：未调用的独立静态斜坡，无reset接口 |
| [code/camera.c:242](../Seekfree_TC264_Opensource_Library/code/camera.c#L242) | `speed_expect()` | 第23节：未调用的独立静态斜坡，返回int截断 |
| [code/motor.c:23](../Seekfree_TC264_Opensource_Library/code/motor.c#L23) | `Motor_init()` | 第4、13.2节：行进编码器、PWM零值、DIR前向初始化 |
| [code/motor.c:38](../Seekfree_TC264_Opensource_Library/code/motor.c#L38) | `motor_output_service()` | 第修复说明节：CPU1执行强制低、等待零周期、DIR建立、恢复PWM四状态 |
| [code/motor.c:90](../Seekfree_TC264_Opensource_Library/code/motor.c#L90) | `change_the_speed()` | 第23节：未调用的旧视觉分段/二次速度目标 |
| [code/motor.c:104](../Seekfree_TC264_Opensource_Library/code/motor.c#L104) | `bujin_motor_contron()` | 第13.2、修复说明节：保护、方向与死区后只暂存行进指令，实际PWM交CPU1 |
| [code/posture_control.c:19](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L19) | `imu_data_get()` | 第8节：读6轴、去陀螺零偏、小量死区、int16状态低通 |
| [code/posture_control.c:39](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L39) | `icm_init()` | 第8节：首次6轴读数初始化两份IMU状态 |
| [code/posture_control.c:59](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L59) | `control_stop()` | 第7、修复说明节：发布停止代次、清PID/斜坡及暂存输出；硬件由独立CPU1 ISR清零 |
| [code/posture_control.c:74](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L74) | `control_request_stop()` | 第7、修复说明节：原子停止代次及软件停止，输出不再等待CPU0控制ISR |
| [code/posture_control.c:81](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L81) | `control_request_start()` | 第7、修复说明节：除原门槛外增加新鲜控制/反馈、故障锁存与停止代次检查 |
| [code/posture_control.c:93](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L93) | `dynamic_value_control()` | 第13.1、修复说明节：飞轮PID组合、非有限检查、限幅；只暂存，不发UART |
| [code/posture_control.c:119](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L119) | `control_feedback_poll()` | 第10、修复说明节：20ms编码器/有效速度对，5ms两路滤波；测速TX已迁CPU1 |
| [code/posture_control.c:139](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L139) | `control_step()` | 第修复说明节：CPU0控制计算与保护步骤，只写暂存输出，不操作硬件 |
| [code/posture_control.c:216](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L216) | `pit_isr_callback()` | 第10、修复说明节：控制计数、完整control_step后发布完成时间与输出邮箱 |
| [code/posture_control.c:225](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L225) | `find_ma()` | 第23节：未调用的IMU采样旧包装 |
| [code/posture_control.c:226](../Seekfree_TC264_Opensource_Library/code/posture_control.c#L226) | `imu_filiter()` | 第23节：未调用的低通状态复制旧接口 |
| [code/safety_output.c:23](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L23) | `atomic_increment()` | 第修复说明节：硬件CAS原子递增停止代次，避免跨核请求丢失 |
| [code/safety_output.c:32](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L32) | `safety_output_epoch()` | 第修复说明节：读取停止代次 |
| [code/safety_output.c:33](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L33) | `safety_output_faults()` | 第修复说明节：读取复位才能清除的故障位 |
| [code/safety_output.c:34](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L34) | `safety_output_ticks_per_ms()` | 第修复说明节：返回输出模块STM每毫秒刻度 |
| [code/safety_output.c:36](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L36) | `safety_output_request_stop()` | 第修复说明节：原子增加停止代次并清运行标志；下一输出tick撤销输出 |
| [code/safety_output.c:43](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L43) | `safety_output_trip()` | 第修复说明节：原子锁存新故障并发布停止，不自动解除 |
| [code/safety_output.c:54](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L54) | `safety_output_feedback_ok()` | 第修复说明节：roll/yaw启用时检查飞轮速度反馈新鲜度 |
| [code/safety_output.c:60](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L60) | `safety_output_init()` | 第修复说明节：初始化独立输出邮箱/时间/事务状态，保留已收到停止请求 |
| [code/safety_output.c:72](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L72) | `safety_output_publish()` | 第修复说明节：CPU0仅在完成控制计算后发布稳定输出、时间与代次 |
| [code/safety_output.c:87](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L87) | `safety_output_can_start()` | 第修复说明节：完整控制心跳、无故障和有效反馈的启动门槛 |
| [code/safety_output.c:98](../Seekfree_TC264_Opensource_Library/code/safety_output.c#L98) | `safety_output_tick()` | 第修复说明节：CPU1唯一运行期输出者：心跳/反馈/代次/限幅，PWM状态机、非阻塞飞轮TX |
| [code/small_driver_uart_control.c:8](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L8) | `small_driver_feedback_fresh()` | 第修复说明节：仅有效完整帧的STM时间决定是否过期 |
| [code/small_driver_uart_control.c:16](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L16) | `small_driver_feedback_snapshot()` | 第修复说明节：用奇偶序列号读取完整左右速度对 |
| [code/small_driver_uart_control.c:30](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L30) | `small_driver_try_frame()` | 第修复说明节：栈帧组包后非阻塞整帧入FIFO，避免共享TX数组 |
| [code/small_driver_uart_control.c:42](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L42) | `small_driver_try_set_duty()` | 第修复说明节：运行期非阻塞占空比帧 |
| [code/small_driver_uart_control.c:47](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L47) | `small_driver_try_get_speed()` | 第修复说明节：运行期非阻塞测速请求 |
| [code/small_driver_uart_control.c:59](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L59) | `uart_control_callback()` | 第14、修复说明节：完整有效速度帧更新双速度、序列号和接收时间；坏帧不续期 |
| [code/small_driver_uart_control.c:130](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L130) | `small_driver_set_duty()` | 第14节：组A5/01左右int16大端与校验，轮询发送7字节 |
| [code/small_driver_uart_control.c:161](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L161) | `small_driver_get_speed()` | 第14节：发送A5/02查询，不等待新的速度回复 |
| [code/small_driver_uart_control.c:188](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L188) | `small_driver_init()` | 第14、修复说明节：清协议状态和反馈有效性/时间/序列号 |
| [code/small_driver_uart_control.c:213](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.c#L213) | `small_driver_uart_init()` | 第4、14节：UART3初始化、RX中断、协议清理、零指令和首次测速 |
| [code/user_value.c:18](../Seekfree_TC264_Opensource_Library/code/user_value.c#L18) | `flash_init()` | 第23节：未调用的Flash存在检查和3项参数读回；无现行保存链 |
| [code/user_value.c:28](../Seekfree_TC264_Opensource_Library/code/user_value.c#L28) | `send_int16_string()` | 第23节：整数格式化，无线UART开才发与换行 |
| [code/user_value.c:35](../Seekfree_TC264_Opensource_Library/code/user_value.c#L35) | `send_float_string()` | 第23节：浮点6位格式化，无线UART开才发与换行 |
| [code/user_value.c:44](../Seekfree_TC264_Opensource_Library/code/user_value.c#L44) | `send_two_floats()` | 第6.2、23节：实际三个浮点CSV，按核/开关送WiFi和无线UART；当前调用关闭 |
| [code/wifi_link.c:131](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L131) | `wl_put_u16()` | 第19节：u16小端写入 |
| [code/wifi_link.c:137](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L137) | `wl_put_u32()` | 第19节：u32小端写入 |
| [code/wifi_link.c:145](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L145) | `wl_put_float()` | 第19节：float位模式经memcpy按u32小端写入 |
| [code/wifi_link.c:152](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L152) | `wl_mux_header()` | 第19节：生成MUDP魔数/版本/类型/负载长度 |
| [code/wifi_link.c:160](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L160) | `wl_crc32()` | 第21节：半字节查表CRC32，图像整帧校验 |
| [code/wifi_link.c:179](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L179) | `wl_c0_append()` | 第16节：CPU0线性日志追加，容量到达后截断 |
| [code/wifi_link.c:187](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L187) | `wifi_link_log()` | 第15、16节：128字节格式化、NET串口镜像、CPU0队列追加 |
| [code/wifi_link.c:202](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L202) | `wifi_link_cpu1_log()` | 第16节：CPU1环形日志与自动LF，满则丢弃，DSYNC发布 |
| [code/wifi_link.c:229](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L229) | `wifi_link_online()` | 第15节：只返回状态机是否RUN，不证明电脑收包 |
| [code/wifi_link.c:235](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L235) | `wl_time_init()` | 第5、15节：初始化CPU0 STM毫秒时间、余数、每微秒tick |
| [code/wifi_link.c:246](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L246) | `wl_time_poll()` | 第5节：累计STM差值与余数，正常轮询支持回绕 |
| [code/wifi_link.c:261](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L261) | `wl_module_idle()` | 第15、16节：读WiFi模块INT高为空闲 |
| [code/wifi_link.c:267](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L267) | `wl_send_wire()` | 第16节：提交IO发送请求，不等于发送成功 |
| [code/wifi_link.c:272](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L272) | `wl_tx_committed()` | 第16节：成功后才消费日志、推进/释放图像，清pending |
| [code/wifi_link.c:304](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L304) | `wl_io_service()` | 第16节：推进IO结果；SENT提交，RX更新周期，错误保留日志并重连 |
| [code/wifi_link.c:327](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L327) | `wl_send_data()` | 第18、19节：ready/序列号检查，打228字节遥测，提交后更新发送周期 |
| [code/wifi_link.c:426](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L426) | `wl_send_logs()` | 第16节：CPU0队列优先，再CPU1环形队列，最多900字节，成功才消费 |
| [code/wifi_link.c:462](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L462) | `wl_send_image_slice()` | 第21节：请求/超时、整图CRC、MCAM分片提交；当前关闭 |
| [code/wifi_link.c:522](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L522) | `wifi_vision_erro()` | 第20节：严格位于视觉死区内部才返回0，当前yaw未开 |
| [code/wifi_link.c:585](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L585) | `wl_param_find()` | 第20节：按名称查32项静态参数表 |
| [code/wifi_link.c:599](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L599) | `wl_split()` | 第20节：空格/tab分词、最多6个token |
| [code/wifi_link.c:627](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L627) | `wl_reply_ok()` | 第20节：RUN时追加带ID的OK参数应答 |
| [code/wifi_link.c:636](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L636) | `wl_reply_done()` | 第20节：RUN时追加带ID的DONE批次结束标记 |
| [code/wifi_link.c:645](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L645) | `wl_reply_err()` | 第20节：RUN时追加带ID的ERR原因 |
| [code/wifi_link.c:654](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L654) | `wl_number()` | 第20节：strtod完整解析、范围和非有限数拒绝 |
| [code/wifi_link.c:666](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L666) | `wl_ipv4()` | 第20节：IPv4长度、格式和四段范围检查 |
| [code/wifi_link.c:675](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L675) | `wl_command_line()` | 第20、修复说明节：合法远程STOP优先执行，不受日志满门限；其余参数保护保留 |
| [code/wifi_link.c:771](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L771) | `wl_command_bytes()` | 第20节：跨片拼命令，CR忽略，非法/超长整行丢弃至LF |
| [code/wifi_link.c:799](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L799) | `wl_enter()` | 第15节：设置WiFi状态机状态 |
| [code/wifi_link.c:804](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L804) | `wl_backoff()` | 第15节：NET原因日志，记录下一状态和5s退避起点 |
| [code/wifi_link.c:812](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L812) | `wl_run()` | 第16节：忙事务优先、host切换、INT超时，RX/日志/数据/图像调度 |
| [code/wifi_link.c:850](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L850) | `wifi_link_init()` | 第3、15节：通信时间/IO/状态/队列/图像握手初始化，不连网 |
| [code/wifi_link.c:881](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L881) | `wifi_link_service()` | 第15节：五状态机，INIT/SOCKET阻塞、RUN分步，末尾延时 |
| [code/wifi_link.c:970](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L970) | `wifi_link_cpu1_service()` | 第18节：物理量显示换算、序列号发布快照，按需复制图像，不做SPI |
| [code/wifi_link_io.c:22](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L22) | `io_exchange()` | 第17节：一次CS交换头与32字节窗口/余载荷，返回命令与接收回调，统计时间 |
| [code/wifi_link_io.c:71](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L71) | `wifi_link_io_init()` | 第17节：清IO状态、回调、发送引用、耗时统计 |
| [code/wifi_link_io.c:82](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L82) | `wifi_link_io_busy()` | 第17节：检查IO是否非IDLE |
| [code/wifi_link_io.c:87](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L87) | `wifi_link_io_send()` | 第17节：检查空闲/指针/长度并进入DATA，保存发送缓冲引用 |
| [code/wifi_link_io.c:100](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L100) | `wifi_link_io_receive()` | 第17节：空闲时开始RX并记录超时起点 |
| [code/wifi_link_io.c:109](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L109) | `wifi_link_io_poll()` | 第17节：超时/INT检查、六状态握手、每次最多一次CS、阶段期限 |
| [code/wifi_link_io.c:181](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L181) | `wifi_link_io_last_us()` | 第17、19节：返回最近一次CS事务μs |
| [code/wifi_link_io.c:182](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L182) | `wifi_link_io_peak_us()` | 第17、19节：返回IO初始化以来事务峰值μs |
| [user/cpu0_main.c:37](../Seekfree_TC264_Opensource_Library/user/cpu0_main.c#L37) | `core0_main()` | 第3、15节：CPU0应用初始化后永远轮询通信/诊断 |
| [user/cpu1_main.c:38](../Seekfree_TC264_Opensource_Library/user/cpu1_main.c#L38) | `core1_main()` | 第4、6节：CPU1应用初始化后永远轮询按键/遥测/状态 |
| [user/isr.c:43](../Seekfree_TC264_Opensource_Library/user/isr.c#L43) | `cc60_pit_ch0_isr()` | 第24节：CPU0控制入口，清标志后调用1ms回调 |
| [user/isr.c:53](../Seekfree_TC264_Opensource_Library/user/isr.c#L53) | `cc60_pit_ch1_isr()` | 第24节：预留PIT，仅清CCU60_CH1标志 |
| [user/isr.c:63](../Seekfree_TC264_Opensource_Library/user/isr.c#L63) | `cc61_pit_ch0_isr()` | 第24节：预留PIT，仅清CCU61_CH0标志 |
| [user/isr.c:74](../Seekfree_TC264_Opensource_Library/user/isr.c#L74) | `cc61_pit_ch1_isr()` | 第24节：预留PIT，仅清CCU61_CH1标志 |
| [user/isr.c:87](../Seekfree_TC264_Opensource_Library/user/isr.c#L87) | `exti_ch0_ch4_isr()` | 第24节：P15.4/P15.5标志检查与清理，无应用动作 |
| [user/isr.c:106](../Seekfree_TC264_Opensource_Library/user/isr.c#L106) | `exti_ch1_ch5_isr()` | 第24节：P14.3分派ToF，P15.8清标志 |
| [user/isr.c:139](../Seekfree_TC264_Opensource_Library/user/isr.c#L139) | `exti_ch3_ch7_isr()` | 第24节：P02.0分派相机场同步，P15.1清标志 |
| [user/isr.c:160](../Seekfree_TC264_Opensource_Library/user/isr.c#L160) | `dma_ch5_isr()` | 第24节：相机DMA完成统一回调 |
| [user/isr.c:170](../Seekfree_TC264_Opensource_Library/user/isr.c#L170) | `uart0_tx_isr()` | 第24节：预留TX入口，当前只开嵌套，没有发送队列服务 |
| [user/isr.c:177](../Seekfree_TC264_Opensource_Library/user/isr.c#L177) | `uart0_rx_isr()` | 第24节：开关允许时读取调试环形缓冲 |
| [user/isr.c:188](../Seekfree_TC264_Opensource_Library/user/isr.c#L188) | `uart1_tx_isr()` | 第24节：预留TX入口，当前只开嵌套，没有发送队列服务 |
| [user/isr.c:196](../Seekfree_TC264_Opensource_Library/user/isr.c#L196) | `uart1_rx_isr()` | 第24节：摄像头参数串口回调 |
| [user/isr.c:203](../Seekfree_TC264_Opensource_Library/user/isr.c#L203) | `uart2_tx_isr()` | 第24节：预留TX入口，当前只开嵌套，没有发送队列服务 |
| [user/isr.c:211](../Seekfree_TC264_Opensource_Library/user/isr.c#L211) | `uart2_rx_isr()` | 第24节：无线模块统一回调 |
| [user/isr.c:220](../Seekfree_TC264_Opensource_Library/user/isr.c#L220) | `uart3_tx_isr()` | 第24节：预留TX入口，当前只开嵌套，没有发送队列服务 |
| [user/isr.c:228](../Seekfree_TC264_Opensource_Library/user/isr.c#L228) | `uart3_rx_isr()` | 第24节：飞轮驱动RX单字节解析 |
| [user/isr.c:239](../Seekfree_TC264_Opensource_Library/user/isr.c#L239) | `uart0_er_isr()` | 第24节：UART0 iLLD错误处理 |
| [user/isr.c:244](../Seekfree_TC264_Opensource_Library/user/isr.c#L244) | `uart1_er_isr()` | 第24节：UART1 iLLD错误处理 |
| [user/isr.c:249](../Seekfree_TC264_Opensource_Library/user/isr.c#L249) | `uart2_er_isr()` | 第24节：UART2 iLLD错误处理 |
| [user/isr.c:254](../Seekfree_TC264_Opensource_Library/user/isr.c#L254) | `uart3_er_isr()` | 第24节：UART3 iLLD错误处理 |

## 全部 295 个宏定义

逐项列出定义，不把同名模板和真实私有配置混淆。代码的 #if/#ifndef 依赖关系与检查在主讲解第2、25节说明。

| 源码位置 | 宏 | 原始定义值 | 解释位置 |
| --- | --- | --- | --- |
| [code/app.h:2](../Seekfree_TC264_Opensource_Library/code/app.h#L2) | `ONEWHEEL_APP_H` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/balance_control.h:9](../Seekfree_TC264_Opensource_Library/code/balance_control.h#L9) | `CODE_BALANCE_CONTROL_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/camera.h:9](../Seekfree_TC264_Opensource_Library/code/camera.h#L9) | `CODE_CAMERA_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/camera.h:11](../Seekfree_TC264_Opensource_Library/code/camera.h#L11) | `IMG_BLACK` | `0X00` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:12](../Seekfree_TC264_Opensource_Library/code/camera.h#L12) | `IMG_WHITE` | `0Xff` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:13](../Seekfree_TC264_Opensource_Library/code/camera.h#L13) | `IR_THRESHOLD` | `CAMERA_IR_THRESHOLD` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:14](../Seekfree_TC264_Opensource_Library/code/camera.h#L14) | `MIN_BEACON_AREA` | `20` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:15](../Seekfree_TC264_Opensource_Library/code/camera.h#L15) | `CENTER_THRESHOLD_X` | `(MT9V03X_W / 10)` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:16](../Seekfree_TC264_Opensource_Library/code/camera.h#L16) | `CENTER_THRESHOLD_Y` | `(MT9V03X_H / 10)` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:18](../Seekfree_TC264_Opensource_Library/code/camera.h#L18) | `GrayScale` | `256` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:19](../Seekfree_TC264_Opensource_Library/code/camera.h#L19) | `grayscale` | `256` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/camera.h:20](../Seekfree_TC264_Opensource_Library/code/camera.h#L20) | `guding_erro` | `2000` | 22：图像数值/阈值/遗留常量；MIN_BEACON_AREA等没有实际判定调用 |
| [code/config.h:6](../Seekfree_TC264_Opensource_Library/code/config.h#L6) | `ONEWHEEL_CONFIG_H` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/config.h:10](../Seekfree_TC264_Opensource_Library/code/config.h#L10) | `APP_DRIVE_ENABLE` | `(1)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:13](../Seekfree_TC264_Opensource_Library/code/config.h#L13) | `APP_CAMERA_ENABLE` | `(0)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:16](../Seekfree_TC264_Opensource_Library/code/config.h#L16) | `APP_CAMERA_DISPLAY_ENABLE` | `(0)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:19](../Seekfree_TC264_Opensource_Library/code/config.h#L19) | `APP_IPS200_ENABLE` | `(0)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:22](../Seekfree_TC264_Opensource_Library/code/config.h#L22) | `APP_WIFI_ENABLE` | `(1)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:25](../Seekfree_TC264_Opensource_Library/code/config.h#L25) | `APP_WIFI_IMAGE_ENABLE` | `(0)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:27](../Seekfree_TC264_Opensource_Library/code/config.h#L27) | `APP_KEYS_ENABLE` | `(1)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:28](../Seekfree_TC264_Opensource_Library/code/config.h#L28) | `APP_WIRELESS_UART_ENABLE` | `(0)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:29](../Seekfree_TC264_Opensource_Library/code/config.h#L29) | `APP_CSV_DEBUG_ENABLE` | `(0)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:30](../Seekfree_TC264_Opensource_Library/code/config.h#L30) | `APP_MAIN_LOOP_DELAY_MS` | `(10u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:31](../Seekfree_TC264_Opensource_Library/code/config.h#L31) | `APP_KEY_SCAN_PERIOD_MS` | `(10u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:32](../Seekfree_TC264_Opensource_Library/code/config.h#L32) | `APP_TELEMETRY_PERIOD_MS` | `(10u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:33](../Seekfree_TC264_Opensource_Library/code/config.h#L33) | `APP_IMU_INIT_ATTEMPTS` | `(3u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:34](../Seekfree_TC264_Opensource_Library/code/config.h#L34) | `APP_IMU_RETRY_DELAY_MS` | `(50u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:35](../Seekfree_TC264_Opensource_Library/code/config.h#L35) | `APP_STATUS_LOG_PERIOD_MS` | `(1000u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:36](../Seekfree_TC264_Opensource_Library/code/config.h#L36) | `APP_CONTROL_WARMUP_MS` | `(2000u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:37](../Seekfree_TC264_Opensource_Library/code/config.h#L37) | `SAFETY_OUTPUT_PERIOD_MS` | `(1u)` | 修复说明：独立输出周期或故障模块标记 |
| [code/config.h:38](../Seekfree_TC264_Opensource_Library/code/config.h#L38) | `CONTROL_HEARTBEAT_TIMEOUT_MS` | `(10u)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:39](../Seekfree_TC264_Opensource_Library/code/config.h#L39) | `MOMENTUM_FEEDBACK_TIMEOUT_MS` | `(100u)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:40](../Seekfree_TC264_Opensource_Library/code/config.h#L40) | `MOTOR_UART_TX_TIMEOUT_MS` | `(5u)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:41](../Seekfree_TC264_Opensource_Library/code/config.h#L41) | `MOTOR_DIR_SETUP_US` | `(100u)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:43](../Seekfree_TC264_Opensource_Library/code/config.h#L43) | `APP_BOOT_DIAGNOSTICS_ENABLE` | `(1)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:44](../Seekfree_TC264_Opensource_Library/code/config.h#L44) | `APP_BOOT_DIAGNOSTICS_PERIOD_MS` | `(1000u)` | 2、4、5、6、25：应用编译开关、初始化重试或后台/预热周期 |
| [code/config.h:47](../Seekfree_TC264_Opensource_Library/code/config.h#L47) | `CONTROL_ROLL_ENABLE` | `(1)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:48](../Seekfree_TC264_Opensource_Library/code/config.h#L48) | `CONTROL_PITCH_ENABLE` | `(1)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:49](../Seekfree_TC264_Opensource_Library/code/config.h#L49) | `CONTROL_MOTOR_ENABLE` | `(1)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:50](../Seekfree_TC264_Opensource_Library/code/config.h#L50) | `CONTROL_YAW_ENABLE` | `(0)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:51](../Seekfree_TC264_Opensource_Library/code/config.h#L51) | `CONTROL_PERIOD_MS` | `(1u)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:52](../Seekfree_TC264_Opensource_Library/code/config.h#L52) | `CONTROL_ANGLE_PERIOD_MS` | `(5u)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:53](../Seekfree_TC264_Opensource_Library/code/config.h#L53) | `CONTROL_SPEED_PERIOD_MS` | `(20u)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:54](../Seekfree_TC264_Opensource_Library/code/config.h#L54) | `CONTROL_FILTER_DT_S` | `(CONTROL_ANGLE_PERIOD_MS * 0.001f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:57](../Seekfree_TC264_Opensource_Library/code/config.h#L57) | `CONTROL_TARGET_SPEED_DEFAULT` | `(0.0f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:58](../Seekfree_TC264_Opensource_Library/code/config.h#L58) | `CONTROL_RAMP_REFERENCE` | `(160.0f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:59](../Seekfree_TC264_Opensource_Library/code/config.h#L59) | `CONTROL_RAMP_MS_DEFAULT` | `(5333.333333f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:60](../Seekfree_TC264_Opensource_Library/code/config.h#L60) | `CONTROL_DEADBAND_PX_DEFAULT` | `(0.0f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:61](../Seekfree_TC264_Opensource_Library/code/config.h#L61) | `CONTROL_K1_DEFAULT` | `(1.25f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:62](../Seekfree_TC264_Opensource_Library/code/config.h#L62) | `CONTROL_K2_DEFAULT` | `(0.5f)` | 2、10、11、12：控制分组开关、周期或初始目标/斜坡/视觉系数 |
| [code/config.h:63](../Seekfree_TC264_Opensource_Library/code/config.h#L63) | `MOTOR_PWM_FREQUENCY` | `(17000u)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:64](../Seekfree_TC264_Opensource_Library/code/config.h#L64) | `MOTOR_PWM_PIN` | `(ATOM0_CH3_P21_5)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:65](../Seekfree_TC264_Opensource_Library/code/config.h#L65) | `MOTOR_DIR_PIN` | `(P21_4)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:66](../Seekfree_TC264_Opensource_Library/code/config.h#L66) | `MOTOR_DIR_FORWARD_LEVEL` | `(0)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:67](../Seekfree_TC264_Opensource_Library/code/config.h#L67) | `MOTOR_DIR_REVERSE_LEVEL` | `(1)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:68](../Seekfree_TC264_Opensource_Library/code/config.h#L68) | `MOTOR_DEADZONE_DUTY` | `(400)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:69](../Seekfree_TC264_Opensource_Library/code/config.h#L69) | `MOTOR_DIRECTION_SIGN` | `(1)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:70](../Seekfree_TC264_Opensource_Library/code/config.h#L70) | `ENCODER_DIRECTION_SIGN` | `(1)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:71](../Seekfree_TC264_Opensource_Library/code/config.h#L71) | `MOTOR_ENCODER_INDEX` | `(TIM6_ENCODER)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:72](../Seekfree_TC264_Opensource_Library/code/config.h#L72) | `MOTOR_ENCODER_COUNT_PIN` | `(TIM6_ENCODER_CH1_P20_3)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:73](../Seekfree_TC264_Opensource_Library/code/config.h#L73) | `MOTOR_ENCODER_DIRECTION_PIN` | `(TIM6_ENCODER_CH2_P20_0)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:74](../Seekfree_TC264_Opensource_Library/code/config.h#L74) | `MOMENTUM_UART` | `(UART_3)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:75](../Seekfree_TC264_Opensource_Library/code/config.h#L75) | `MOMENTUM_UART_BAUD` | `(460800u)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:76](../Seekfree_TC264_Opensource_Library/code/config.h#L76) | `MOMENTUM_UART_TX` | `(UART3_TX_P15_6)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:77](../Seekfree_TC264_Opensource_Library/code/config.h#L77) | `MOMENTUM_UART_RX` | `(UART3_RX_P15_7)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:78](../Seekfree_TC264_Opensource_Library/code/config.h#L78) | `MOMENTUM_LEFT_SIGN` | `(1)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:79](../Seekfree_TC264_Opensource_Library/code/config.h#L79) | `MOMENTUM_RIGHT_SIGN` | `(-1)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:82](../Seekfree_TC264_Opensource_Library/code/config.h#L82) | `IMU_GYRO_BIAS_X` | `(7.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:83](../Seekfree_TC264_Opensource_Library/code/config.h#L83) | `IMU_GYRO_BIAS_Y` | `(-10.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:84](../Seekfree_TC264_Opensource_Library/code/config.h#L84) | `IMU_GYRO_BIAS_Z` | `(4.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:85](../Seekfree_TC264_Opensource_Library/code/config.h#L85) | `IMU_GYRO_DEADBAND_RAW` | `(5)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:86](../Seekfree_TC264_Opensource_Library/code/config.h#L86) | `IMU_LOWPASS_TIME_CONSTANT_S` | `(0.0079617834f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:87](../Seekfree_TC264_Opensource_Library/code/config.h#L87) | `IMU_SAMPLE_DT_S` | `(CONTROL_PERIOD_MS * 0.001f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:88](../Seekfree_TC264_Opensource_Library/code/config.h#L88) | `ROLL_GYRO_RAW` | `(icm_data_lpf.gyroY)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:89](../Seekfree_TC264_Opensource_Library/code/config.h#L89) | `ROLL_ACCEL_RAW` | `(icm_data_lpf.accZ)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:90](../Seekfree_TC264_Opensource_Library/code/config.h#L90) | `PITCH_GYRO_RAW` | `(icm_data_lpf.gyroZ)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:91](../Seekfree_TC264_Opensource_Library/code/config.h#L91) | `PITCH_ACCEL_RAW` | `(icm_data_lpf.accY)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:92](../Seekfree_TC264_Opensource_Library/code/config.h#L92) | `YAW_GYRO_RAW` | `(icm_data_lpf.gyroX)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:93](../Seekfree_TC264_Opensource_Library/code/config.h#L93) | `ROLL_FILTER_GYRO_WEIGHT` | `(4.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:94](../Seekfree_TC264_Opensource_Library/code/config.h#L94) | `ROLL_FILTER_ACCEL_WEIGHT` | `(6.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:95](../Seekfree_TC264_Opensource_Library/code/config.h#L95) | `PITCH_FILTER_GYRO_WEIGHT` | `(-6.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:96](../Seekfree_TC264_Opensource_Library/code/config.h#L96) | `PITCH_FILTER_ACCEL_WEIGHT` | `(4.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:97](../Seekfree_TC264_Opensource_Library/code/config.h#L97) | `ROLL_MECHANICAL_ZERO` | `(30.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:98](../Seekfree_TC264_Opensource_Library/code/config.h#L98) | `PITCH_MECHANICAL_ZERO` | `(-100.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:99](../Seekfree_TC264_Opensource_Library/code/config.h#L99) | `FILTER_STATE_LIMIT_RAW` | `(4000.0f)` | 8、9：传感器去偏/死区/低通、车体轴、互补权重和机械零点 |
| [code/config.h:102](../Seekfree_TC264_Opensource_Library/code/config.h#L102) | `PROTECT_ROLL_RAW` | `(1500.0f)` | 13：内部raw姿态或指令阈值，不是物理角度 |
| [code/config.h:103](../Seekfree_TC264_Opensource_Library/code/config.h#L103) | `PROTECT_PITCH_RAW` | `(2000.0f)` | 13：内部raw姿态或指令阈值，不是物理角度 |
| [code/config.h:104](../Seekfree_TC264_Opensource_Library/code/config.h#L104) | `PROTECT_ROLL_DUTY` | `(10000.0f)` | 13：内部raw姿态或指令阈值，不是物理角度 |
| [code/config.h:105](../Seekfree_TC264_Opensource_Library/code/config.h#L105) | `PROTECT_TRAVEL_DUTY` | `(8000.0f)` | 13：内部raw姿态或指令阈值，不是物理角度 |
| [code/config.h:106](../Seekfree_TC264_Opensource_Library/code/config.h#L106) | `MOMENTUM_DUTY_LIMIT` | `(10000)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/config.h:109](../Seekfree_TC264_Opensource_Library/code/config.h#L109) | `PID_ROLL_RATE_P` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:110](../Seekfree_TC264_Opensource_Library/code/config.h#L110) | `PID_ROLL_RATE_I` | `(0.13f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:111](../Seekfree_TC264_Opensource_Library/code/config.h#L111) | `PID_ROLL_RATE_D` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:112](../Seekfree_TC264_Opensource_Library/code/config.h#L112) | `PID_ROLL_RATE_I_LIMIT` | `(1000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:113](../Seekfree_TC264_Opensource_Library/code/config.h#L113) | `PID_ROLL_RATE_I_SCALE` | `(0.1f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:114](../Seekfree_TC264_Opensource_Library/code/config.h#L114) | `PID_ROLL_RATE_OUT_LIMIT` | `(10000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:115](../Seekfree_TC264_Opensource_Library/code/config.h#L115) | `PID_ROLL_ANGLE_P` | `(0.85f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:116](../Seekfree_TC264_Opensource_Library/code/config.h#L116) | `PID_ROLL_ANGLE_I` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:117](../Seekfree_TC264_Opensource_Library/code/config.h#L117) | `PID_ROLL_ANGLE_D` | `(0.08f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:118](../Seekfree_TC264_Opensource_Library/code/config.h#L118) | `PID_ROLL_ANGLE_I_LIMIT` | `(4000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:119](../Seekfree_TC264_Opensource_Library/code/config.h#L119) | `PID_ROLL_ANGLE_I_SCALE` | `(0.05f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:120](../Seekfree_TC264_Opensource_Library/code/config.h#L120) | `PID_ROLL_ANGLE_OUT_LIMIT` | `(8000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:121](../Seekfree_TC264_Opensource_Library/code/config.h#L121) | `PID_ROLL_SPEED_P` | `(0.2f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:122](../Seekfree_TC264_Opensource_Library/code/config.h#L122) | `PID_ROLL_SPEED_I` | `(0.001f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:123](../Seekfree_TC264_Opensource_Library/code/config.h#L123) | `PID_ROLL_SPEED_D` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:124](../Seekfree_TC264_Opensource_Library/code/config.h#L124) | `PID_ROLL_SPEED_I_LIMIT` | `(4000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:125](../Seekfree_TC264_Opensource_Library/code/config.h#L125) | `PID_ROLL_SPEED_I_SCALE` | `(0.5f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:126](../Seekfree_TC264_Opensource_Library/code/config.h#L126) | `PID_ROLL_SPEED_OUT_LIMIT` | `(1500.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:127](../Seekfree_TC264_Opensource_Library/code/config.h#L127) | `PID_PITCH_RATE_P` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:128](../Seekfree_TC264_Opensource_Library/code/config.h#L128) | `PID_PITCH_RATE_I` | `(0.023f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:129](../Seekfree_TC264_Opensource_Library/code/config.h#L129) | `PID_PITCH_RATE_D` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:130](../Seekfree_TC264_Opensource_Library/code/config.h#L130) | `PID_PITCH_RATE_I_LIMIT` | `(1000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:131](../Seekfree_TC264_Opensource_Library/code/config.h#L131) | `PID_PITCH_RATE_I_SCALE` | `(0.1f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:132](../Seekfree_TC264_Opensource_Library/code/config.h#L132) | `PID_PITCH_RATE_OUT_LIMIT` | `(10000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:133](../Seekfree_TC264_Opensource_Library/code/config.h#L133) | `PID_PITCH_ANGLE_P` | `(0.58f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:134](../Seekfree_TC264_Opensource_Library/code/config.h#L134) | `PID_PITCH_ANGLE_I` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:135](../Seekfree_TC264_Opensource_Library/code/config.h#L135) | `PID_PITCH_ANGLE_D` | `(0.003f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:136](../Seekfree_TC264_Opensource_Library/code/config.h#L136) | `PID_PITCH_ANGLE_I_LIMIT` | `(4000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:137](../Seekfree_TC264_Opensource_Library/code/config.h#L137) | `PID_PITCH_ANGLE_I_SCALE` | `(0.05f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:138](../Seekfree_TC264_Opensource_Library/code/config.h#L138) | `PID_PITCH_ANGLE_OUT_LIMIT` | `(8000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:139](../Seekfree_TC264_Opensource_Library/code/config.h#L139) | `PID_PITCH_SPEED_P` | `(5.8f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:140](../Seekfree_TC264_Opensource_Library/code/config.h#L140) | `PID_PITCH_SPEED_I` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:141](../Seekfree_TC264_Opensource_Library/code/config.h#L141) | `PID_PITCH_SPEED_D` | `(0.001f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:142](../Seekfree_TC264_Opensource_Library/code/config.h#L142) | `PID_PITCH_SPEED_I_LIMIT` | `(4000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:143](../Seekfree_TC264_Opensource_Library/code/config.h#L143) | `PID_PITCH_SPEED_I_SCALE` | `(0.5f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:144](../Seekfree_TC264_Opensource_Library/code/config.h#L144) | `PID_PITCH_SPEED_OUT_LIMIT` | `(3500.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:145](../Seekfree_TC264_Opensource_Library/code/config.h#L145) | `PID_YAW_RATE_P` | `(0.62f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:146](../Seekfree_TC264_Opensource_Library/code/config.h#L146) | `PID_YAW_RATE_I` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:147](../Seekfree_TC264_Opensource_Library/code/config.h#L147) | `PID_YAW_RATE_D` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:148](../Seekfree_TC264_Opensource_Library/code/config.h#L148) | `PID_YAW_RATE_I_LIMIT` | `(1000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:149](../Seekfree_TC264_Opensource_Library/code/config.h#L149) | `PID_YAW_RATE_I_SCALE` | `(0.1f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:150](../Seekfree_TC264_Opensource_Library/code/config.h#L150) | `PID_YAW_RATE_OUT_LIMIT` | `(10000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:151](../Seekfree_TC264_Opensource_Library/code/config.h#L151) | `PID_YAW_P` | `(9.4f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:152](../Seekfree_TC264_Opensource_Library/code/config.h#L152) | `PID_YAW_I` | `(1.55f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:153](../Seekfree_TC264_Opensource_Library/code/config.h#L153) | `PID_YAW_D` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:154](../Seekfree_TC264_Opensource_Library/code/config.h#L154) | `PID_YAW_I_LIMIT` | `(8500.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:155](../Seekfree_TC264_Opensource_Library/code/config.h#L155) | `PID_YAW_I_SCALE` | `(0.05f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:156](../Seekfree_TC264_Opensource_Library/code/config.h#L156) | `PID_YAW_OUT_LIMIT` | `(8000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:157](../Seekfree_TC264_Opensource_Library/code/config.h#L157) | `PID_MOTOR_P` | `(11.1f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:158](../Seekfree_TC264_Opensource_Library/code/config.h#L158) | `PID_MOTOR_I` | `(0.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:159](../Seekfree_TC264_Opensource_Library/code/config.h#L159) | `PID_MOTOR_D` | `(0.29f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:160](../Seekfree_TC264_Opensource_Library/code/config.h#L160) | `PID_MOTOR_I_LIMIT` | `(5500.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:161](../Seekfree_TC264_Opensource_Library/code/config.h#L161) | `PID_MOTOR_I_SCALE` | `(0.05f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:162](../Seekfree_TC264_Opensource_Library/code/config.h#L162) | `PID_MOTOR_OUT_LIMIT` | `(8000.0f)` | 11：相应PID的P/I/D、积分存储限幅/比例或输出限幅；增量环不使用I_LIMIT/I_SCALE |
| [code/config.h:165](../Seekfree_TC264_Opensource_Library/code/config.h#L165) | `TUNE_SPEED_LIMIT` | `(1000.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:166](../Seekfree_TC264_Opensource_Library/code/config.h#L166) | `TUNE_GAIN_LIMIT` | `(10000.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:167](../Seekfree_TC264_Opensource_Library/code/config.h#L167) | `TUNE_ZERO_LIMIT` | `(4000.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:168](../Seekfree_TC264_Opensource_Library/code/config.h#L168) | `TUNE_VISION_GAIN_LIMIT` | `(10.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:169](../Seekfree_TC264_Opensource_Library/code/config.h#L169) | `TUNE_RAMP_MIN_MS` | `(100.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:170](../Seekfree_TC264_Opensource_Library/code/config.h#L170) | `TUNE_RAMP_MAX_MS` | `(20000.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:171](../Seekfree_TC264_Opensource_Library/code/config.h#L171) | `TUNE_DEADBAND_MAX_PX` | `(10.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:172](../Seekfree_TC264_Opensource_Library/code/config.h#L172) | `TUNE_CRUISE_MAX_CMS` | `(200.0f)` | 20：运行期命令允许范围，不替代执行器保护 |
| [code/config.h:175](../Seekfree_TC264_Opensource_Library/code/config.h#L175) | `CAMERA_WIDTH` | `(188)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:176](../Seekfree_TC264_Opensource_Library/code/config.h#L176) | `CAMERA_HEIGHT` | `(120)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:177](../Seekfree_TC264_Opensource_Library/code/config.h#L177) | `CAMERA_THRESHOLD_MIN` | `(180)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:178](../Seekfree_TC264_Opensource_Library/code/config.h#L178) | `CAMERA_IR_THRESHOLD` | `(200)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:179](../Seekfree_TC264_Opensource_Library/code/config.h#L179) | `CAMERA_BLANK_TOP_ROWS` | `(30)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:180](../Seekfree_TC264_Opensource_Library/code/config.h#L180) | `CAMERA_SCAN_MIN_X` | `(10)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:181](../Seekfree_TC264_Opensource_Library/code/config.h#L181) | `CAMERA_SCAN_MIN_Y` | `(39)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:182](../Seekfree_TC264_Opensource_Library/code/config.h#L182) | `CAMERA_SCAN_BOTTOM_MARGIN` | `(10)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:183](../Seekfree_TC264_Opensource_Library/code/config.h#L183) | `CAMERA_WINDOW_SIZE` | `(10)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:184](../Seekfree_TC264_Opensource_Library/code/config.h#L184) | `CAMERA_CANDIDATE_SEPARATION` | `(20)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:185](../Seekfree_TC264_Opensource_Library/code/config.h#L185) | `CAMERA_CENTER_X` | `(CAMERA_WIDTH / 2)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:186](../Seekfree_TC264_Opensource_Library/code/config.h#L186) | `CAMERA_WEIGHT_X` | `(1.5f)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:187](../Seekfree_TC264_Opensource_Library/code/config.h#L187) | `CAMERA_WEIGHT_Y` | `(6.5f)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:188](../Seekfree_TC264_Opensource_Library/code/config.h#L188) | `CAMERA_WEIGHT_AREA` | `(2)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:189](../Seekfree_TC264_Opensource_Library/code/config.h#L189) | `CAMERA_LOST_ERROR` | `(65.0f)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:190](../Seekfree_TC264_Opensource_Library/code/config.h#L190) | `CAMERA_LOST_SWITCH_ERROR` | `(20.0f)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:191](../Seekfree_TC264_Opensource_Library/code/config.h#L191) | `CAMERA_OTSU_START` | `(5)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:192](../Seekfree_TC264_Opensource_Library/code/config.h#L192) | `CAMERA_OTSU_END` | `(245)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:193](../Seekfree_TC264_Opensource_Library/code/config.h#L193) | `CAMERA_AUTO_EXPOSURE` | `(0)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:194](../Seekfree_TC264_Opensource_Library/code/config.h#L194) | `CAMERA_EXPOSURE` | `(512)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:195](../Seekfree_TC264_Opensource_Library/code/config.h#L195) | `CAMERA_FPS` | `(50)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:196](../Seekfree_TC264_Opensource_Library/code/config.h#L196) | `CAMERA_GAIN` | `(32)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:197](../Seekfree_TC264_Opensource_Library/code/config.h#L197) | `DISPLAY_INTERFACE` | `(IPS200_TYPE_SPI)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:198](../Seekfree_TC264_Opensource_Library/code/config.h#L198) | `DISPLAY_IMAGE_DIVISOR` | `(2)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:199](../Seekfree_TC264_Opensource_Library/code/config.h#L199) | `DISPLAY_MARKER_SIZE` | `(5)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:200](../Seekfree_TC264_Opensource_Library/code/config.h#L200) | `DISPLAY_ERROR_X` | `(0)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:201](../Seekfree_TC264_Opensource_Library/code/config.h#L201) | `DISPLAY_ERROR_Y` | `(60)` | 21、22：相机/窗口/检测/显示配置；当前路径关闭 |
| [code/config.h:205](../Seekfree_TC264_Opensource_Library/code/config.h#L205) | `WIFI_LINK_CM_PER_COUNT` | `(0.0f)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:207](../Seekfree_TC264_Opensource_Library/code/config.h#L207) | `WIFI_LINK_IMG_W` | `(CAMERA_WIDTH)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:208](../Seekfree_TC264_Opensource_Library/code/config.h#L208) | `WIFI_LINK_IMG_H` | `(CAMERA_HEIGHT)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:209](../Seekfree_TC264_Opensource_Library/code/config.h#L209) | `WIFI_LINK_IMG_SIZE` | `(WIFI_LINK_IMG_W * WIFI_LINK_IMG_H)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:210](../Seekfree_TC264_Opensource_Library/code/config.h#L210) | `WIFI_LINK_DATA_PERIOD_MS` | `(50u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:211](../Seekfree_TC264_Opensource_Library/code/config.h#L211) | `WIFI_LINK_LOG_PERIOD_MS` | `(20u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:212](../Seekfree_TC264_Opensource_Library/code/config.h#L212) | `WIFI_LINK_IMAGE_PERIOD_MS` | `(66u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:213](../Seekfree_TC264_Opensource_Library/code/config.h#L213) | `WIFI_LINK_RETRY_MS` | `(5000u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:214](../Seekfree_TC264_Opensource_Library/code/config.h#L214) | `WIFI_LINK_BOOT_DELAY_MS` | `(300u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:215](../Seekfree_TC264_Opensource_Library/code/config.h#L215) | `WIFI_LINK_RX_PERIOD_MS` | `(5u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:216](../Seekfree_TC264_Opensource_Library/code/config.h#L216) | `WIFI_LINK_IO_TIMEOUT_MS` | `(250u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:217](../Seekfree_TC264_Opensource_Library/code/config.h#L217) | `WIFI_LINK_CPU0_DELAY_US` | `(300u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:218](../Seekfree_TC264_Opensource_Library/code/config.h#L218) | `WIFI_LINK_FRAME_TIMEOUT_MS` | `(300u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:219](../Seekfree_TC264_Opensource_Library/code/config.h#L219) | `WIFI_LINK_SOCKET_RETRY_LIMIT` | `(6u)` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/config.h:221](../Seekfree_TC264_Opensource_Library/code/config.h#L221) | `HW_KEY_LIST` | `{P20_6, P20_7, P11_2, P11_3}` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:224](../Seekfree_TC264_Opensource_Library/code/config.h#L224) | `HW_IMU660RB_SPI_SPEED` | `(10 * 1000 * 1000)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:225](../Seekfree_TC264_Opensource_Library/code/config.h#L225) | `HW_IMU660RB_SPI` | `(SPI_0)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:226](../Seekfree_TC264_Opensource_Library/code/config.h#L226) | `HW_IMU660RB_SPC_PIN` | `(SPI0_SCLK_P20_11 )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:227](../Seekfree_TC264_Opensource_Library/code/config.h#L227) | `HW_IMU660RB_SDI_PIN` | `(SPI0_MOSI_P20_14)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:228](../Seekfree_TC264_Opensource_Library/code/config.h#L228) | `HW_IMU660RB_SDO_PIN` | `(SPI0_MISO_P20_12)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:229](../Seekfree_TC264_Opensource_Library/code/config.h#L229) | `HW_IMU660RB_CS_PIN` | `(P20_13)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:230](../Seekfree_TC264_Opensource_Library/code/config.h#L230) | `HW_IMU660RB_ACC_SAMPLE` | `(0x3C)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:231](../Seekfree_TC264_Opensource_Library/code/config.h#L231) | `HW_IMU660RB_GYR_SAMPLE` | `(0x5C)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:232](../Seekfree_TC264_Opensource_Library/code/config.h#L232) | `HW_IPS200_SPI_SPEED` | `(60*1000*1000)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:233](../Seekfree_TC264_Opensource_Library/code/config.h#L233) | `HW_IPS200_SPI` | `(SPI_2           )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:234](../Seekfree_TC264_Opensource_Library/code/config.h#L234) | `HW_IPS200_SCL_PIN_SPI` | `(SPI2_SCLK_P15_3 )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:235](../Seekfree_TC264_Opensource_Library/code/config.h#L235) | `HW_IPS200_SDA_PIN_SPI` | `(SPI2_MOSI_P15_5 )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:236](../Seekfree_TC264_Opensource_Library/code/config.h#L236) | `HW_IPS200_SDA_IN_PIN_SPI` | `(SPI2_MISO_P15_4 )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:237](../Seekfree_TC264_Opensource_Library/code/config.h#L237) | `HW_IPS200_RST_PIN_SPI` | `(P15_1)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:238](../Seekfree_TC264_Opensource_Library/code/config.h#L238) | `HW_IPS200_DC_PIN_SPI` | `(P15_0)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:239](../Seekfree_TC264_Opensource_Library/code/config.h#L239) | `HW_IPS200_CS_PIN_SPI` | `(P15_2)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:240](../Seekfree_TC264_Opensource_Library/code/config.h#L240) | `HW_IPS200_BLk_PIN_SPI` | `(P15_4)` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:241](../Seekfree_TC264_Opensource_Library/code/config.h#L241) | `HW_WIFI_SPI_INDEX` | `(SPI_3             )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:242](../Seekfree_TC264_Opensource_Library/code/config.h#L242) | `HW_WIFI_SPI_SPEED` | `(30 * 1000 * 1000  )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:243](../Seekfree_TC264_Opensource_Library/code/config.h#L243) | `HW_WIFI_SPI_SCK_PIN` | `(SPI3_SCLK_P22_3   )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:244](../Seekfree_TC264_Opensource_Library/code/config.h#L244) | `HW_WIFI_SPI_MOSI_PIN` | `(SPI3_MOSI_P22_0   )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:245](../Seekfree_TC264_Opensource_Library/code/config.h#L245) | `HW_WIFI_SPI_MISO_PIN` | `(SPI3_MISO_P22_1   )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:246](../Seekfree_TC264_Opensource_Library/code/config.h#L246) | `HW_WIFI_SPI_CS_PIN` | `(P22_2             )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:247](../Seekfree_TC264_Opensource_Library/code/config.h#L247) | `HW_WIFI_SPI_INT_PIN` | `(P15_8             )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/config.h:248](../Seekfree_TC264_Opensource_Library/code/config.h#L248) | `HW_WIFI_SPI_RST_PIN` | `(P23_1             )` | 8、15、22：外设引脚/总线/寄存器配置；HW_KEY_LIST见6.1 |
| [code/motor.h:9](../Seekfree_TC264_Opensource_Library/code/motor.h#L9) | `CODE_MOTOR_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/motor.h:12](../Seekfree_TC264_Opensource_Library/code/motor.h#L12) | `MAX_DUTY` | `(50 )` | 13：旧名映射或遗留宏；MAX_DUTY未用于实际总限幅 |
| [code/motor.h:13](../Seekfree_TC264_Opensource_Library/code/motor.h#L13) | `DIR1` | `(MOTOR_DIR_PIN)` | 13：旧名映射或遗留宏；MAX_DUTY未用于实际总限幅 |
| [code/motor.h:14](../Seekfree_TC264_Opensource_Library/code/motor.h#L14) | `PWM1` | `(MOTOR_PWM_PIN)` | 13：旧名映射或遗留宏；MAX_DUTY未用于实际总限幅 |
| [code/motor.h:16](../Seekfree_TC264_Opensource_Library/code/motor.h#L16) | `ENCODER_1` | `(MOTOR_ENCODER_INDEX)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/motor.h:17](../Seekfree_TC264_Opensource_Library/code/motor.h#L17) | `ENCODER_1_A` | `(MOTOR_ENCODER_COUNT_PIN)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/motor.h:18](../Seekfree_TC264_Opensource_Library/code/motor.h#L18) | `ENCODER_1_B` | `(MOTOR_ENCODER_DIRECTION_PIN)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/picture_process.h:9](../Seekfree_TC264_Opensource_Library/code/picture_process.h#L9) | `CODE_PICTURE_PROCESS_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/posture_control.h:9](../Seekfree_TC264_Opensource_Library/code/posture_control.h#L9) | `CODE_POSTURE_CONTROL_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/safety_output.h:2](../Seekfree_TC264_Opensource_Library/code/safety_output.h#L2) | `ONEWHEEL_SAFETY_OUTPUT_H` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/small_driver_uart_control.h:9](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.h#L9) | `SMALL_DRIVER_UART_CONTROL_H_` | `（无值）` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/small_driver_uart_control.h:14](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.h#L14) | `SMALL_DRIVER_UART` | `(MOMENTUM_UART)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/small_driver_uart_control.h:16](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.h#L16) | `SMALL_DRIVER_BAUDRATE` | `(MOMENTUM_UART_BAUD)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/small_driver_uart_control.h:18](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.h#L18) | `SMALL_DRIVER_RX` | `(MOMENTUM_UART_TX)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/small_driver_uart_control.h:20](../Seekfree_TC264_Opensource_Library/code/small_driver_uart_control.h#L20) | `SMALL_DRIVER_TX` | `(MOMENTUM_UART_RX)` | 12、13、14：执行器/编码器/串口与方向、死区配置 |
| [code/user_value.h:9](../Seekfree_TC264_Opensource_Library/code/user_value.h#L9) | `CODE_USER_VALUE_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/user_value.h:13](../Seekfree_TC264_Opensource_Library/code/user_value.h#L13) | `FLASH_SECTION_INDEX` | `(0)` | 23：未调用Flash读取接口的页/扇区 |
| [code/user_value.h:14](../Seekfree_TC264_Opensource_Library/code/user_value.h#L14) | `FLASH_PAGE_INDEX` | `(8)` | 23：未调用Flash读取接口的页/扇区 |
| [code/wifi_link.c:30](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L30) | `WL_MUX_HEADER_SIZE` | `(8)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:31](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L31) | `WL_DATA_SIZE` | `(228)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:32](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L32) | `WL_CAM_HEADER_SIZE` | `(24)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:33](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L33) | `WL_CAM_CHUNK_SIZE` | `(1400)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:34](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L34) | `WL_CAM_CHUNK_COUNT` | `((WIFI_LINK_IMG_SIZE + WL_CAM_CHUNK_SIZE - 1) / WL_CAM_CHUNK_SIZE)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:35](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L35) | `WL_PACKET_SIZE` | `(WL_MUX_HEADER_SIZE + WL_CAM_HEADER_SIZE + WL_CAM_CHUNK_SIZE)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:36](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L36) | `WL_LOG_CHUNK_SIZE` | `(900)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:37](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L37) | `WL_CMD_SIZE` | `(64)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:39](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L39) | `WL_MUX_TYPE_DATA` | `(1)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:40](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L40) | `WL_MUX_TYPE_LOG` | `(2)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:41](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L41) | `WL_MUX_TYPE_CAMERA` | `(3)` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.c:583](../Seekfree_TC264_Opensource_Library/code/wifi_link.c#L583) | `WL_PARAM_COUNT` | `(sizeof(wl_params) / sizeof(wl_params[0]))` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link.h:21](../Seekfree_TC264_Opensource_Library/code/wifi_link.h#L21) | `CODE_WIFI_LINK_H_` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/wifi_link_config.example.h:2](../Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h#L2) | `ONEWHEEL_WIFI_CONFIG_H` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [code/wifi_link_config.example.h:7](../Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h#L7) | `WIFI_LINK_SSID` | `"YOUR_WIFI_SSID"` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/wifi_link_config.example.h:8](../Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h#L8) | `WIFI_LINK_PASSWORD` | `"YOUR_WIFI_PASSWORD"` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/wifi_link_config.example.h:9](../Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h#L9) | `WIFI_LINK_HOST_IP` | `"192.168.1.100"` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/wifi_link_config.example.h:10](../Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h#L10) | `WIFI_LINK_HOST_PORT` | `"8081"` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/wifi_link_config.example.h:11](../Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h#L11) | `WIFI_LINK_LOCAL_PORT` | `"6666"` | 15—21：通信配置、周期、超时、帧尺寸或距离换算；example.h值仅为公开模板 |
| [code/wifi_link_io.c:9](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.c#L9) | `IO_TIMEOUT_MS` | `WIFI_LINK_IO_TIMEOUT_MS` | 16—21：私有协议容量/类型/数量/IO超时常量 |
| [code/wifi_link_io.h:2](../Seekfree_TC264_Opensource_Library/code/wifi_link_io.h#L2) | `ONEWHEEL_WIFI_LINK_IO_H` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [user/cpu0_main.h:49](../Seekfree_TC264_Opensource_Library/user/cpu0_main.h#L49) | `CPU0_MAIN_H` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [user/isr.h:37](../Seekfree_TC264_Opensource_Library/user/isr.h#L37) | `_isr_h` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [user/isr_config.h:37](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L37) | `_isr_config_h` | `（无值）` | 头文件防重复包含标记；不形成独立运行功能 |
| [user/isr_config.h:56](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L56) | `CCU6_0_CH0_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:57](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L57) | `CCU6_0_CH0_ISR_PRIORITY` | `30` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:59](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L59) | `CCU6_0_CH1_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:60](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L60) | `CCU6_0_CH1_ISR_PRIORITY` | `31` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:62](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L62) | `CCU6_1_CH0_INT_SERVICE` | `IfxSrc_Tos_cpu1` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:63](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L63) | `CCU6_1_CH0_ISR_PRIORITY` | `32` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:65](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L65) | `CCU6_1_CH1_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:66](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L66) | `CCU6_1_CH1_ISR_PRIORITY` | `33` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:72](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L72) | `EXTI_CH0_CH4_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:73](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L73) | `EXTI_CH0_CH4_INT_PRIO` | `40` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:76](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L76) | `EXTI_CH1_CH5_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:77](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L77) | `EXTI_CH1_CH5_INT_PRIO` | `41` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:80](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L80) | `EXTI_CH2_CH6_INT_SERVICE` | `IfxSrc_Tos_dma` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:81](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L81) | `EXTI_CH2_CH6_INT_PRIO` | `5` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:84](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L84) | `EXTI_CH3_CH7_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:85](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L85) | `EXTI_CH3_CH7_INT_PRIO` | `43` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:89](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L89) | `DMA_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:90](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L90) | `DMA_INT_PRIO` | `60` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:94](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L94) | `UART0_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:95](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L95) | `UART0_TX_INT_PRIO` | `11` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:96](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L96) | `UART0_RX_INT_PRIO` | `10` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:97](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L97) | `UART0_ER_INT_PRIO` | `12` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:99](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L99) | `UART1_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:100](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L100) | `UART1_TX_INT_PRIO` | `13` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:101](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L101) | `UART1_RX_INT_PRIO` | `14` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:102](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L102) | `UART1_ER_INT_PRIO` | `15` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:104](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L104) | `UART2_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:105](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L105) | `UART2_TX_INT_PRIO` | `16` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:106](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L106) | `UART2_RX_INT_PRIO` | `17` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:107](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L107) | `UART2_ER_INT_PRIO` | `18` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:109](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L109) | `UART3_INT_SERVICE` | `IfxSrc_Tos_cpu0` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:110](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L110) | `UART3_TX_INT_PRIO` | `19` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:111](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L111) | `UART3_RX_INT_PRIO` | `20` | 24：中断服务核或优先级，存在定义不等于已使能 |
| [user/isr_config.h:112](../Seekfree_TC264_Opensource_Library/user/isr_config.h#L112) | `UART3_ER_INT_PRIO` | `21` | 24：中断服务核或优先级，存在定义不等于已使能 |

## 私有网络配置与文件覆盖

`wifi_link_config.h` 没有复制进审查材料；实际网络宏含 SSID、PASSWORD、HOST_IP、HOST_PORT、LOCAL_PORT，作用见主讲解第15节。公开 example.h 不是当前联网凭据。

| 文件 | 本轮原始SHA256 | 实际函数数 |
| --- | --- | --- |
| code/app.c | `aafd7b972c69b654db24eb6cf5b72b46c11f26dd3242cad2c47f071134cbcc8b` | 11 |
| code/app.h | `3c7340fb506c7eaec34413752b2c1e5be6eb05a16031243fae72f4f3bf2745b6` | 0 |
| code/balance_control.c | `ff57243858ece86f384924ed5574f19602199868508a592f381e1dfc0b502d9e` | 6 |
| code/balance_control.h | `149cc5bc19a48061878472b29120532f409bde56be49f44d486bc6d596321227` | 0 |
| code/camera.c | `6f906fc331846b1a1416c01d9b498f67792d530f8992f2b7c479a479a0b46bee` | 7 |
| code/camera.h | `017cf4492846b204841ecbc4fa4c7d8cae2a767e227e831995d164ea5b87c71d` | 0 |
| code/config.h | `655372e9bff3bbe9f94c005bb1cc149548b464f55acca35675f9474cd46c2342` | 0 |
| code/four_posture.c | `28104bcaf124db779d3f7f77ea8c527591ef45da1e1850377a759eca11f97718` | 0 |
| code/four_posture.h | `be6245a47a488e19c345001cdc7848a64b6969bec80d529cd458c21b8a925fa3` | 0 |
| code/motor.c | `9a98b5d49b1379b6a8dd241cb7910b09719f85f7a7f8d027d3ea2fdc28253855` | 4 |
| code/motor.h | `ef9863fc9fa68369625b87bf9d916a8667612b51524013554dacc8278114e777` | 0 |
| code/picture_process.c | `90cd9f45030f39333db43dc7c585e7d7ae1465f4f0beb88c425c06a6c6368f64` | 0 |
| code/picture_process.h | `df65b76754586f78b3fb12f56b146258d94354e0216e25d245870e5feec389b4` | 0 |
| code/posture_control.c | `6197aa021893477709f28c8bac669e5e5629f61d68666aa6a259955ccc30ae4b` | 11 |
| code/posture_control.h | `9902f1f812b40f94e153d965acb0f7132d455e748abd4d18352b148372400fa7` | 0 |
| code/safety_output.c | `9150e2c813606dadb9712b908d855ac1263abf49155794ebfaf89f65ec8a2e31` | 11 |
| code/safety_output.h | `70716afe24903489348727762cf9b671b87c08743a7407e60db436d62e135780` | 0 |
| code/small_driver_uart_control.c | `a25f8bbb30a587e6d3ce241fac45d981ebc6dd713687e57953e48fbe31d0cfe7` | 10 |
| code/small_driver_uart_control.h | `66a81b082539a0ec8ec88d3b9ca4da26d3dfc998273f7a1b5cf609c7aa76568d` | 0 |
| code/user_value.c | `de0dd4f62bad41611c6ecf6e7e5f73b6081f05b864f9b9f179d9b53fafb35ad4` | 4 |
| code/user_value.h | `736272530e5529a53fa35851f16e0aaaacae2799768ab1e1f8fefc8f70f96318` | 0 |
| code/wifi_link.c | `bc434b1424a613349b3479958aae495e504912ff026a293d8d86194d5d98548c` | 34 |
| code/wifi_link.h | `116614a3ca3448e7c53987e9557ee60a49ae0a7ab9bf787c3a8c522413242f85` | 0 |
| code/wifi_link_config.example.h | `b230f5c438f2454396b596afb32115ff020e04486b1b4fdba6588c8f83fef111` | 0 |
| code/wifi_link_io.c | `932ce1d4abfd29491cf6e2e423be11966e52243cdfd5768f24b7c1a61e7facd9` | 8 |
| code/wifi_link_io.h | `99741616633af3e9da080d08c9abad8876d3cc2f9e4a9d28aaf5af18149064fb` | 0 |
| user/cpu0_main.c | `deb4ae9f2ebd476c39010eb691c282fc7c90593e42085f9ab6854bce84b2e6d6` | 1 |
| user/cpu0_main.h | `fa3364c113d4733bf1d769c6396b42a9b148304f75a77b374517b74e83a2d767` | 0 |
| user/cpu1_main.c | `88fd0b5317e6223434b7d2f187154165b7f3ed3320da75e80dac7b1082a1590c` | 1 |
| user/isr.c | `94e931aba6978baf5e1a73ce137850924d2d3cc6ea9d7bb701ec25bfbe9584a7` | 20 |
| user/isr.h | `d2d1e7ce3182f5a6895a0ce5ea0756e2a9490388352a136fb3942710e0f95503` | 0 |
| user/isr_config.h | `7362fc63e6894d5090484d83f558e165cd5a864f55c5be99963113da2546ff54` | 0 |
