/* OneWheel 全局配置中心：应用开关、标定、PID、引脚和默认值。
 * 上位机修改的是运行时变量；重启恢复本文件默认值。原始量不是角度。
 * WiFi 账号密码保留在被 Git 忽略的 wifi_link_config.h 中。
 */
#ifndef ONEWHEEL_CONFIG_H
#define ONEWHEEL_CONFIG_H

/* 1. 直立调试模式。摄像头调用在 app_poll() 中保留为注释。 */
#ifndef APP_DRIVE_ENABLE
#define APP_DRIVE_ENABLE                 (1)
#endif
#ifndef APP_CAMERA_ENABLE
#define APP_CAMERA_ENABLE                (0)
#endif
#ifndef APP_CAMERA_DISPLAY_ENABLE
#define APP_CAMERA_DISPLAY_ENABLE        (0)
#endif
#ifndef APP_IPS200_ENABLE
#define APP_IPS200_ENABLE                (0)
#endif
#ifndef APP_WIFI_ENABLE
#define APP_WIFI_ENABLE                  (1)
#endif
#ifndef APP_WIFI_IMAGE_ENABLE
#define APP_WIFI_IMAGE_ENABLE            (0)
#endif
#define APP_KEYS_ENABLE                  (1)
#define APP_WIRELESS_UART_ENABLE         (0)
#define APP_CSV_DEBUG_ENABLE             (0)
#define APP_MAIN_LOOP_DELAY_MS           (10u)
#define APP_KEY_SCAN_PERIOD_MS           (10u)
#define APP_TELEMETRY_PERIOD_MS          (10u)
#define APP_IMU_INIT_ATTEMPTS            (3u)
#define APP_IMU_RETRY_DELAY_MS           (50u)
#define APP_STATUS_LOG_PERIOD_MS        (1000u)
#define APP_CONTROL_WARMUP_MS            (2000u)
#define SAFETY_OUTPUT_PERIOD_MS          (1u)
#define CONTROL_HEARTBEAT_TIMEOUT_MS     (10u)
#define MOMENTUM_FEEDBACK_TIMEOUT_MS     (100u)
#define MOTOR_UART_TX_TIMEOUT_MS         (5u)
#define MOTOR_DIR_SETUP_US               (100u)
/* UART0 boot/network diagnostics; does not change the UDP protocol. */
#define APP_BOOT_DIAGNOSTICS_ENABLE       (1)
#define APP_BOOT_DIAGNOSTICS_PERIOD_MS    (1000u)

/* 2. 可分别停用某组执行器/控制环；关闭后该组输出为零。 */
#define CONTROL_ROLL_ENABLE              (1)
#define CONTROL_PITCH_ENABLE             (1)
#define CONTROL_MOTOR_ENABLE             (1)
#define CONTROL_YAW_ENABLE               (0)
#define CONTROL_PERIOD_MS                (1u)
#define CONTROL_ANGLE_PERIOD_MS          (5u)
#define CONTROL_SPEED_PERIOD_MS          (20u)
#define CONTROL_FILTER_DT_S              (CONTROL_ANGLE_PERIOD_MS * 0.001f)

/* 3. 速度与方向。计数/20ms；直立调试不主动前进。 */
#define CONTROL_TARGET_SPEED_DEFAULT     (0.0f)
#define CONTROL_RAMP_REFERENCE           (160.0f)
#define CONTROL_RAMP_MS_DEFAULT          (5333.333333f)
#define CONTROL_DEADBAND_PX_DEFAULT      (0.0f)
#define CONTROL_K1_DEFAULT               (1.25f)
#define CONTROL_K2_DEFAULT               (0.5f) /* 旧函数保留，当前路径不使用 k2 */
#define MOTOR_PWM_FREQUENCY              (17000u)
#define MOTOR_PWM_PIN                    (ATOM0_CH3_P21_5)
#define MOTOR_DIR_PIN                    (P21_4)
#define MOTOR_DIR_FORWARD_LEVEL          (0)
#define MOTOR_DIR_REVERSE_LEVEL          (1)
#define MOTOR_DEADZONE_DUTY              (400)
#define MOTOR_DIRECTION_SIGN             (1)
#define ENCODER_DIRECTION_SIGN           (1)
#define MOTOR_ENCODER_INDEX              (TIM2_ENCODER)
#define MOTOR_ENCODER_COUNT_PIN          (TIM2_ENCODER_CH1_P33_7)
#define MOTOR_ENCODER_DIRECTION_PIN      (TIM2_ENCODER_CH2_P33_6)
#define MOMENTUM_UART                    (UART_3)
#define MOMENTUM_UART_BAUD               (460800u)
#define MOMENTUM_UART_TX                 (UART3_TX_P15_6)
#define MOMENTUM_UART_RX                 (UART3_RX_P15_7)
#define MOMENTUM_LEFT_SIGN               (1)
#define MOMENTUM_RIGHT_SIGN              (-1)

/* 4. IMU 标定。零偏由原始读数减去；符号需按车体安装实测。 */
#define IMU_GYRO_BIAS_X                  (7.0f)
#define IMU_GYRO_BIAS_Y                  (-10.0f)
#define IMU_GYRO_BIAS_Z                  (4.0f)
#define IMU_GYRO_DEADBAND_RAW            (5)
#define IMU_LOWPASS_TIME_CONSTANT_S      (0.0079617834f)
#define IMU_SAMPLE_DT_S                  (CONTROL_PERIOD_MS * 0.001f)
#define ROLL_GYRO_RAW                    (icm_data_lpf.gyroY)
#define ROLL_ACCEL_RAW                   (icm_data_lpf.accZ)
#define PITCH_GYRO_RAW                   (icm_data_lpf.gyroZ)
#define PITCH_ACCEL_RAW                  (icm_data_lpf.accY)
#define YAW_GYRO_RAW                     (icm_data_lpf.gyroX)
#define ROLL_FILTER_GYRO_WEIGHT          (4.0f)
#define ROLL_FILTER_ACCEL_WEIGHT         (6.0f)
#define PITCH_FILTER_GYRO_WEIGHT         (-6.0f)
#define PITCH_FILTER_ACCEL_WEIGHT        (4.0f)
#define ROLL_MECHANICAL_ZERO             (30.0f)
#define PITCH_MECHANICAL_ZERO            (-100.0f)
#define FILTER_STATE_LIMIT_RAW          (4000.0f)

/* 5. 输出/倾倒保护，均为原控制器内部尺度，非度数。 */
#define PROTECT_ROLL_RAW                 (1500.0f)
#define PROTECT_PITCH_RAW                (2000.0f)
#define PROTECT_ROLL_DUTY                (10000.0f)
#define PROTECT_TRAVEL_DUTY              (8000.0f)
#define MOMENTUM_DUTY_LIMIT              (10000)

/* 6. PID 默认值。P/I/D、积分限幅、积分步长系数、输出限幅。 */
#define PID_ROLL_RATE_P                (0.0f)
#define PID_ROLL_RATE_I                (0.13f)
#define PID_ROLL_RATE_D                (0.0f)
#define PID_ROLL_RATE_I_LIMIT          (1000.0f)
#define PID_ROLL_RATE_I_SCALE          (0.1f)
#define PID_ROLL_RATE_OUT_LIMIT        (10000.0f)
#define PID_ROLL_ANGLE_P                (0.85f)
#define PID_ROLL_ANGLE_I                (0.0f)
#define PID_ROLL_ANGLE_D                (0.08f)
#define PID_ROLL_ANGLE_I_LIMIT          (4000.0f)
#define PID_ROLL_ANGLE_I_SCALE          (0.05f)
#define PID_ROLL_ANGLE_OUT_LIMIT        (8000.0f)
#define PID_ROLL_SPEED_P                (0.2f)
#define PID_ROLL_SPEED_I                (0.001f)
#define PID_ROLL_SPEED_D                (0.0f)
#define PID_ROLL_SPEED_I_LIMIT          (4000.0f)
#define PID_ROLL_SPEED_I_SCALE          (0.5f)
#define PID_ROLL_SPEED_OUT_LIMIT        (1500.0f)
#define PID_PITCH_RATE_P                (0.0f)
#define PID_PITCH_RATE_I                (0.023f)
#define PID_PITCH_RATE_D                (0.0f)
#define PID_PITCH_RATE_I_LIMIT          (1000.0f)
#define PID_PITCH_RATE_I_SCALE          (0.1f)
#define PID_PITCH_RATE_OUT_LIMIT        (10000.0f)
#define PID_PITCH_ANGLE_P                (0.58f)
#define PID_PITCH_ANGLE_I                (0.0f)
#define PID_PITCH_ANGLE_D                (0.003f)
#define PID_PITCH_ANGLE_I_LIMIT          (4000.0f)
#define PID_PITCH_ANGLE_I_SCALE          (0.05f)
#define PID_PITCH_ANGLE_OUT_LIMIT        (8000.0f)
#define PID_PITCH_SPEED_P                (5.8f)
#define PID_PITCH_SPEED_I                (0.0f)
#define PID_PITCH_SPEED_D                (0.001f)
#define PID_PITCH_SPEED_I_LIMIT          (4000.0f)
#define PID_PITCH_SPEED_I_SCALE          (0.5f)
#define PID_PITCH_SPEED_OUT_LIMIT        (3500.0f)
#define PID_YAW_RATE_P                (0.62f)
#define PID_YAW_RATE_I                (0.0f)
#define PID_YAW_RATE_D                (0.0f)
#define PID_YAW_RATE_I_LIMIT          (1000.0f)
#define PID_YAW_RATE_I_SCALE          (0.1f)
#define PID_YAW_RATE_OUT_LIMIT        (10000.0f)
#define PID_YAW_P                (9.4f)
#define PID_YAW_I                (1.55f)
#define PID_YAW_D                (0.0f)
#define PID_YAW_I_LIMIT          (8500.0f)
#define PID_YAW_I_SCALE          (0.05f)
#define PID_YAW_OUT_LIMIT        (8000.0f)
#define PID_MOTOR_P                (11.1f)
#define PID_MOTOR_I                (0.0f)
#define PID_MOTOR_D                (0.29f)
#define PID_MOTOR_I_LIMIT          (5500.0f)
#define PID_MOTOR_I_SCALE          (0.05f)
#define PID_MOTOR_OUT_LIMIT        (8000.0f)

/* 调参界限：固件校验和独轮上位机同时读取这些定义。 */
#define TUNE_SPEED_LIMIT                (1000.0f)
#define TUNE_GAIN_LIMIT                 (10000.0f)
#define TUNE_ZERO_LIMIT                 (4000.0f)
#define TUNE_VISION_GAIN_LIMIT          (10.0f)
#define TUNE_RAMP_MIN_MS                (100.0f)
#define TUNE_RAMP_MAX_MS                (20000.0f)
#define TUNE_DEADBAND_MAX_PX            (10.0f)
#define TUNE_CRUISE_MAX_CMS             (200.0f)

/* 7. 视觉模块预留，当前不初始化/不处理摄像头。 */
#define CAMERA_WIDTH                    (188)
#define CAMERA_HEIGHT                   (120)
#define CAMERA_THRESHOLD_MIN            (180)
#define CAMERA_IR_THRESHOLD             (200)
#define CAMERA_BLANK_TOP_ROWS            (30)
#define CAMERA_SCAN_MIN_X               (10)
#define CAMERA_SCAN_MIN_Y               (39)
#define CAMERA_SCAN_BOTTOM_MARGIN       (10)
#define CAMERA_WINDOW_SIZE              (10)
#define CAMERA_CANDIDATE_SEPARATION      (20)
#define CAMERA_CENTER_X                 (CAMERA_WIDTH / 2)
#define CAMERA_WEIGHT_X                 (1.5f)
#define CAMERA_WEIGHT_Y                 (6.5f)
#define CAMERA_WEIGHT_AREA              (2)
#define CAMERA_LOST_ERROR               (65.0f)
#define CAMERA_LOST_SWITCH_ERROR        (20.0f)
#define CAMERA_OTSU_START               (5)
#define CAMERA_OTSU_END                 (245)
#define CAMERA_AUTO_EXPOSURE            (0)
#define CAMERA_EXPOSURE                 (512)
#define CAMERA_FPS                      (50)
#define CAMERA_GAIN                     (32)
#define DISPLAY_INTERFACE               (IPS200_TYPE_SPI)
#define DISPLAY_IMAGE_DIVISOR           (2)
#define DISPLAY_MARKER_SIZE             (5)
#define DISPLAY_ERROR_X                 (0)
#define DISPLAY_ERROR_Y                 (60)

/* 8. 无线周期与编码器距离标定。0 表示尚未标定物理距离。 */
#ifndef WIFI_LINK_CM_PER_COUNT
#define WIFI_LINK_CM_PER_COUNT           (0.0f)
#endif
#define WIFI_LINK_IMG_W                  (CAMERA_WIDTH)
#define WIFI_LINK_IMG_H                  (CAMERA_HEIGHT)
#define WIFI_LINK_IMG_SIZE               (WIFI_LINK_IMG_W * WIFI_LINK_IMG_H)
#define WIFI_LINK_DATA_PERIOD_MS         (50u)
#define WIFI_LINK_LOG_PERIOD_MS          (20u)
#define WIFI_LINK_IMAGE_PERIOD_MS        (66u)
#define WIFI_LINK_RETRY_MS               (5000u)
#define WIFI_LINK_BOOT_DELAY_MS          (300u)
#define WIFI_LINK_RX_PERIOD_MS           (5u)
#define WIFI_LINK_IO_TIMEOUT_MS          (250u)
#define WIFI_LINK_CPU0_DELAY_US          (300u)
#define WIFI_LINK_FRAME_TIMEOUT_MS       (300u)
#define WIFI_LINK_SOCKET_RETRY_LIMIT     (6u)

#define HW_KEY_LIST {P20_6, P20_7, P11_2, P11_3}

/* 9. 常用外设映射。未列出的库协议常量不属于应用调参项。 */
#define HW_IMU660RB_SPI_SPEED             (10 * 1000 * 1000)
#define HW_IMU660RB_SPI                   (SPI_0)
#define HW_IMU660RB_SPC_PIN               (SPI0_SCLK_P20_11 )
#define HW_IMU660RB_SDI_PIN               (SPI0_MOSI_P20_14)
#define HW_IMU660RB_SDO_PIN               (SPI0_MISO_P20_12)
#define HW_IMU660RB_CS_PIN                (P20_13)
#define HW_IMU660RB_ACC_SAMPLE            (0x3C)
#define HW_IMU660RB_GYR_SAMPLE            (0x5C)
#define HW_IPS200_SPI_SPEED               (60*1000*1000)
#define HW_IPS200_SPI                     (SPI_2           )
#define HW_IPS200_SCL_PIN_SPI             (SPI2_SCLK_P15_3 )
#define HW_IPS200_SDA_PIN_SPI             (SPI2_MOSI_P15_5 )
#define HW_IPS200_SDA_IN_PIN_SPI          (SPI2_MISO_P15_4 )
#define HW_IPS200_RST_PIN_SPI             (P15_1)
#define HW_IPS200_DC_PIN_SPI              (P15_0)
#define HW_IPS200_CS_PIN_SPI              (P15_2)
#define HW_IPS200_BLk_PIN_SPI             (P15_4)
#define HW_WIFI_SPI_INDEX                 (SPI_3             )
#define HW_WIFI_SPI_SPEED                 (30 * 1000 * 1000  )
#define HW_WIFI_SPI_SCK_PIN               (SPI3_SCLK_P22_3   )
#define HW_WIFI_SPI_MOSI_PIN              (SPI3_MOSI_P22_0   )
#define HW_WIFI_SPI_MISO_PIN              (SPI3_MISO_P22_1   )
#define HW_WIFI_SPI_CS_PIN                (P22_2             )
#define HW_WIFI_SPI_INT_PIN               (P15_8             )
#define HW_WIFI_SPI_RST_PIN               (P23_1             )

#if MOTOR_DIRECTION_SIGN != 1 && MOTOR_DIRECTION_SIGN != -1
#error "Motor direction sign must be +1 or -1."
#endif
#if (MOTOR_DIR_FORWARD_LEVEL != 0 && MOTOR_DIR_FORWARD_LEVEL != 1) || (MOTOR_DIR_REVERSE_LEVEL != 0 && MOTOR_DIR_REVERSE_LEVEL != 1) || MOTOR_DIR_FORWARD_LEVEL == MOTOR_DIR_REVERSE_LEVEL
#error "PWM/DIR requires opposite low/high direction levels."
#endif
#if APP_IMU_INIT_ATTEMPTS < 1 || APP_IMU_INIT_ATTEMPTS > 255
#error "IMU init attempts must be between 1 and 255."
#endif
#if APP_KEY_SCAN_PERIOD_MS < 1 || APP_TELEMETRY_PERIOD_MS < 1 || APP_STATUS_LOG_PERIOD_MS < 1 || APP_MAIN_LOOP_DELAY_MS < 1
#error "Background periods must be positive."
#endif
#if SAFETY_OUTPUT_PERIOD_MS != 1 || CONTROL_HEARTBEAT_TIMEOUT_MS <= SAFETY_OUTPUT_PERIOD_MS || MOMENTUM_FEEDBACK_TIMEOUT_MS < CONTROL_SPEED_PERIOD_MS || MOTOR_UART_TX_TIMEOUT_MS < 1 || MOTOR_DIR_SETUP_US < 1
#error "Invalid independent output/feedback timing."
#endif
#if MOTOR_DIRECTION_SIGN != 1 && MOTOR_DIRECTION_SIGN != -1
#error "Travel direction must be +1/-1."
#endif
#if (MOMENTUM_LEFT_SIGN != 1 && MOMENTUM_LEFT_SIGN != -1) || (MOMENTUM_RIGHT_SIGN != 1 && MOMENTUM_RIGHT_SIGN != -1)
#error "Momentum directions must be +1/-1."
#endif
#if CONTROL_PERIOD_MS != 1
#error "This controller is tuned for a 1 ms base tick."
#endif
#if CONTROL_ANGLE_PERIOD_MS % CONTROL_PERIOD_MS || CONTROL_SPEED_PERIOD_MS % CONTROL_PERIOD_MS
#error "Control periods must be multiples of the base tick."
#endif
#if APP_CAMERA_DISPLAY_ENABLE && (!APP_CAMERA_ENABLE || !APP_IPS200_ENABLE)
#error "Camera display requires both camera and IPS200."
#endif
#if APP_WIFI_IMAGE_ENABLE && (!APP_CAMERA_ENABLE || !APP_WIFI_ENABLE)
#error "Image upload requires camera and WiFi."
#endif
#if CAMERA_WIDTH > 255 || CAMERA_HEIGHT > 255
#error "Current vision loops require image dimensions <= 255."
#endif

#endif
