#ifndef MCAR_IMU_CONFIG_H
#define MCAR_IMU_CONFIG_H

/* Six-axis AHRS selection, ported from rt1064_imu_vofa. */
#define AHRS_METHOD_MAHONY       1
#define AHRS_METHOD_MADGWICK     2
#define AHRS_METHOD_EKF          3
#ifndef AHRS_METHOD
#define AHRS_METHOD              AHRS_METHOD_EKF
#endif
#if AHRS_METHOD != AHRS_METHOD_MAHONY && AHRS_METHOD != AHRS_METHOD_MADGWICK && AHRS_METHOD != AHRS_METHOD_EKF
#error Invalid AHRS_METHOD
#endif

#ifndef MADGWICK_BETA
#define MADGWICK_BETA            0.05f
#endif

/* EKF noise densities (rad^2/s, rad^2/s^3), normalized gravity variance.
 * Gate uses normalized innovation squared, 3 DOF, 99 percent threshold.
 * No small-gyro deadband: preserve slow real yaw motion. */
#define EKF_GYRO_NOISE           0.0001f
#define EKF_BIAS_NOISE           0.00000001f
#define EKF_ACCEL_VARIANCE       0.0025f
#define EKF_CHI_SQUARE_LIMIT     11.345f
#define EKF_ACCEL_LPF_TAU_S      0.01f
#define EKF_BIAS_LIMIT_RADPS     0.08726646f

#define AHRS_KP                  2.0f
#define AHRS_KI                  0.02f
#define AHRS_INTEGRAL_LIMIT      0.08726646f

#define ACC_NORM_MIN             0.85f
#define ACC_NORM_MAX             1.15f
#define MAX_SAMPLE_DT            0.020f

#define CALIBRATION_SAMPLES      400u
#define CAL_GYRO_LIMIT_DPS       10.0f
#define CAL_GYRO_SPAN_DPS        1.5f
#define CAL_ACCEL_SPAN_G         0.08f
#define IMU_TIMEOUT_MS           100u

/* 深度静止时的陀螺 z 零偏自动跟踪（温漂补偿）：六轴 yaw 无绝对基准，
 * 上电 2s 标定的残余零偏会持续积分成航向漂移。融合层确认深度静止
 * （轮速/陀螺/加速度三门限连续 MIN_STILL 秒）后，把 z 零偏以时间常数
 * TAU 一阶拉向当前原始读数，偏离标定基准不得超过 LIMIT。 */
#define IMU_BIAS_TRACK_MIN_STILL_S 1.0f
#define IMU_BIAS_TRACK_TAU_S       5.0f
#define IMU_BIAS_TRACK_LIMIT_DPS   1.0f

/* Observed mounting: side tilt was old pitch, fore/aft was old roll.
 * Body X/Y/Z = sensor Z/-Y/X. New X=old Y, new Y=-old X, new Z=old Z.
 * Both accel and gyro use this right-handed mapping. Motor sign is separate. */
#define IMU_BODY_X_AXIS 2
#define IMU_BODY_Y_AXIS 1
#define IMU_BODY_Z_AXIS 0
#define IMU_BODY_X_SIGN 1.0f
#define IMU_BODY_Y_SIGN -1.0f
#define IMU_BODY_Z_SIGN 1.0f
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.29577951308232088f
#endif
