/* Compatibility view for Qcar control, backed by imu.c / Ahrs6. */
#ifndef _ATTITUDE_ALGORITHM_H_
#define _ATTITUDE_ALGORITHM_H_
#include "zf_common_headfile.h"
#include "imu.h"
#define RAD2DEG 57.29577951308232088f
#define DEG2RAD 0.01745329251994329577f
struct MAHONY_FILTER_t {
    /* Historical name only; default solver is EKF.
     * gyro: bias-corrected body deg/s, acc: body m/s^2; angles: degrees. */
    Axis3f gyro, acc;
    float q0,q1,q2,q3,rMat[3][3];
    float pitch,roll,yaw,True_yaw;
};
extern struct MAHONY_FILTER_t mahony_filter;
void attitude_reset_legacy(void);
void attitude_publish_legacy(const float q[4],const float gyro_dps[3],const float accel_g[3]);
#endif
