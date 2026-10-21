/* Qcar compatibility adapter. Algorithm and calibration ported from
 * mcar-mailun/MCARpreparation, GPL-3.0-or-later; see THIRD_PARTY.md. */
#include "attitude_algorithm.h"
#include <string.h>
struct MAHONY_FILTER_t mahony_filter;
static float last_yaw;
static uint8 have_yaw;
void attitude_reset_legacy(void)
{
    memset(&mahony_filter,0,sizeof(mahony_filter));
    mahony_filter.q0=1.0f;
    last_yaw=0; have_yaw=0;
}
void attitude_publish_legacy(const float q[4],const float g[3],const float a[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3],delta;
    mahony_filter.q0=w; mahony_filter.q1=x; mahony_filter.q2=y; mahony_filter.q3=z;
    mahony_filter.gyro.x=g[0]; mahony_filter.gyro.y=g[1]; mahony_filter.gyro.z=g[2];
    mahony_filter.acc.x=a[0]*9.80665f; mahony_filter.acc.y=a[1]*9.80665f; mahony_filter.acc.z=a[2]*9.80665f;
    mahony_filter.rMat[0][0]=1-2*(y*y+z*z); mahony_filter.rMat[0][1]=2*(x*y-w*z); mahony_filter.rMat[0][2]=2*(x*z+w*y);
    mahony_filter.rMat[1][0]=2*(x*y+w*z); mahony_filter.rMat[1][1]=1-2*(x*x+z*z); mahony_filter.rMat[1][2]=2*(y*z-w*x);
    mahony_filter.rMat[2][0]=2*(x*z-w*y); mahony_filter.rMat[2][1]=2*(y*z+w*x); mahony_filter.rMat[2][2]=1-2*(x*x+y*y);
    mahony_filter.roll=imu_roll_deg; mahony_filter.pitch=imu_pitch_deg; mahony_filter.yaw=imu_yaw_deg;
    if(have_yaw) {
        delta=imu_yaw_deg-last_yaw;
        if(delta>180) delta-=360;
        if(delta< -180) delta+=360;
        mahony_filter.True_yaw+=delta;
    } else { mahony_filter.True_yaw=imu_yaw_deg; have_yaw=1; }
    last_yaw=imu_yaw_deg;
}
