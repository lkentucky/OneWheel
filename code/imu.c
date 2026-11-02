/* ASC A+ r10h port, preserves Qcar IMU wiring and public telemetry API.
 * Raw physical input -> r10h; startup-bias-only gyro -> control. No EKF/LPF
 * between gyro and controller. Fixed 1ms updates, 6000 stationary + 3000 warmup.
 * Reference frame R=(Qcar Y,-Qcar X,Qcar Z), a proper right-handed rotation.
 * Public legacy angles/quaternion remain in the original Qcar body frame.
 * See ASC_PORT.md for units, fault recovery and hardware verification limits. */
#include "imu.h"
#include "attitude.h"
#include "config.h"
#include "IfxStm.h"
#include "attitude_algorithm.h"
#include "zf_common_interrupt.h"
#include "zf_device_imu660rb.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "r10h/include/r10h_attitude.h"
#include <math.h>
#include <string.h>
#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
#define IMU_STATUS_REG 0x1eu
#define IMU_CTRL8_REG 0x17u
static float g_stm_hz;
volatile int32 imu_attitude_status=IMU_ATTITUDE_INIT_FAILED;
volatile float imu_roll_deg, imu_pitch_deg, imu_yaw_deg;
volatile float imu_accel_norm_g, imu_calibration_percent;
volatile float imu_accel_g[3], imu_gyro_dps[3];
volatile float imu_gyro_bias_z_dps;
volatile float imu_sample_dt_ms, imu_solve_us, imu_solve_max_us;
volatile int32 imu_accel_used=-1;
volatile float imu_ekf_nis=-1.0f, imu_ekf_bias_xy_dps[2];
volatile uint32 imu_sample_sequence;
volatile uint32 imu_spi_timeouts,imu_not_ready_samples,imu_last_status_reg,imu_bad_bursts;
volatile int32 imu_quality,imu_reject_reason,imu_bias_learning;
volatile float imu_reject_ms,imu_accept_ratio,imu_accel_vibration_g;
volatile float imu_gyro_step_dps,imu_r_scale,imu_bias_slew_dps2,imu_recovery_percent;
volatile uint32 imu_reject_count,imu_gyro_clips,imu_accel_clips,imu_recovery_count;
volatile float imu_pre_gyro_dps[3],imu_pre_accel_g[3],imu_pre_norm_g;
volatile int32 imu_reacquiring;
volatile float imu_gyro_z_integral_deg;
const uint8 imu_attitude_method=5u; /* A+ r10h, distinct from legacy EKF=3. */
volatile float imu_asc_roll_deg,imu_asc_pitch_deg,imu_asc_yaw_deg;
static float g_bias[3],g_sum_g[3],g_sum_a[3],g_min_g[3],g_max_g[3],g_min_a[3],g_max_a[3];
static unsigned g_cal_count,g_warmup,g_identity_count;
static uint32 g_last_sample_ticks;
static uint8 g_have_sample,g_calibrated,g_core_ready,g_sample_gyro_clip;
static volatile uint8 g_update_enabled,g_recalibration_requested;
static imu_navigation_sample_t g_navigation_sample;
static uint32 g_navigation_generation;
#include "imu_spi.inc"
static uint32 imu_ticks(void){return IfxStm_getLower(&MODULE_STM0);}
static uint32_t imu_r10_clock(void){return (uint32_t)imu_ticks();}
static float imu_seconds(uint32 elapsed){return (float)elapsed/g_stm_hz;}
static void imu_reg_write(uint8 reg,uint8 value)
{
 uint8 tx[2]={reg|IMU660RB_SPI_W,value};
 IMU660RB_CS(0);if(spi_transfer_8bit_timeout(IMU660RB_SPI,tx,0,2,1))++imu_spi_timeouts;IMU660RB_CS(1);system_delay_ms(1);
}
static uint8 imu_reg_read(uint8 reg)
{
 uint8 value=0xff;if(!r10h_bus_read(reg,&value,1)){++imu_spi_timeouts;return 0xff;}return value;
}
static uint8 imu_read_sample(float gyro_dps[3],float accel_g[3])
{
    uint8 data[12],i;
    float sensor_g[3],sensor_a[3];
    uint8 status=imu_reg_read(IMU_STATUS_REG);
    uint8 accel_clip=0;
    g_sample_gyro_clip=0;
    /* 单次读到 0xFF/无 ready 位多为总线毛刺，同一 tick 内重试一次。 */
    /* No retries in the active sample path. */
    imu_last_status_reg=status;
    if (status==0xFFu || (status&0x03u)!=0x03u) {++imu_not_ready_samples;return 0u;}
    if(!r10h_bus_read(IMU660RB_OUTX_L_G,data,12u)){++imu_spi_timeouts;return 0u;}
    /* 12 字节完全相同（全 0x00 或全 0xFF）是总线掉线，不能当样本交给 EKF。 */
    for (i=1u;i<12u;++i) if (data[i]!=data[0]) break;
    if (i==12u) {++imu_bad_bursts;return 0u;}
    imu660rb_gyro_x=(int16)((uint16)data[0]|((uint16)data[1]<<8));
    imu660rb_gyro_y=(int16)((uint16)data[2]|((uint16)data[3]<<8));
    imu660rb_gyro_z=(int16)((uint16)data[4]|((uint16)data[5]<<8));
    imu660rb_acc_x=(int16)((uint16)data[6]|((uint16)data[7]<<8));
    imu660rb_acc_y=(int16)((uint16)data[8]|((uint16)data[9]<<8));
    imu660rb_acc_z=(int16)((uint16)data[10]|((uint16)data[11]<<8));
    for(i=0u;i<3u;++i) {
        int16 raw_g=(int16)((uint16)data[2u*i]|((uint16)data[2u*i+1u]<<8));
        int16 raw_a=(int16)((uint16)data[6u+2u*i]|((uint16)data[7u+2u*i]<<8));
        if(raw_g>=IMU_RAW_CLIP_COUNTS || raw_g<=-IMU_RAW_CLIP_COUNTS) g_sample_gyro_clip=1;
        if(raw_a>=IMU_RAW_CLIP_COUNTS || raw_a<=-IMU_RAW_CLIP_COUNTS) accel_clip=1;
        sensor_g[i]=(float)raw_g/14.3f;
        sensor_a[i]=(float)raw_a/4098.0f;
    }
    gyro_dps[0]=sensor_g[IMU_BODY_X_AXIS]*IMU_BODY_X_SIGN;
    gyro_dps[1]=sensor_g[IMU_BODY_Y_AXIS]*IMU_BODY_Y_SIGN;
    gyro_dps[2]=sensor_g[IMU_BODY_Z_AXIS]*IMU_BODY_Z_SIGN;
    accel_g[0]=sensor_a[IMU_BODY_X_AXIS]*IMU_BODY_X_SIGN;
    accel_g[1]=sensor_a[IMU_BODY_Y_AXIS]*IMU_BODY_Y_SIGN;
    accel_g[2]=sensor_a[IMU_BODY_Z_AXIS]*IMU_BODY_Z_SIGN;
    if(g_sample_gyro_clip) ++imu_gyro_clips;
    if(accel_clip) ++imu_accel_clips;
    if(g_sample_gyro_clip || accel_clip) return 0u;
    return 1u;
}
static uint8 imu_hardware_init(void)
{
    if (imu660rb_init()!=0u) return 0u;
    /* 1667Hz physical ODR, 1kHz consumer; preserve board wiring and filters. */
    imu_reg_write(IMU660RB_CTRL3_C,IMU_CTRL3_C_CONFIG);
    imu_reg_write(IMU660RB_CTRL4_C,IMU_CTRL4_C_CONFIG);
    imu_reg_write(IMU660RB_CTRL6_C,IMU_CTRL6_C_CONFIG);
    imu_reg_write(IMU660RB_CTRL7_G,IMU_CTRL7_G_CONFIG);
    imu_reg_write(IMU_CTRL8_REG,IMU_CTRL8_XL_CONFIG);
    imu_reg_write(IMU660RB_CTRL1_XL,IMU_CTRL1_XL_CONFIG);
    imu_reg_write(IMU660RB_CTRL2_G,IMU_CTRL2_G_CONFIG);
    return (imu_reg_read(IMU660RB_WHO_AM_I)==0x6bu && imu_reg_read(IMU660RB_CTRL1_XL)==IMU_CTRL1_XL_CONFIG &&
            imu_reg_read(IMU660RB_CTRL2_G)==IMU_CTRL2_G_CONFIG &&
            imu_reg_read(IMU660RB_CTRL3_C)==IMU_CTRL3_C_CONFIG &&
            imu_reg_read(IMU660RB_CTRL4_C)==IMU_CTRL4_C_CONFIG &&
            imu_reg_read(IMU660RB_CTRL6_C)==IMU_CTRL6_C_CONFIG &&
            imu_reg_read(IMU660RB_CTRL7_G)==IMU_CTRL7_G_CONFIG &&
            imu_reg_read(IMU_CTRL8_REG)==IMU_CTRL8_XL_CONFIG)?1u:0u;
}

static void calibration_clear(void)
{
 g_cal_count=0;memset(g_sum_g,0,sizeof(g_sum_g));memset(g_sum_a,0,sizeof(g_sum_a));
}
static void reference_sample(ahrs_sample_t *s,const float g[3],const float a[3])
{ s->gx=g[1];s->gy=-g[0];s->gz=g[2];s->ax=a[1];s->ay=-a[0];s->az=a[2]; }
static int calibration_sample(const float g[3],const float a[3])
{
 unsigned i;float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
 if(norm<.75f||norm>1.25f){calibration_clear();return 0;}
 for(i=0;i<3;++i){
  if(fabsf(g[i])>CAL_GYRO_LIMIT_DPS){calibration_clear();return 0;}
  if(!g_cal_count)g_min_g[i]=g_max_g[i]=g[i],g_min_a[i]=g_max_a[i]=a[i];
  if(g[i]<g_min_g[i])g_min_g[i]=g[i];
  if(g[i]>g_max_g[i])g_max_g[i]=g[i];
  if(a[i]<g_min_a[i])g_min_a[i]=a[i];
  if(a[i]>g_max_a[i])g_max_a[i]=a[i];
  if(g_max_g[i]-g_min_g[i]>CAL_GYRO_SPAN_DPS||g_max_a[i]-g_min_a[i]>CAL_ACCEL_SPAN_G){calibration_clear();return 0;}
 }
 for(i=0;i<3;++i){g_sum_g[i]+=g[i];g_sum_a[i]+=a[i];}
 return ++g_cal_count>=CALIBRATION_SAMPLES;
}
static void publish(const ahrs_output_t *o,const float gyro[3],const float accel[3])
{
 float hr=o->roll*.00872664625997f,hp=o->pitch*.00872664625997f,hy=o->yaw*.00872664625997f;
 float cr=cosf(hr),sr=sinf(hr),cp=cosf(hp),sp=sinf(hp),cy=cosf(hy),sy=sinf(hy);
 float qr[4]={cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
 float qb[4]={qr[0],-qr[2],qr[1],qr[3]},e[3],g[3];unsigned i;
 attitude_euler(qb,e);imu_roll_deg=e[0];imu_pitch_deg=e[1];imu_yaw_deg=e[2];
 imu_asc_roll_deg=o->roll;imu_asc_pitch_deg=o->pitch;imu_asc_yaw_deg=o->yaw;
 for(i=0;i<3;++i){g[i]=gyro[i]-g_bias[i];g_navigation_sample.gyro_dps[i]=g[i];g_navigation_sample.accel_g[i]=accel[i];imu_pre_gyro_dps[i]=g[i];imu_pre_accel_g[i]=accel[i];}
 attitude_publish_state(qb,g,accel);
 g_navigation_sample.roll_deg=imu_roll_deg;g_navigation_sample.pitch_deg=imu_pitch_deg;g_navigation_sample.yaw_deg=imu_yaw_deg;
 g_navigation_sample.dt_s=.001f;g_navigation_sample.generation=g_navigation_generation;
 if(++g_navigation_sample.sequence==0u)++g_navigation_sample.sequence;
 imu_sample_sequence=g_navigation_sample.sequence;
 imu_accel_used=o->acc_weight>0;imu_quality=imu_accel_used?0:1;
 imu_accept_ratio=o->acc_weight;imu_r_scale=1;imu_pre_norm_g=imu_accel_norm_g;
 imu_gyro_bias_z_dps=g_bias[2];imu_gyro_z_integral_deg+=g[2]*.001f;
}
bool imu_get_navigation_sample(imu_navigation_sample_t *out)
{
 if(!out||imu_attitude_status!=IMU_ATTITUDE_RUNNING||!g_navigation_sample.sequence)return false;
 *out=g_navigation_sample;return true;
}
static void imu_reset_pipeline(void)
{
 ++g_navigation_generation;memset(&g_navigation_sample,0,sizeof(g_navigation_sample));
 memset(g_bias,0,sizeof(g_bias));calibration_clear();g_warmup=g_identity_count=0;
 g_have_sample=g_calibrated=g_core_ready=0;imu_sample_sequence=0;
 imu_roll_deg=imu_pitch_deg=imu_yaw_deg=0;imu_asc_roll_deg=imu_asc_pitch_deg=imu_asc_yaw_deg=0;
 imu_calibration_percent=imu_sample_dt_ms=imu_solve_us=imu_solve_max_us=0;imu_accel_used=-1;
 imu_ekf_nis=-1;imu_ekf_bias_xy_dps[0]=imu_ekf_bias_xy_dps[1]=0;imu_reacquiring=0;
 imu_quality=imu_reject_reason=imu_bias_learning=0;imu_reject_ms=0;imu_accept_ratio=imu_r_scale=1;
 imu_recovery_percent=0.0f;imu_recovery_count=imu_reject_count=0;
 imu_accel_vibration_g=imu_gyro_step_dps=imu_bias_slew_dps2=0;imu_pre_norm_g=0;
 imu_gyro_z_integral_deg=imu_gyro_bias_z_dps=0;
 attitude_reset_state();g_last_sample_ticks=imu_ticks();
}
void imu_init(void)
{
 g_update_enabled=0;g_recalibration_requested=0;g_stm_hz=IfxStm_getFrequency(&MODULE_STM0);
#ifndef R10H_HOST_TEST
 r10h_bus_timeout_ticks=(uint32)(g_stm_hz*.0001f);
#endif
 imu_reset_pipeline();
 if(g_stm_hz<=0||!isfinite(g_stm_hz)||!imu_hardware_init()){imu_attitude_status=IMU_ATTITUDE_INIT_FAILED;return;}
 imu_attitude_status=IMU_ATTITUDE_CALIBRATING;g_update_enabled=1;
}
void imu_update(void)
{
 float g[3],a[3],mg[3],ma[3];uint32 now,started;unsigned i;ahrs_sample_t sample;ahrs_output_t output;
 if(!g_update_enabled||imu_attitude_status<0)return;
 now=imu_ticks();imu_sample_dt_ms=imu_seconds((uint32)(now-g_last_sample_ticks))*1000;
 g_last_sample_ticks=now;
 if(g_have_sample&&(imu_sample_dt_ms<.75f||imu_sample_dt_ms>1.25f)){imu_attitude_status=IMU_ATTITUDE_BAD_DT;return;}
 g_have_sample=1;
 if(!imu_read_sample(g,a)){imu_attitude_status=IMU_ATTITUDE_TIMEOUT;return;}
 if((g_identity_count++%100u)==0u&&imu_reg_read(IMU660RB_WHO_AM_I)!=0x6bu){imu_attitude_status=IMU_ATTITUDE_TIMEOUT;return;}
 for(i=0;i<3;++i){imu_gyro_dps[i]=g[i];imu_accel_g[i]=a[i];}
 imu_accel_norm_g=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
 if(!g_core_ready){
  if(calibration_sample(g,a)){
   for(i=0;i<3;++i){mg[i]=g_sum_g[i]/(float)CALIBRATION_SAMPLES;ma[i]=g_sum_a[i]/(float)CALIBRATION_SAMPLES;g_bias[i]=mg[i];}
   reference_sample(&sample,mg,ma);
   if(!r10h_init(&sample)){imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR;return;}
   r10h_set_clock(imu_r10_clock,1000000.0f/g_stm_hz);g_core_ready=1;
  }
  imu_calibration_percent=100.0f*(float)g_cal_count/(float)CALIBRATION_SAMPLES;return;
 }
 reference_sample(&sample,g,a);started=imu_ticks();
 if(!r10h_update(&sample)||!r10h_get_output(&output)||output.fault||!isfinite(output.roll)||!isfinite(output.pitch)||!isfinite(output.yaw)){
  imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR;return;
 }
 imu_solve_us=imu_seconds((uint32)(imu_ticks()-started))*1000000;
 if(imu_solve_us>imu_solve_max_us)imu_solve_max_us=imu_solve_us;
 if(imu_solve_us>=1000){imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR;return;}
 if(g_warmup<ASC_ATT_READY_SAMPLES)++g_warmup;
 if(g_warmup>=ASC_ATT_READY_SAMPLES){g_calibrated=1;imu_attitude_status=IMU_ATTITUDE_RUNNING;}
 publish(&output,g,a);
}
void imu_request_recalibration(void){g_recalibration_requested=1;}
void imu_service(void)
{
 uint32 irq;if(!g_recalibration_requested)return;
 irq=interrupt_global_disable();g_update_enabled=0;g_recalibration_requested=0;interrupt_global_enable(irq);
 if(!imu_hardware_init()){imu_attitude_status=IMU_ATTITUDE_INIT_FAILED;return;}
 irq=interrupt_global_disable();imu_reset_pipeline();imu_attitude_status=IMU_ATTITUDE_CALIBRATING;g_update_enabled=1;interrupt_global_enable(irq);
}
void imu_prepare_start(void)
{
 /* Healthy Start retains calibration/core, but never resumes a faulted core.
  * The IMU timer restarts with a new timing origin; stale publication is invalid. */
 uint32 irq=interrupt_global_disable();
 if(g_calibrated&&imu_attitude_status==IMU_ATTITUDE_RUNNING&&!g_recalibration_requested){
  g_have_sample=0;g_last_sample_ticks=imu_ticks();imu_attitude_status=IMU_ATTITUDE_CALIBRATING;
 }else g_recalibration_requested=1;
 interrupt_global_enable(irq);imu_service();
}
void imu_recover_after_stall(void){imu_request_recalibration();}
#if defined(__TASKING__)
#pragma section all restore
#endif
