/* ASC 0eed51b balance controller port: no planner/vision dependencies.
 * Retains positional Roll/Yaw, incremental Pitch, antiwindup and Roll priority.
 * Qcar adaptations: explicit frame/polarity boundary; reject nonfinite inputs;
 * reset incremental Pitch history when its gains change. See ASC_PORT.md. */
#include "asc_control.h"
#include <math.h>
#include <string.h>
static void gains(asc_pid_t *s,float p,float i,float d,float imax,int angle)
{
 s->kp=p;s->ki=angle?i*.2f:i;s->kd=angle?d*5.0f:d;
 s->imax=angle?imax*5.0f:imax;
}
void asc_control_reset(asc_controller_t *s){memset(s,0,sizeof(*s));}
void asc_reference_zero(float legacy_roll,float legacy_pitch,float *roll,float *pitch)
{
 float r=legacy_roll*.017453292519943f,p=legacy_pitch*.017453292519943f;
 float x=sinf(r)*cosf(p),y=sinf(p),z=cosf(r)*cosf(p);
 *roll=atan2f(y,z)*57.29577951308232f;
 *pitch=atan2f(-x,sqrtf(y*y+z*z))*57.29577951308232f;
}
static int valid(const asc_config_t *p,const asc_input_t *a)
{
 const float config[]={p->roll_zero,p->pitch_zero,p->recovery_kp,p->recovery_ki,p->recovery_kd,
 p->roll_angle_kp,p->roll_angle_ki,p->roll_angle_kd,p->roll_rate_kp,p->roll_rate_ki,p->roll_rate_kd,
 p->pitch_speed_kp,p->pitch_speed_ki,p->pitch_speed_kd,p->pitch_angle_kp,p->pitch_angle_ki,p->pitch_angle_kd,
 p->pitch_rate_kp,p->pitch_rate_ki,p->pitch_rate_kd,p->yaw_angle_kp,p->yaw_angle_ki,p->yaw_angle_kd,
 p->yaw_rate_kp,p->yaw_rate_ki,p->yaw_rate_kd,p->fly_limit,p->ground_limit,p->roll_target_limit,p->pitch_tilt_limit};
 const float input[]={a->roll,a->pitch,a->yaw,a->roll_rate,a->pitch_rate,a->yaw_rate,a->left_rpm,a->right_rpm,a->ground_counts_20ms};
 unsigned i;for(i=0;i<sizeof(config)/sizeof(config[0]);++i)if(!isfinite(config[i]))return 0;
 for(i=0;i<sizeof(input)/sizeof(input[0]);++i)if(!isfinite(input[i]))return 0;
 return p->fly_limit>=0&&p->fly_limit<=10000&&p->ground_limit>=0&&p->ground_limit<=8000&&
 p->roll_target_limit>=0&&p->pitch_tilt_limit>=0&&
 (p->ground_sign==1||p->ground_sign==-1)&&(p->roll_sign==1||p->roll_sign==-1);
}
int asc_control_update(asc_controller_t *s,const asc_config_t *p,const asc_input_t *a,asc_output_t *o)
{
 int run20;float rt,pt,pitch_dot,headroom,roll,yaw;
 if(!valid(p,a)){asc_control_reset(s);*o=s->output;return 0;}
 if(!s->initialized){s->yaw_target=a->yaw;s->initialized=1;}
 if(s->have_config&&(p->pitch_rate_kp!=s->last_config.pitch_rate_kp||
 p->pitch_rate_ki!=s->last_config.pitch_rate_ki||p->pitch_rate_kd!=s->last_config.pitch_rate_kd))asc_pid_reset(&s->pitch_rate);
 s->last_config=*p;s->have_config=1;
 run20=(s->ticks++%20u)==0;
 gains(&s->recovery,p->recovery_kp,p->recovery_ki,p->recovery_kd,50,0);
 gains(&s->roll_angle,p->roll_angle_kp,p->roll_angle_ki,p->roll_angle_kd,2000,1);
 gains(&s->roll_rate,p->roll_rate_kp,p->roll_rate_ki,p->roll_rate_kd,100,0);
 gains(&s->pitch_speed,p->pitch_speed_kp,p->pitch_speed_ki,p->pitch_speed_kd,200,0);
 gains(&s->pitch_angle,p->pitch_angle_kp,p->pitch_angle_ki,p->pitch_angle_kd,50,1);
 gains(&s->pitch_rate,p->pitch_rate_kp,p->pitch_rate_ki,p->pitch_rate_kd,100,0);
 gains(&s->yaw_angle,p->yaw_angle_kp,p->yaw_angle_ki,p->yaw_angle_kd,50,1);
 gains(&s->yaw_rate,p->yaw_rate_kp,p->yaw_rate_ki,p->yaw_rate_kd,100,0);
 if(run20){
  asc_pid_loc_calc(&s->recovery,a->left_rpm-a->right_rpm);
  if(a->speed_fresh)asc_pid_loc_calc_limited(&s->pitch_speed,a->ground_counts_20ms,-p->pitch_tilt_limit,p->pitch_tilt_limit);
 }
 rt=p->roll_zero+asc_clamp(-s->recovery.out,-p->roll_target_limit,p->roll_target_limit);
 asc_pid_loc_calc(&s->roll_angle,a->roll-rt);
 roll=asc_pid_loc_calc_limited(&s->roll_rate,a->roll_rate+s->roll_angle.out,-p->fly_limit,p->fly_limit);
 pt=p->pitch_zero+s->pitch_speed.out;
 asc_pid_loc_calc(&s->pitch_angle,pt-a->pitch);
 pitch_dot=a->pitch_rate*cosf(a->roll*.017453292519943f)-a->yaw_rate*sinf(a->roll*.017453292519943f);
 asc_pid_inc_calc_limited(&s->pitch_rate,-pitch_dot+s->pitch_angle.out,-p->ground_limit,p->ground_limit);
 yaw=0;headroom=p->fly_limit-fabsf(roll);
 if(p->yaw_enabled){asc_pid_loc_calc(&s->yaw_angle,s->yaw_target-a->yaw);yaw=asc_pid_loc_calc(&s->yaw_rate,s->yaw_angle.out-a->yaw_rate);}
 else{asc_pid_reset(&s->yaw_angle);asc_pid_reset(&s->yaw_rate);s->yaw_target=a->yaw;}
 yaw=asc_clamp(yaw,-headroom,headroom);
 /* With Qcar's retained LEFT/RIGHT=-1 driver mapping these logical values
  * produce the upstream physical A=-Roll+Yaw, B=Roll+Yaw combination.
  * roll_sign is the existing verified Qcar balance sign, normally -1. */
 s->output.roll_pwm=-p->roll_sign*roll;s->output.yaw_pwm=yaw;
 s->output.pitch_pwm=s->pitch_rate.out;
 s->output.left_pwm=s->output.roll_pwm-yaw;s->output.right_pwm=-s->output.roll_pwm-yaw;
 s->output.ground_pwm=-p->ground_sign*s->pitch_rate.out;
 s->output.roll_target=rt;s->output.pitch_target=pt;
 s->output.roll_target_rate=-s->roll_angle.out;s->output.pitch_target_rate=s->pitch_angle.out;
 s->output.roll_rate_error=-s->roll_angle.out-a->roll_rate;
 s->output.pitch_rate_error=s->pitch_angle.out-pitch_dot;
 s->output.pitch_feedback_rate=pitch_dot;
 s->output.recovery=s->recovery.out;s->output.ground_tilt=s->pitch_speed.out;
 *o=s->output;
 if(!isfinite(o->left_pwm)||!isfinite(o->right_pwm)||!isfinite(o->ground_pwm)){
  asc_control_reset(s);*o=s->output;return 0;
 }
 return 1;
}
