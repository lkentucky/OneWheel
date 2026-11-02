#include "asc_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float a,float b){if(fabsf(a-b)>.01f)printf("got %f expected %f\n",a,b);assert(fabsf(a-b)<.01f);}
static asc_config_t cfg(void){asc_config_t c={0};c.roll_angle_kp=36;c.roll_rate_kp=20;c.roll_rate_kd=-8;
 c.recovery_kp=.0015f;c.pitch_speed_kp=.06f;c.pitch_speed_ki=.002f;c.pitch_angle_kp=9;c.pitch_rate_kp=20;c.pitch_rate_ki=.64f;
 c.yaw_angle_kp=18;c.yaw_rate_kp=-56;c.fly_limit=10000;c.ground_limit=8000;c.roll_target_limit=10;c.pitch_tilt_limit=8;c.roll_sign=-1;c.ground_sign=1;return c;}
int main(void){asc_controller_t s;asc_config_t p=cfg();asc_input_t a={0};asc_output_t o;
 float zr,zp;asc_reference_zero(30,20,&zr,&zp);near(zr,22.795877f);near(zp,-28.02432f);
 asc_control_reset(&s);a.speed_fresh=1;assert(asc_control_update(&s,&p,&a,&o));near(o.left_pwm,0);
 a.roll=1;assert(asc_control_update(&s,&p,&a,&o));near(o.roll_pwm,432); /* 20*36 - 8*36 */
 asc_control_update(&s,&p,&a,&o);near(o.roll_pwm,720);near(o.left_pwm,720);near(o.right_pwm,-720);
 a.roll=100;asc_control_update(&s,&p,&a,&o);assert(fabsf(o.left_pwm)<=10000);
 a.roll=0;asc_control_update(&s,&p,&a,&o);asc_control_update(&s,&p,&a,&o);near(o.roll_pwm,0); /* no frozen P offset */
 asc_control_reset(&s);a=(asc_input_t){0};a.left_rpm=1000;a.speed_fresh=1;
 asc_control_update(&s,&p,&a,&o);near(o.recovery,1.5f);near(o.roll_target,-1.5f);
 a.left_rpm=2000;for(int i=0;i<19;i++){asc_control_update(&s,&p,&a,&o);near(o.recovery,1.5f);}
 asc_control_update(&s,&p,&a,&o);near(o.recovery,3);
 a.left_rpm=30000;for(int i=0;i<20;i++)asc_control_update(&s,&p,&a,&o);near(o.roll_target,-10);
 asc_control_reset(&s);a=(asc_input_t){0};a.speed_fresh=1;a.ground_counts_20ms=2;
 asc_control_update(&s,&p,&a,&o);near(o.ground_tilt,.124f);near(o.pitch_pwm,23.03424f);near(o.ground_pwm,-o.pitch_pwm);
 for(int i=0;i<100;i++){a.ground_counts_20ms=10000;asc_control_update(&s,&p,&a,&o);}assert(fabsf(o.ground_tilt)<=8);
 asc_control_reset(&s);a=(asc_input_t){0};p.yaw_enabled=1;a.yaw=10;asc_control_update(&s,&p,&a,&o);
 a.yaw=11;a.yaw_rate=2;asc_control_update(&s,&p,&a,&o);near(o.yaw_pwm,1120);near(o.left_pwm,-1120);near(o.right_pwm,-1120);
 a.roll=100;asc_control_update(&s,&p,&a,&o);near(o.yaw_pwm,0);assert(fabsf(o.left_pwm)<=10000&&fabsf(o.right_pwm)<=10000);
 asc_control_reset(&s);a=(asc_input_t){0};a.roll=30;a.pitch_rate=10;a.yaw_rate=2;
 asc_control_update(&s,&p,&a,&o);near(o.pitch_feedback_rate,10*cosf(30*.01745329252f)-2*sinf(30*.01745329252f));
 a.roll=NAN;assert(!asc_control_update(&s,&p,&a,&o));near(o.left_pwm,0);near(o.ground_pwm,0);
 asc_control_reset(&s);a=(asc_input_t){0};p=cfg();p.pitch_angle_kp=0;p.pitch_rate_kp=0;p.pitch_rate_ki=0;p.pitch_rate_kd=0;
 a.pitch=2;a.pitch_rate=3;a.speed_fresh=1;asc_control_update(&s,&p,&a,&o);near(o.ground_pwm,0);
 puts("PASS: ASC equations, positional Roll recovery, per-update D, 20ms loops, incremental Pitch, yaw headroom, invalid inputs and ground disable");return 0;}
