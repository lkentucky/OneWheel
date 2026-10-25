"""Execute real upright/PID C with a fake hardware boundary (no target board)."""
from pathlib import Path
import subprocess,tempfile,os
root=Path(__file__).resolve().parents[1]
compiler=os.environ.get('HOST_CC','gcc')
with tempfile.TemporaryDirectory(prefix='onewheel-balance-') as d:
    work=Path(d)
    (work/'zf_common_headfile.h').write_text(r'''
#ifndef HOST_BOUNDARY
#define HOST_BOUNDARY
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "qcar_types.h"
typedef uint8_t uint8;typedef uint32_t uint32;typedef int32_t int32;typedef int16_t int16;
#define IMU_ATTITUDE_CALIBRATING 0
extern struct host_attitude { struct {float x,y;} gyro; } attitude_state;
extern volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms;
extern volatile uint32 imu_sample_sequence;
extern volatile int32 imu_attitude_status;
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32);
void small_driver_set_duty(int16,int16);
void motor_m_ctrl(int);
#endif
''')
    (work/'test.c').write_text(r'''
#include "balance_control.h"
#include <assert.h>
#include <stdio.h>
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms=5;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status;
static int fly_l,fly_r,ground;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void small_driver_set_duty(int16 l,int16 r){fly_l=l;fly_r=r;}
void motor_m_ctrl(int v){ground=v;}
static void tick(int fresh){if(fresh)++imu_sample_sequence;for(int i=0;i<APP_CONTROL_DIVIDER;++i)balance_control_tick_1ms();}
static void arm(int mode){
    balance_control_defaults();control_startup_ramp_ms=0;shacheflag=1;
    imu_attitude_status=1;imu_roll_deg=BALANCE_ZERO_X_DEG;imu_pitch_deg=BALANCE_ZERO_Y_DEG;
    attitude_state.gyro.x=attitude_state.gyro.y=0;++imu_sample_sequence;
    if(mode==3)roll_rate_test_arm();else if(mode==4)ground_rate_test_arm();
    else if(mode==5)dual_balance_arm();else if(mode==6)roll_rate_only_arm();else ground_rate_only_arm();
}
int main(void){
    /* Rate-only axis ownership, signs, output limits and P-only equivalence. */
    arm(6);attitude_state.gyro.y=10;tick(1);assert(fly_l==-350 && fly_r==350 && ground==0);
    arm(7);attitude_state.gyro.x=10;tick(1);assert(ground==-300 && fly_l==0 && fly_r==0);
    ground_rate_test_sign=-1;tick(1);assert(ground==300);
    ground_rate_test_limit=100;tick(1);assert(ground==100);
    arm(5);tick(1);assert(!ground && !fly_l && !fly_r);
    /* Compare the original default P/P + LPF equation over a moving trajectory. */
    float filter[2]={BALANCE_ZERO_Y_DEG,BALANCE_ZERO_X_DEG};
    float alpha=0.005f/(BALANCE_ANGLE_LPF_TAU_S+0.005f);
    for(int k=0;k<150;++k){
        imu_pitch_deg=BALANCE_ZERO_Y_DEG+((k%17)-8)*0.12f;
        imu_roll_deg=BALANCE_ZERO_X_DEG+((k%13)-6)*0.15f;
        attitude_state.gyro.y=((k%7)-3)*0.8f;attitude_state.gyro.x=((k%9)-4)*0.7f;
        filter[0]+=alpha*(imu_pitch_deg-filter[0]);filter[1]+=alpha*(imu_roll_deg-filter[1]);
        float tr0=FLY_ANGLE_KP*(BALANCE_ZERO_Y_DEG-filter[0]);
        float tr1=GROUND_ANGLE_KP*(BALANCE_ZERO_X_DEG-filter[1]);
        if(tr0>FLY_RATE_LIMIT_DPS)tr0=FLY_RATE_LIMIT_DPS;
        if(tr0< -FLY_RATE_LIMIT_DPS)tr0= -FLY_RATE_LIMIT_DPS;
        if(tr1>GROUND_RATE_LIMIT_DPS)tr1=GROUND_RATE_LIMIT_DPS;
        if(tr1< -GROUND_RATE_LIMIT_DPS)tr1= -GROUND_RATE_LIMIT_DPS;
        float uf=FLY_RATE_KP*(tr0-attitude_state.gyro.y);
        float ug=GROUND_RATE_KP*(tr1-attitude_state.gyro.x);
        if(uf>FLY_PWM_LIMIT)uf=FLY_PWM_LIMIT;if(uf< -FLY_PWM_LIMIT)uf= -FLY_PWM_LIMIT;
        if(ug>GROUND_PWM_LIMIT)ug=GROUND_PWM_LIMIT;if(ug< -GROUND_PWM_LIMIT)ug= -GROUND_PWM_LIMIT;
        int f=(int)roundf(uf),g=(int)roundf(ug);
        tick(1);
        if(fly_l!=f || fly_r!=-f || ground!=g)fprintf(stderr,"k=%d actual=%d,%d expected=%d,%d\n",k,fly_l,ground,f,g);
        assert(fly_l==f && fly_r==-f && ground==g);
    }
    /* LPF cannot delay tilt/nonfinite protection; faults stay latched. */
    arm(5);imu_roll_deg=31;tick(1);assert(roll_rate_test_state==-2 && !ground && !fly_l);
    imu_roll_deg=0;tick(1);assert(roll_rate_test_state==-2 && !ground);
    arm(5);imu_pitch_deg=NAN;tick(1);assert(roll_rate_test_state==-1 && !fly_l && !ground);
    arm(5);imu_attitude_status=-2;tick(1);assert(control_fault_reason==1 && !ground && !fly_l);
    arm(5);for(int k=0;k<5;++k)tick(0);assert(control_fault_reason==4 && !ground && !fly_l);
    arm(5);imu_attitude_status=0;tick(1);assert(roll_rate_test_state==1 && !ground && !fly_l);
    arm(5);ground_rate_test_sign=0;tick(1);assert(roll_rate_test_state==-3 && !ground && !fly_l);
    arm(5);attitude_state.gyro.x=4;tick(1);assert(ground!=0);shacheflag=0;tick(1);assert(!ground && !fly_l);
    arm(7);control_startup_ramp_ms=1000;attitude_state.gyro.x=100;tick(1);assert(control_startup_scale>0 && control_startup_scale<0.01f && ground==-20);
    roll_rate_test_stop();balance_control_tick_1ms();assert(!ground && !fly_l && !roll_rate_test_enabled);
    puts("balance control: axes, reference output, LPF, limits, ramp, stop and faults PASS");
    return 0;
}
''')
    exe=work/'balance.exe'
    subprocess.run([compiler,'-I'+str(work),'-I'+str(root/'code'),str(root/'code/balance_control.c'),str(root/'code/pid.c'),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
