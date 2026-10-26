"""Exercise extracted left/right driver protection code; no hardware or protection bypass."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
vendor=root/'tools/cyt2bl3_driver/code'
def read(p):
    data=p.read_bytes()
    try:return data.decode('utf-8-sig')
    except UnicodeDecodeError:return data.decode('gbk')
def block(s,start):
    a=s.index(start);b=s.index('{',a)+1;d=1
    while d:d+=(s[b]=='{')-(s[b]=='}');b+=1
    return s[a:b]
source=read(vendor/'motor_control.c')
prefix = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "vendor_config.h"
#define SENSORLESS 2
#define BRUSHED 3
#define HALL_SIX_STEP 1
#define NORMAL_RUN_MODE 0
#define PROTECT_MODE 1
#define OVERLOAD_PROTECT 2
#define LOCKED_PROTECT 3
#define ENCODER_NORMAL 0
typedef struct {float motor_duty,motor_speed_filter,overload_conefficient;unsigned driver_mode,overload_count,motor_protect_state,motor_protect_cause,encoder_state;struct{unsigned protect_flag,protect_check_time;float protect_duty_max;}locked_value;} Motor;
static Motor motor_left;
static struct{float battery_voltage;}battery_value;
static unsigned left_protect_count;
static float func_abs(float x){return fabsf(x);}
static float func_limit_ab(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
static void reset(void){memset(&motor_left,0,sizeof motor_left);battery_value.battery_voltage=12;left_protect_count=0;motor_left.locked_value.protect_flag=1;motor_left.locked_value.protect_duty_max=MOTOR_LOCKED_DUTY_MAX;motor_left.locked_value.protect_check_time=MOTOR_LOCKED_TIME;}
'''
parts=[]
for side in ['left','right']:
    local=source[source.index('void motor_'+side+'_update_isr('):]
    a=local.index('#if MOTOR_OVERLOAD_PROTECT');b=local.index('#endif',a)+len('#endif')
    overload=local[a:b].replace('motor_right','motor_left')
    stall=block(local,'if(motor_'+side+'.locked_value.protect_flag)').replace('motor_right','motor_left').replace('right_protect_count','left_protect_count')
    parts.append('static void '+side+'_overload(void){\n'+overload+'\n}\nstatic void '+side+'_stall(void){'+stall+'}\n')
main = r'''
int main(void){
    void (*overloads[2])(void)={left_overload,right_overload};
    void (*stalls[2])(void)={left_stall,right_stall};
    for(unsigned side=0;side<2;++side){
        reset();motor_left.motor_duty=.254f;motor_left.motor_speed_filter=250;
        for(unsigned i=0;i<60001;++i)overloads[side]();
        assert(motor_left.motor_protect_cause==OVERLOAD_PROTECT && motor_left.encoder_state==ENCODER_NORMAL);
        reset();motor_left.motor_duty=.3f;motor_left.motor_speed_filter=300;
        for(unsigned i=0;i<32000;++i)overloads[side]();
        motor_left.motor_duty=0;for(unsigned i=0;i<20000;++i)overloads[side]();
        assert(!motor_left.overload_count);
        motor_left.motor_duty=.3f;for(unsigned i=0;i<32000;++i)overloads[side]();
        assert(motor_left.motor_protect_state==NORMAL_RUN_MODE);
        reset();motor_left.motor_duty=.3f;motor_left.motor_speed_filter=0;
        for(unsigned i=0;i<1001;++i)stalls[side]();
        assert(motor_left.motor_protect_cause==LOCKED_PROTECT);
        motor_left.motor_duty=0;for(unsigned i=0;i<1000;++i)stalls[side]();
        assert(motor_left.motor_protect_state==PROTECT_MODE);
        printf("PASS %s: zero duty interrupts overload accumulation; continuous overload/stall still latch\n",side?"right":"left");
    }
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-driver-protect-') as directory:
    work=Path(directory)
    (work/'vendor_config.h').write_text(read(vendor/'driver_config.h').replace('#include "zf_common_headfile.h"',''),encoding='utf-8')
    (work/'test.c').write_text(prefix+''.join(parts)+main,encoding='utf-8')
    exe=work/'test.exe'
    subprocess.run(['gcc','-std=c99','-I'+str(work),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
