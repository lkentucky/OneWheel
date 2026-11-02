"""ASC Flash schema: persist new gains, reject unlike-unit legacy records and torn writes."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
source=(root/'code/flash.c').read_text(encoding='utf-8');payload=source[source.index('#define BALANCE_STORAGE_MAGIC'):]
array=payload[payload.index('static float *const values'):payload.index('static uint32 checksum_word')]
names=sorted(set(re.findall(r'&([A-Za-z_]\w*)',array))-{'mono_params'})
prefix=r"""
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "config.h"
typedef uint32_t uint32;
static struct {float balance_angle_x,balance_angle_y;} mono_params;
static uint32 control_startup_ramp_ms;
static uint32 memory[64];static unsigned writes,erases,last_page;static int dropped=-1,irq;
#define my_flash_read(sector,page,type) memory[page]
static uint32 interrupt_global_disable(void){assert(!irq);irq=1;return 0;}
static void interrupt_global_enable(uint32 n){assert(!n && irq);irq=0;}
static void eeprom_erase_sector(uint32 n){assert(!n && !irq);memset(memory,0xff,sizeof(memory));++erases;}
static void eeprom_page_program(uint32 n,uint32 p,uint32 *v){assert(!n && p<64 && !irq);if((int)p!=dropped)memory[p]=*v;++writes;last_page=p;}
static void IfxFlash_resetToRead(int n){assert(!n);}
static void balance_control_defaults(void);
"""
prefix+='\nstatic float '+','.join(names)+';\n'
test=r"""
static void balance_control_defaults(void){
 for(unsigned i=0;i<BALANCE_PARAMETER_COUNT;i++)*values[i]=0;
 roll_rate_test_kp=ASC_FLY_RATE_KP;roll_rate_test_limit=ASC_FLY_PWM_LIMIT;roll_rate_test_sign=FLY_OUTPUT_SIGN;
 pitch_balance_kp=ASC_FLY_ANGLE_KP;pitch_balance_rate_limit=FLY_RATE_LIMIT_DPS;
 ground_rate_test_kp=ASC_GROUND_RATE_KP;ground_rate_test_limit=ASC_GROUND_PWM_LIMIT;ground_rate_test_sign=GROUND_OUTPUT_SIGN;
 ground_angle_kp=ASC_GROUND_ANGLE_KP;ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;
 mono_params.balance_angle_x=BALANCE_ZERO_X_DEG;mono_params.balance_angle_y=BALANCE_ZERO_Y_DEG;
 fly_rate_kd=ASC_FLY_RATE_KD;asc_ground_rate_ki=ASC_GROUND_RATE_KI;asc_ground_speed_kp=ASC_GROUND_SPEED_KP;
 asc_ground_speed_ki=ASC_GROUND_SPEED_KI;asc_yaw_enabled=1;asc_yaw_angle_kp=18;asc_yaw_rate_kp=56;control_startup_ramp_ms=1000;
}
int main(void){
 balance_control_defaults();roll_rate_test_kp=42;fly_rate_kd=7;asc_ground_rate_ki=.5f;asc_ground_speed_ki=.003f;asc_yaw_enabled=0;
 assert(write_flash());assert(last_page==0&&memory[0]==BALANCE_STORAGE_MAGIC);
 get_flash();
#if QCAR_BOOT_LOAD_FLASH
 assert(roll_rate_test_kp==42&&fly_rate_kd==7&&asc_ground_rate_ki==.5f&&asc_ground_speed_ki==.003f&&asc_yaw_enabled==0);
 memory[STORAGE_CRC_PAGE]^=1;get_flash();assert(roll_rate_test_kp==ASC_FLY_RATE_KP&&fly_rate_kd==8);
 assert(write_flash());memory[0]=0x4f574232u;get_flash();assert(roll_rate_test_kp==ASC_FLY_RATE_KP);
 for(unsigned page=0;page<=STORAGE_CRC_PAGE;++page){balance_control_defaults();roll_rate_test_kp=42;dropped=(int)page;int ok=write_flash();assert(!ok);get_flash();assert(roll_rate_test_kp==ASC_FLY_RATE_KP);}dropped=-1;
#else
 assert(roll_rate_test_kp==ASC_FLY_RATE_KP&&fly_rate_kd==8);
#endif
 balance_control_defaults();asc_yaw_enabled=.5f;unsigned count=erases;assert(!write_flash()&&erases==count);
 balance_control_defaults();fly_rate_kd=NAN;assert(!write_flash()&&erases==count);
 puts("PASS: ASC parameter roundtrip, distinct magic/units, legacy/CRC rejection, every torn-write page, invalid values and boot-load policy");
}
"""
with tempfile.TemporaryDirectory(prefix='qcar-asc-flash-') as d:
 w=Path(d);(w/'test.c').write_text(prefix+payload+test,encoding='utf-8')
 for enabled in [1,0]:
  exe=w/f'test-{enabled}.exe'
  subprocess.run(['gcc','-std=c99','-Wall','-Wextra',f'-DQCAR_BOOT_LOAD_FLASH={enabled}','-I'+str(root/'code'),str(w/'test.c'),'-lm','-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True,timeout=10)
