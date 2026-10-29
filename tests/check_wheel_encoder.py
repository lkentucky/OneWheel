"""Run the actual observation module with GPT12/STM boundaries and no actuators."""
from pathlib import Path
import subprocess, tempfile

root = Path(__file__).resolve().parents[1]
boundary = r'''
#ifndef ENCODER_HOST_H
#define ENCODER_HOST_H
#include <stdint.h>
#include "config.h"
#define TIM2_ENCODER 2
int16_t encoder_get_count(int timer);
uint32_t interrupt_global_disable(void);
void interrupt_global_enable(uint32_t);
#endif
'''
stm = r'''
#include <stdint.h>
extern int host_stm;
#define MODULE_STM0 host_stm
float IfxStm_getFrequency(int *);
uint32_t IfxStm_getLower(int *);
'''
test = r'''
#include "wheel_encoder.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
int host_stm;
static uint32_t ticks, reads, irq;
static uint16_t counter;
static float frequency=100000000.0f;
float IfxStm_getFrequency(int *p){(void)p;return frequency;}
uint32_t IfxStm_getLower(int *p){(void)p;return ticks;}
int16_t encoder_get_count(int timer){assert(timer==2);++reads;return (int16_t)counter;}
uint32_t interrupt_global_disable(void){uint32_t old=irq;irq=1;return old;}
void interrupt_global_enable(uint32_t old){assert(irq);irq=old;}
static void step(unsigned us,int delta){ticks+=us*100u;counter=(uint16_t)(counter+delta);wheel_encoder_tick_1ms();}
static void init(uint16_t count){counter=count;wheel_encoder_init();assert(!irq && !wheel_encoder_seq && !wheel_encoder_valid);}
int main(void){
    step(20000,123);assert(!reads && !wheel_encoder_seq); /* Before hardware init. */
    assert(!wheel_encoder_rebase_position() && !reads);
    init(65530);assert(reads==1 && wheel_encoder_raw==65530);
    wheel_encoder_sample_t snap;wheel_encoder_snapshot(&snap);
    assert(!snap.sequence && !snap.valid && !snap.age_ms);
    for(int i=0;i<20;++i)step(1000,1);
    assert(reads==11 && wheel_encoder_seq==1 && wheel_encoder_raw==14);
    assert(wheel_encoder_delta==20*WHEEL_ENCODER_SIGN && wheel_position_count==20*WHEEL_ENCODER_SIGN);
    assert(fabsf(wheel_speed_cps-1000*WHEEL_ENCODER_SIGN)<.01f && fabsf(wheel_encoder_dt_ms-20)<.001f);
    for(int i=0;i<20;++i)step(1000,-1);
    assert(wheel_encoder_raw==65530 && wheel_position_count==0 && wheel_encoder_delta==-20*WHEEL_ENCODER_SIGN);
    assert(wheel_encoder_valid && wheel_position_valid && !wheel_encoder_errors);
    ticks+=500000u;wheel_encoder_snapshot(&snap);
    assert(snap.sequence==2 && snap.valid && snap.age_ms==5);
    ticks-=500000u;
    for(int i=0;i<20;++i)step(1000,0);
    assert(wheel_encoder_seq==3 && !wheel_speed_cps && wheel_encoder_valid); /* 0 is valid stillness. */
    init(30000);
    for(int i=0;i<7;++i)step(3000,3);
    assert(wheel_encoder_delta==21*WHEEL_ENCODER_SIGN && fabsf(wheel_encoder_dt_ms-21)<.001f);
    assert(fabsf(wheel_speed_cps-1000*WHEEL_ENCODER_SIGN)<.01f); /* Actual elapsed, not fixed 20ms. */
    ticks=UINT32_MAX-99999u;init(0);
    for(int i=0;i<20;++i)step(1000,-2);
    assert(wheel_encoder_seq==1 && wheel_encoder_delta==-40*WHEEL_ENCODER_SIGN && wheel_encoder_valid);
    step(50000,1000);assert(!wheel_encoder_valid && !wheel_position_valid && wheel_encoder_errors==1);
    assert(!wheel_encoder_delta && !wheel_speed_cps && wheel_position_count==-40*WHEEL_ENCODER_SIGN);
    for(int i=0;i<20;++i)step(1000,0);
    assert(wheel_encoder_valid && !wheel_position_valid && wheel_encoder_errors==1);
    init(0);step(2000,32768);assert(!wheel_encoder_valid && !wheel_position_valid && wheel_encoder_errors==1);
    for(int i=0;i<20;++i)step(1000,0);
    assert(wheel_encoder_valid && !wheel_position_valid && wheel_position_count==0);
    init(0);
    for(int i=0;i<520;++i){step(2000,32767);if(wheel_encoder_errors)assert(!wheel_position_valid);}
    assert(!wheel_position_valid && wheel_encoder_errors>0 && wheel_encoder_valid);
    assert(wheel_position_count<=WHEEL_ENCODER_POSITION_LIMIT && wheel_position_count>=-WHEEL_ENCODER_POSITION_LIMIT);
    frequency=NAN;init(0);uint32_t old=reads;step(20000,5);assert(reads==old && wheel_encoder_errors==1);
    frequency=100000000.0f;init(0);
    for(int i=0;i<20;++i)step(1000,0);
    assert(wheel_encoder_valid && wheel_position_valid && !wheel_position_count && !wheel_encoder_errors);
    wheel_encoder_snapshot(&snap);uint32_t old_epoch=snap.position_epoch;
    step(2000,15);assert(wheel_encoder_rebase_position());
    wheel_encoder_snapshot(&snap);
    assert(snap.position_epoch!=old_epoch && !snap.position_count && !snap.position_valid && !snap.valid && !snap.dt_ms);
    for(int i=0;i<20;++i)step(1000,1);
    wheel_encoder_snapshot(&snap);
    assert(snap.position_valid && snap.valid && snap.position_count==20*WHEEL_ENCODER_SIGN);
    step(50000,1000);wheel_encoder_snapshot(&snap);assert(!snap.position_valid);
    assert(wheel_encoder_rebase_position());
    for(int i=0;i<20;++i)step(1000,0);
    wheel_encoder_snapshot(&snap);assert(snap.position_valid && !snap.position_count && wheel_encoder_errors==1);
    printf("PASS: encoder sign %d, init gate, +/-wrap, measured dt, STM wrap, gap/recovery, ambiguity, immediate bounds invalidation\n",WHEEL_ENCODER_SIGN);
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-encoder-') as directory:
    work = Path(directory)
    (work/'zf_common_headfile.h').write_text(boundary)
    (work/'IfxStm.h').write_text(stm)
    (work/'test.c').write_text(test)
    for sign in (1, -1):
        exe=work/f'encoder-{sign}.exe'
        subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',f'-DWHEEL_ENCODER_SIGN={sign}',
                        '-I'+str(work),'-I'+str(root/'code'),str(root/'code/wheel_encoder.c'),
                        str(work/'test.c'),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=10)

# Integration ownership: Stop cannot stop this timer and hardware init precedes ready.
isr=(root/'user/isr.c').read_text(encoding='utf-8')
key_isr=isr[isr.index('IFX_INTERRUPT(cc61_pit_ch1_isr'):isr.index('// **************************** 外部中断')]
assert 'wheel_encoder_tick_1ms();' in key_isr
menu=(root/'code/menu.c').read_text(encoding='utf-8')
stop=menu[menu.index('static void stop_car(void)'):menu.index('static void start_car(void)')]
assert 'CCU61_CH1' not in stop
motor=(root/'code/motor.c').read_text(encoding='utf-8')
assert motor.index('encoder_dir_init(')<motor.index('wheel_encoder_init();')
