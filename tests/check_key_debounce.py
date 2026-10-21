"""Exercise the actual key driver with simulated GPIO and interrupt state."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
stub = r'''
#ifndef KEY_TEST_STUB
#define KEY_TEST_STUB
#include <stdint.h>
#include <assert.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int gpio_pin_enum;
#define P20_6 0
#define P20_7 1
#define P11_2 2
#define P11_3 3
#define P20_8 4
#define P20_9 5
#define GPIO_HIGH 1
#define GPI 0
#define GPI_PULL_UP 0
#define zf_assert assert
int gpio_get_level(int pin);
void gpio_init(int pin,int mode,int level,int pull);
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 mask);
#endif
'''
test = r'''
#include <stdio.h>
#include "zf_device_key.h"
static int pins[8], irq_enabled=1;
static const int mapping[] = KEY_LIST;
int gpio_get_level(int pin) { return pins[pin]; }
void gpio_init(int pin,int mode,int level,int pull) { pins[pin]=level; }
uint32 interrupt_global_disable(void) { uint32 mask=!irq_enabled; irq_enabled=0; return mask; }
void interrupt_global_enable(uint32 mask) { if (!mask) irq_enabled=1; }
static void ticks(int count) { while(count--) key_scanner(); }
static void set(int key,int pressed) { pins[mapping[key]]=!pressed; }
int main(void) {
    for(int period=1;period<=10;period*= (period==1 ? 5 : 2)) {
        key_init(period);
        int n=(KEY_MAX_SHOCK_PERIOD+period-1)/period;
        set(0,1); ticks(n-1); assert(key_get_state(KEY_1)==KEY_RELEASE);
        ticks(1); assert(key_get_state(KEY_1)==KEY_SHORT_PRESS);
        ticks(500/period); assert(key_get_state(KEY_1)==KEY_SHORT_PRESS);
        key_consume_state(KEY_1,KEY_SHORT_PRESS);
        ticks(100/period); assert(key_get_state(KEY_1)==KEY_RELEASE);
        set(0,0); ticks(n);
        for(int i=0;i<3;i++) { set(0,1);ticks(n);set(0,0);ticks(n); }
        ticks(200/period);
        for(int i=0;i<3;i++) { assert(key_get_state(KEY_1)==KEY_SHORT_PRESS); key_clear_state(KEY_1); }
        assert(key_get_state(KEY_1)==KEY_RELEASE);
    }
    key_init(1);
    for(int i=0;i<8;i++) { set(0,i%2);ticks(1); }
    set(0,0);ticks(20);assert(key_get_state(KEY_1)==KEY_RELEASE);
    set(0,1);ticks(10);key_clear_state(KEY_1);
    for(int i=0;i<8;i++) { set(0,i%2);ticks(1); }
    set(0,1);ticks(20);assert(key_get_state(KEY_1)==KEY_RELEASE);
    ticks(1100);assert(key_get_state(KEY_1)==KEY_LONG_PRESS);
    key_clear_state(KEY_1);ticks(1100);assert(key_get_state(KEY_1)==KEY_RELEASE);
    set(0,0);ticks(10);assert(key_get_state(KEY_1)==KEY_RELEASE);
    set(0,1);set(1,1);ticks(10);key_consume_state(KEY_1,KEY_SHORT_PRESS);
    assert(key_get_state(KEY_2)==KEY_SHORT_PRESS);
    /* Consuming an old long event must not eat a newly pending short event. */
    key_consume_state(KEY_2,KEY_LONG_PRESS);assert(key_get_state(KEY_2)==KEY_SHORT_PRESS);
    irq_enabled=0;key_clear_all_state();assert(!irq_enabled);
    irq_enabled=1;key_clear_all_state();assert(irq_enabled);
    puts("PASS: debounce, immediate press, delayed polling, rapid presses, bounce, hold, multi-key and IRQ restore");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-debounce-') as directory:
    work = Path(directory)
    for name in ['zf_device_key.c','zf_device_key.h']:
        shutil.copy2(root/'libraries/zf_device'/name,work/name)
    for name in ['zf_common_debug.h','zf_driver_gpio.h','zf_common_interrupt.h']:
        (work/name).write_text(stub,encoding='utf-8')
    (work/'test.c').write_text(test,encoding='utf-8')
    exe=work/'test.exe'
    subprocess.run(['gcc','-std=c99','-I'+str(work),str(work/'zf_device_key.c'),
                    str(work/'test.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
