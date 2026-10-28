"""Execute the real passive recorder and UART export with simulated registers."""
from pathlib import Path
import subprocess,tempfile,os,sys
root=Path(__file__).resolve().parents[1]
header=r'''
#include <stdint.h>
#include <stddef.h>
uint32_t trace_ticks(void);uint32_t trace_lock(void);void trace_unlock(uint32_t);
int trace_zero(void);uint16_t trace_flags(void);
void trace_widths(uint16_t *,uint16_t *);void trace_send(const char *,size_t);
'''
test=r'''
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "driver_trace.c"
static uint32_t ticks=0xffff0000u;
static int zero=1,locked;
static uint16_t hw_active[3],hw_buffer[3];
static char output[1600];
uint32_t trace_ticks(void){return ticks;}
uint32_t trace_lock(void){assert(!locked);locked=1;return 0;}
void trace_unlock(uint32_t n){(void)n;assert(locked);locked=0;}
int trace_zero(void){return zero;}
uint16_t trace_flags(void){return 0;}
void trace_widths(uint16_t *a,uint16_t *b){memcpy(a,hw_active,6);memcpy(b,hw_buffer,6);}
void trace_send(const char *s,size_t n){assert(!locked && n<sizeof output);memcpy(output,s,n);output[n]=0;
const char *path=getenv("DRIVER_TRACE_FIXTURE");if(path){FILE *f=fopen(path,"ab");assert(f);assert(fwrite(s,1,n,f)==n);fclose(f);}}
static void verify_crc(void){
    uint16_t c=0xffff;char *star=strchr(output,'*');assert(star);
    for(char *p=output;p<star;p++){c^=(uint16_t)((uint8_t)*p<<8);for(int j=0;j<8;j++)c=(uint16_t)(c&0x8000?(c<<1)^0x1021:c<<1);}
    assert(c==strtoul(star+1,0,16));
}
static void command(const char *s){assert(driver_trace_text((const uint8_t*)s));driver_trace_service();verify_crc();}
static void irq_tick(int delta){
    ticks+=400u;memcpy(hw_active,hw_buffer,6);
    driver_trace_on_encoder((int32_t)(ticks%32768u),delta);
    uint16_t g=driver_trace_generation();
    hw_buffer[0]=(uint16_t)abs(last_left);hw_buffer[1]=(uint16_t)(abs(last_left)/2);hw_buffer[2]=0;
    driver_trace_on_pwm_written(g);
}
static void duty(int value){
    driver_trace_on_rx_byte('\n');ticks+=16;
    driver_trace_on_command((int16_t)value,0);
    zero=value==0;
}
int main(void){
    driver_trace_init();
    assert(!driver_trace_text((const uint8_t*)"SET-DUTY,1,2\n"));
    command("SET-TRACE,1\n");assert(trace_state==1 && capture_id==1);
    for(unsigned i=0;i<4000;i++)irq_tick(0); /* Pretrigger ring wraps. */
    assert(pre_count==128 && post_count==0);
    duty(-300);assert(trace_state==2 && event_count==1);
    uint32_t start=trigger_tick;
    for(unsigned i=0;i<150100;i++){
        if(i==30000)duty(-600);
        if(i==60000)duty(-300);
        if(i==90000)duty(0);
        irq_tick(i<90000?1:0);
    }
    assert(trace_state==3 && event_count==4 && !overflow && pre_count==128);
    assert(post_count>=7490 && post_count<=7501);
    for(unsigned i=0;i<4;i++) {
        assert(events[i].received!=UINT32_MAX && events[i].dispatched-events[i].received==16);
        assert(events[i].written-events[i].dispatched<=400);
        assert(events[i].active-events[i].written==400);
    }
    assert((uint32_t)(ticks-start)>=TRACE_DURATION && max_gap>=400);
    command("GET-TRACE,65535\n");assert(!strncmp(output,"TR,M,1,3,",9));
    command("GET-TRACE,0\n");assert(!strncmp(output,"TR,D,1,0,8;",11));
    /* Chronological pretrigger rows and uninterrupted accumulated counts. */
    unsigned oldest=(pre_head+128-pre_count)%128;
    uint32_t previous=rows[oldest].ticks;
    for(unsigned i=1;i<128;i++){uint32_t t=rows[(oldest+i)%128].ticks;assert(t>previous);previous=t;}
    assert(rows[128].ticks>previous && rows[128].event==0);
    command("GET-TRACE,60000\n");assert(!strncmp(output,"TR,E,1,0,",9));
    command("GET-TRACE,8191\n");assert(strstr(output,"index"));
    for(unsigned i=0;i<event_count;i++){char cmd[64];snprintf(cmd,sizeof cmd,"GET-TRACE,%u\n",60000+i);command(cmd);}
    for(unsigned i=0;i<pre_count+post_count;i+=8){char cmd[64];snprintf(cmd,sizeof cmd,"GET-TRACE,%u\n",i);command(cmd);}
    uint32_t count=post_count;for(unsigned i=0;i<100;i++)irq_tick(99);assert(post_count==count);
    command("SET-TRACE,0\n");assert(trace_state==4);
    command("SET-TRACE,1\n");assert(trace_state==1 && capture_id==2);
    driver_trace_text((const uint8_t*)"SET-TRACE,2\n");assert(!request);
    duty(-300);irq_tick(1);zero=0;
    driver_trace_text((const uint8_t*)"GET-TRACE,65535\n");driver_trace_service();assert(request==3);
    zero=1;driver_trace_service();assert(!request);verify_crc();
    puts("PASS: recorder clocks/wrap, UART end/dispatch/PWM active timing, pretrigger order, bounded 1kHz capture, CRC chunks and zero-only dump");
}
'''
if len(sys.argv)>1:
    os.environ['DRIVER_TRACE_FIXTURE']=str(Path(sys.argv[1]).resolve())
    Path(sys.argv[1]).write_bytes(b'')
with tempfile.TemporaryDirectory(prefix='driver-trace-test-') as d:
    w=Path(d);(w/'driver_trace_host.h').write_text(header,encoding='utf-8');(w/'test.c').write_text(test,encoding='utf-8')
    exe=w/'trace.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-DDRIVER_TRACE_HOST','-I'+str(w),'-I'+str(root/'tools/cyt2bl3_trace/code'),str(w/'test.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=10)
