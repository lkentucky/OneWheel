"""Exercise real TC264 UART trace queue and long-response demultiplexing."""
from pathlib import Path
import ast,subprocess,tempfile
r=Path(__file__).resolve().parents[1]
t=ast.parse((r/'tests/check_pwm_uart.py').read_text('utf-8-sig'))
boundary=next(ast.literal_eval(n.value) for n in t.body if isinstance(n,ast.Assign) and any(isinstance(a,ast.Name) and a.id=='boundary' for a in n.targets))
test=r'''
#include "small_driver_uart_control.h"
#include <assert.h>
#include <stdio.h>
static uint8 rx[4096],tx[4096];static unsigned rp,rn,tn;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void uart_init(int p,uint32 b,int tx,int rx){(void)p;(void)b;(void)tx;(void)rx;}
void uart_rx_interrupt(int p,int on){(void)p;(void)on;}
uint8 uart_query_byte(int p,uint8 *b){(void)p;if(rp==rn)return 0;*b=rx[rp++];return 1;}
void uart_write_buffer(int p,const uint8 *b,uint32 n){(void)p;assert(tn+n<4096);memcpy(tx+tn,b,n);tn+=n;tx[tn]=0;}
static void feed(const char *s){rn=(unsigned)strlen(s);rp=0;memcpy(rx,s,rn);while(rp<rn)uart_control_callback();}
int main(void){
    char line[1536],out[1600];
    small_driver_uart_init();tn=0;
    assert(small_driver_trace_request(65534));assert(!small_driver_trace_request(65535));
    small_driver_set_duty(0,0);assert(strstr((char*)tx,"SET-TRACE,1\n"));
    feed("0,0\r\n");uint32 frames=small_driver_speed_frames;
    strcpy(line,"TR,D,123,0,8;");memset(line+strlen(line),'1',1100-strlen(line));strcpy(line+1100,"*1234\n");
    feed(line);assert(small_driver_speed_frames==frames);
    assert(!small_driver_trace_request(65535));
    assert(small_driver_trace_take(out,sizeof(out))==(int)strlen(line)-1);
    assert(!memcmp(out,line,strlen(line)-1) && out[strlen(line)-1]==0);
    assert(!small_driver_trace_take(out,sizeof out));
    assert(small_driver_trace_request(65535));tn=0;small_driver_set_duty(0,0);
    assert(strstr((char*)tx,"GET-TRACE,65535\n"));
    feed("123,-456\r\n");assert(small_driver_speed_frames==frames+1);
    assert(motor_value.receive_left_speed_data==123 && motor_value.receive_right_speed_data==-456);
    assert(!small_driver_trace_request(65536));
    memset(line,'x',sizeof(line)-1);line[sizeof(line)-1]=0;feed(line);feed("xxxxxxxxxxxxxxxxxxxxxx\n0,0\n");
    assert(motor_value.receive_left_speed_data==0);
    assert(small_driver_trace_request(65533));tn=0;small_driver_set_duty(0,0);assert(strstr((char*)tx,"SET-TRACE,0\n"));
    puts("PASS: real TC264 UART queue, ISR-owned TX, >1KB trace relay, bounded overflow and normal speed feedback");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-trace-bridge-') as d:
    w=Path(d);(w/'zf_common_headfile.h').write_text(boundary,encoding='utf-8');(w/'test.c').write_text(test,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(w),'-I'+str(r/'code'),str(r/'code/small_driver_uart_control.c'),str(w/'test.c'),'-o',str(w/'test.exe')],check=True)
    subprocess.run([str(w/'test.exe')],check=True,timeout=5)
