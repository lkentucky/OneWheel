"""Actual nonblocking UART3 driver: framing, latest-wins, zero, RX and watchdog."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def test_text_mailbox_and_binary_rpm(tmp_path):
 (tmp_path/'zf_common_headfile.h').write_text(r'''#ifndef HOST_HW
#define HOST_HW
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
typedef uint8_t uint8;typedef uint16_t uint16;typedef uint32_t uint32;typedef int16_t int16;typedef int32_t int32;typedef uint8_t boolean;
typedef struct {int dummy;} Ifx_SRC_SRCR;
extern struct host_uart {void *asclin;} uart3_handle;
#define UART_3 3
#define UART3_TX_P15_6 6
#define UART3_RX_P15_7 7
#define func_limit_ab(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))
uint32 interrupt_global_disable(void);void interrupt_global_enable(uint32);
void uart_init(int,uint32,int,int);void uart_rx_interrupt(int,int);uint8 uart_query_byte(int,uint8*);
int IfxAsclin_getTxFifoFillLevel(void*);void IfxAsclin_writeTxData(void*,uint8);
volatile Ifx_SRC_SRCR *IfxAsclin_getSrcPointerTx(void*);void IfxAsclin_enableTxFifoFillLevelFlag(void*,boolean);
void IfxSrc_enable(volatile Ifx_SRC_SRCR*);void IfxSrc_disable(volatile Ifx_SRC_SRCR*);void IfxAsclin_clearTxFifoFillLevelFlag(void*);
#endif
''')
 c=r'''#include "HW_Drivers/Motor/W_Motor.c"
#include <stdlib.h>
param_t g_param;
struct host_uart uart3_handle;
static Ifx_SRC_SRCR src;
static int irq,tx_enabled;static uint8 fifo[16],rx[512];static unsigned fill,rn,ri,written;static char wire[8192];static unsigned wn;
uint32 interrupt_global_disable(void){int old=irq;irq=1;return old;}void interrupt_global_enable(uint32 x){irq=(int)x;}
void uart_init(int n,uint32 baud,int tx,int r){assert(n==3&&baud==460800);(void)tx;(void)r;uart3_handle.asclin=&src;}
void uart_rx_interrupt(int n,int en){assert(n==3&&en);}
uint8 uart_query_byte(int n,uint8 *v){assert(n==3);if(ri==rn)return 0;*v=rx[ri++];return 1;}
int IfxAsclin_getTxFifoFillLevel(void *p){(void)p;return (int)fill;}
void IfxAsclin_writeTxData(void *p,uint8 v){(void)p;assert(fill<16&&irq);fifo[fill++]=v;++written;}
volatile Ifx_SRC_SRCR *IfxAsclin_getSrcPointerTx(void *p){(void)p;return &src;}
void IfxAsclin_enableTxFifoFillLevelFlag(void *p,boolean b){(void)p;(void)b;}
void IfxSrc_enable(volatile Ifx_SRC_SRCR *p){assert(p==&src);tx_enabled=1;}
void IfxSrc_disable(volatile Ifx_SRC_SRCR *p){assert(p==&src);tx_enabled=0;}
void IfxAsclin_clearTxFifoFillLevelFlag(void *p){(void)p;}
static void drain(void){for(unsigned i=0;i<10000;i++){
 if(fill){wire[wn++]=(char)fifo[0];memmove(fifo,fifo+1,--fill);assert(wn<sizeof(wire)-1);}
 else if(tx_enabled)W_Motor_TxHandler();else{wire[wn]=0;return;}
 }assert(!"TX did not finish");}
static void feed(const uint8 *b,unsigned n){assert(n<sizeof(rx));memcpy(rx,b,n);rn=n;ri=0;W_Motor_RxHandler();}
int main(void){
 g_param.motor_dir_a=g_param.motor_dir_b=1;g_param.fly_slew=0;W_Motor_Init();assert(written<=16);drain();
 assert(strstr(wire,"SET-DUTY,0,0\n")&&strstr(wire,"GET-SPEED\n"));assert(W_Motor_LinkLost());
 W_Motor_Release();wn=0;unsigned before=written;W_Motor_SetDuty(-4000,4000);assert(written-before<=16);drain();assert(!strcmp(wire,"SET-DUTY,-4000,4000\n"));
 wn=0;W_Motor_SetDuty(10000,-10000);for(int i=0;i<100;i++)W_Motor_SetDuty(i,-i);W_Motor_Stop();drain();
 assert(strstr(wire,"SET-DUTY,10000,-10000\n"));assert(strstr(wire,"SET-DUTY,0,0\n"));assert(!strstr(wire,"SET-DUTY,99,-99\n"));
 feed((const uint8*)"123,-456\n",9);W_Motor_Tick1ms();assert(W_Motor_GetSpeed1()==123&&W_Motor_GetSpeed2()==-456&&!W_Motor_LinkLost());
 uint32 frames=w_motor_rx_frames;feed((const uint8*)"bad,123\n",8);assert(w_motor_rx_frames==frames);
 uint8 bad[90];memset(bad,'1',sizeof(bad));bad[86]=',';bad[87]='2';bad[88]='\n';feed(bad,89);assert(w_motor_rx_frames==frames);
 uint8 packet[7]={0xa5,2,0x0a,0xf0,0xf5,0x10,0};for(int i=0;i<6;i++)packet[6]+=packet[i];feed(packet,7);W_Motor_Tick1ms();assert(W_Motor_GetSpeed1()==2800&&W_Motor_GetSpeed2()==-2800);
 for(int i=0;i<W_MOTOR_LINK_TIMEOUT_MS;i++)W_Motor_Tick1ms();assert(W_Motor_LinkLost());
 puts("PASS: bounded UART3 TX, complete-line latest-wins/stop, strict overflow/malformed text RX, binary LF payload and watchdog");return 0;}
'''
 (tmp_path/'test.c').write_text(c);exe=tmp_path/'driver.exe'
 r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-unknown-pragmas','-I'+str(tmp_path),'-I'+str(ROOT/'code'),str(tmp_path/'test.c'),'-o',str(exe)],capture_output=True,text=True)
 assert r.returncode==0,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=10);assert r.returncode==0,r.stdout+r.stderr
