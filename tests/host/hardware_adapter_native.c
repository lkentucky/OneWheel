/* Compile the actual two SDK wrapper bodies against register-level stubs. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef int uart_index_enum;
typedef int pwm_channel_enum;
typedef struct {int asclin_value;int *asclin;} IfxAsclin_Asc;
typedef struct {struct {int *port;int pinIndex;} pin;int select;} IfxGtm_Atom_ToutMap;
static int port_value;
static IfxAsclin_Asc handle;
static IfxGtm_Atom_ToutMap mapping;
static uint8 fifo_fill;
static unsigned writes,port_event;
static int forced_low,pin_mode;
enum {IfxPort_OutputMode_pushPull=1,IfxPort_OutputIdx_general=0};
static IfxAsclin_Asc *uart_get_handle(int n) {assert(n==3);return &handle;}
static uint8 IfxAsclin_getTxFifoFillLevel(int *p) {(void)p;return fifo_fill;}
static uint32 IfxAsclin_write8(int *p,const uint8 *data,uint32 n)
{(void)p;assert(data&&fifo_fill+n<=16);fifo_fill+=(uint8)n;writes++;return 0;}
static IfxGtm_Atom_ToutMap *get_pwm_pin(int n) {assert(n==215);return &mapping;}
static void IfxPort_setPinLow(int *port,int pin) {assert(port==&port_value&&pin==5);forced_low=1;port_event=1;}
static void IfxPort_setPinModeOutput(int *port,int pin,int mode,int select)
{assert(port==&port_value&&pin==5&&mode==1);if(!select)assert(port_event==1&&forced_low);pin_mode=select;port_event=2;}
#include "hardware_adapter_functions.h"
int main(void)
{
    uint8 data[7]={0};
    handle.asclin=&handle.asclin_value;
    mapping.pin.port=&port_value;mapping.pin.pinIndex=5;mapping.select=6;
    fifo_fill=9;assert(uart_try_write_buffer(3,data,7)&&fifo_fill==16&&writes==1);
    assert(!uart_try_write_buffer(3,data,1)&&writes==1);
    fifo_fill=10;assert(!uart_try_write_buffer(3,data,7)&&fifo_fill==10&&writes==1);
    assert(!uart_try_write_buffer(3,0,7)&&!uart_try_write_buffer(3,data,0)&&!uart_try_write_buffer(3,data,17));
    pin_mode=6;pwm_output_enable(215,0);assert(pin_mode==0&&forced_low);
    pwm_output_enable(215,1);assert(pin_mode==6);
    puts("PASS: actual SDK adapters: FIFO capacity is all-or-nothing; PWM pin low latch precedes GPIO override");
    return 0;
}
