"""E2E: real controllers/UART -> stock and fixed CYT2BL3 parsers, with host hardware boundaries."""
from pathlib import Path
import ast,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
def read(p):
    data=p.read_bytes()
    try:return data.decode('utf-8-sig')
    except UnicodeDecodeError:return data.decode('gbk')
def function(s,signature):
    a=s.index(signature);b=s.index('{',a)+1;depth=1
    while depth:depth+=(s[b]=='{')-(s[b]=='}');b+=1
    return s[a:b]+'\n'
assignments=ast.parse(read(root/'tests/check_pwm_uart.py'))
boundary=next(ast.literal_eval(n.value) for n in assignments.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='boundary' for t in n.targets))
prefix = r'''
#include "balance_control.h"
#include "small_driver_uart_control.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint16_t uint16;
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms=5;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status=1;
#define FIFO_READ_ONLY 0
#define FIFO_READ_AND_CLEAN 1
#define MOTOR_DRIVER_UART 2
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
'''
glue = r'''
typedef struct {uint8 data[128];uint32 used;} host_fifo;
static host_fifo motor_driver_fifo;
static uint8 read_buffer[128];static int16 receive_enter_flag;
static vendor_device device;
static int16 applied_left,applied_right;
static unsigned duty_frames,speed_requests,other_commands;
static uint8 driver_input;static int driver_available;
static uint8 host_rx[256];static unsigned host_rx_index,host_rx_count;
static uint8 last_wire[32];static int ground;
static uint32 fifo_used(host_fifo *f){return f->used;}
static void fifo_read_buffer(host_fifo *f,uint8 *out,uint32 *n,int clean){
    if(*n>f->used)*n=f->used;memcpy(out,f->data,*n);
    if(clean){memmove(f->data,f->data+*n,f->used-*n);f->used-=*n;}
}
static void fifo_write_element(host_fifo *f,uint8 v){assert(f->used<128);f->data[f->used++]=v;}
static int func_str_to_int(char *s){return atoi(s);}
static float func_limit_ab(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
static void driver_cmd_forthwith(void){
    if(device.refresh_flag==CMD_FORTHWITH && device.immediate_command==SET_DUTY){
        applied_left=device.receive_left_duty_data;applied_right=device.receive_right_duty_data;++duty_frames;
    }
    if(device.continue_command==GET_SPEED)speed_requests=1;
    if(device.immediate_command!=NULL_CMD && device.immediate_command!=SET_DUTY)++other_commands;
    device.immediate_command=NULL_CMD;device.refresh_flag=CMD_NULL;
}
void motor_driver_control_callback(vendor_device *d);
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void motor_m_ctrl(int n){ground=n;}
uint8 uart_query_byte(int port,uint8 *b){
    if(port==2){if(!driver_available)return 0;*b=driver_input;driver_available=0;return 1;}
    if(host_rx_index==host_rx_count)return 0;*b=host_rx[host_rx_index++];return 1;
}
static void feed(const uint8 *data,unsigned size){
    for(unsigned i=0;i<size;++i){driver_input=data[i];driver_available=1;motor_driver_control_callback(&device);}
}
void uart_write_buffer(int port,const uint8 *data,uint32 size){
    assert(port==3 && size<=32);memcpy(last_wire,data,size);feed(data,size);
}
void uart_init(int p,uint32 b,int tx,int rx){(void)p;(void)b;(void)tx;(void)rx;}
void uart_rx_interrupt(int p,int on){(void)p;(void)on;}
static void reset_driver(void){
    memset(&device,0,sizeof device);memset(&motor_driver_fifo,0,sizeof motor_driver_fifo);
    receive_enter_flag=0;applied_left=applied_right=0;duty_frames=speed_requests=other_commands=0;
}
static void tick(void){++imu_sample_sequence;for(unsigned i=0;i<APP_CONTROL_DIVIDER;++i){small_driver_tick_1ms();balance_control_tick_1ms();}}
'''
main = r'''
static void feedback(const uint8 *b,unsigned n){
    memcpy(host_rx,b,n);host_rx_index=0;host_rx_count=n;
    while(host_rx_index<host_rx_count)uart_control_callback();
}
int main(void){
    small_driver_uart_init();
    for(int mode=3;mode<=8;++mode){
        if(mode==4 || mode==7)continue;
        reset_driver();balance_control_defaults();shacheflag=1;control_startup_ramp_ms=0;
        imu_pitch_deg=3;imu_roll_deg=BALANCE_ZERO_X_DEG;attitude_state.gyro.x=attitude_state.gyro.y=0;
        if(mode==3)roll_rate_test_arm();else if(mode==5)dual_balance_arm();else if(mode==6)roll_rate_only_arm();else pwm_debug_arm();
        if(mode==8){pwm_debug_fly_left=-2800;pwm_debug_fly_right=2800;}
        for(unsigned i=0;i<200;++i)tick();
        assert(!control_fault_reason && roll_rate_test_state==2 && duty_frames==200);
        assert(applied_left==(mode==6?0:mode==8?2800:(int)(FLY_RATE_KP*FLY_RATE_LIMIT_DPS+FLY_ANGLE_FF_KP*3.5f)) && applied_right==-applied_left);
        printf("Mode%d: 200/200 applied commands, no unexpected protection/calibration command\n",mode);
    }
    reset_driver();balance_control_defaults();shacheflag=1;imu_pitch_deg=3;roll_rate_test_arm();
    for(unsigned i=0;i<1000000u/APP_IMU_PERIOD_US;++i)tick();
    assert(duty_frames==1000000u/APP_IMU_PERIOD_US && applied_left==(int)(FLY_RATE_KP*FLY_RATE_LIMIT_DPS+FLY_ANGLE_FF_KP*3.5f) && applied_right==-applied_left);
    puts("Default startup ramp reaches PID + angle compensation without a stale duty hold");
    for(int left=-4000;left<=4000;++left){
        int right=((left+4000)*37)%8001-4000;
        small_driver_set_duty((int16)left,(int16)right);
        assert(applied_left==-left && applied_right==-right);
    }
    puts("8001 independent signed command pairs including the entire binary-LF band accepted");
    /* Driver reset only: main ready state is preserved. */
    reset_driver();
    for(unsigned i=0;i<QCAR_DRIVER_SPEED_QUERY_MS+1u;++i){small_driver_tick_1ms();small_driver_set_duty(0,0);}
    assert(device.continue_command==GET_SPEED && speed_requests);
    /* One interrupt drains a complete frame; newline in binary payload is legal. */
    uint8 packet[7]={0xA5,2,0x0A,0xF0,0xF5,0x10,0};
    for(unsigned i=0;i<6;++i)packet[6]+=packet[i];
    memcpy(host_rx,packet,7);host_rx_index=0;host_rx_count=7;
    uint32 previous=small_driver_speed_frames;
    uart_control_callback();
    assert(host_rx_index==7 && small_driver_speed_frames==previous+1);
    assert(motor_value.receive_left_speed_data==2800 && motor_value.receive_right_speed_data==-2800);
    feedback((const uint8*)"-32768,32767\r\n",14);
    assert(motor_value.receive_left_speed_data==-32768 && motor_value.receive_right_speed_data==32767);
    previous=small_driver_speed_frames;
    const char *bad[]={"1,2garbage\n","32768,0\n","1,\n",",2\n","999999999999999999999999999999999,0\n"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i)feedback((const uint8*)bad[i],strlen(bad[i]));
    assert(previous==small_driver_speed_frames);
    feedback((const uint8*)"-123,456\n",9);
    assert(small_driver_feedback_fresh && small_driver_feedback_age_ms==0);
    for(unsigned i=0;i<QCAR_DRIVER_FEEDBACK_TIMEOUT_MS;++i)small_driver_tick_1ms();
    assert(small_driver_feedback_fresh);
    small_driver_tick_1ms();assert(!small_driver_feedback_fresh);
    for(unsigned i=0;i<65536;++i)small_driver_tick_1ms();
    assert(!small_driver_feedback_fresh && small_driver_feedback_age_ms>65536u);
    feedback((const uint8*)"0,0\n",4);
    assert(small_driver_feedback_fresh && small_driver_feedback_age_ms==0);
    /* Invalid checksum + next valid packet resynchronizes without interpreting binary LF as text. */
    uint8 combined[10]={0xA5,2,0};memcpy(combined+3,packet,7);
    feedback(combined,10);
    assert(motor_value.receive_left_speed_data==2800 && motor_value.receive_right_speed_data==-2800);
#if !STOCK_DRIVER
    reset_driver();
    uint8 binary[7]={0xA5,1,0x0A,0xF0,0xF5,0x10,0};
    for(unsigned i=0;i<6;++i)binary[6]+=binary[i];
    for(unsigned j=0;j<2;++j)for(unsigned i=0;i<7;++i)fifo_write_element(&motor_driver_fifo,binary[i]);
    motor_driver_control_loop(&device);
    assert(duty_frames==2 && applied_left==2800 && applied_right==-2800);
    /* Invalid long numeric text must neither overrun nor update PWM. */
    feed((const uint8*)"SET-DUTY,999999999999,1\n",24);
    assert(duty_frames==2);
    feed((const uint8*)"SET-DUTY,1,2\n",13);assert(applied_left==1 && applied_right==2);
#endif
    assert(!other_commands);
    puts("PASS: reset query recovery, bounded text/binary RX, stale feedback, malformed input and directions");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-driver-link-') as directory:
    work=Path(directory);(work/'zf_common_headfile.h').write_text(boundary,encoding='utf-8')
    for stock,text in [(True,1),(False,1),(False,0)]:
        folder=root/('tests/fixtures/cyt2bl3_stock' if stock else 'tools/cyt2bl3_driver/code')
        driver=read(folder/('motor_driver_uart_control.c.in' if stock else 'motor_driver_uart_control.c'))
        header=read(folder/'motor_driver_uart_control.h')
        types=header[header.index('typedef enum'):header.index('extern small_device_value_struct')].replace('small_device_value_struct','vendor_device')
        names=['void motor_driver_fifo_clear(','void motor_driver_parse_statement(','void motor_driver_control_loop(','void motor_driver_control_callback(']
        if not stock:names.insert(0,'static uint8 motor_driver_duty_number(')
        actual=''.join(function(driver,n) for n in names).replace('small_device_value_struct','vendor_device')
        (work/'test.c').write_text(prefix+types+glue+actual+main,encoding='utf-8')
        exe=work/'test.exe'
        subprocess.run(['gcc','-std=c99','-DQCAR_DRIVER_TEXT_DUTY='+str(text),'-DSTOCK_DRIVER='+str(int(stock)),'-I'+str(work),'-I'+str(root/'code'),str(root/'code/balance_control.c'),str(root/'code/pid.c'),str(root/'code/small_driver_uart_control.c'),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
        print('PARSER', 'stock' if stock else 'fixed', 'TX', 'text' if text else 'binary',flush=True)
        subprocess.run([str(exe)],check=True,timeout=15)
