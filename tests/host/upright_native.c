/* Production app/control, output ISR, motor and full UART protocol. */
#include <assert.h>
#include <stdarg.h>
#include "app.c"
#include "balance_control.c"
#include "camera.c"
#include "posture_control.c"
#include "motor.c"
#include "small_driver_uart_control.c"
#include "safety_output.c"

volatile uint32 system_count;
volatile uint8 run_flag;
volatile float k1=CONTROL_K1_DEFAULT,k2=CONTROL_K2_DEFAULT;
volatile float wifi_target_speed=CONTROL_TARGET_SPEED_DEFAULT;
volatile float wifi_ramp_step=CONTROL_RAMP_REFERENCE/CONTROL_RAMP_MS_DEFAULT;
volatile float wifi_ramp_ms=CONTROL_RAMP_MS_DEFAULT,wifi_deadband_px;
volatile uint32 wifi_encoder_total;
int16 car_speed;
uint8 mt9v03x_finish_flag;
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W],base_image[MT9V03X_H][MT9V03X_W];
int16 imu660rb_acc_x=4098,imu660rb_acc_y,imu660rb_acc_z;
int16 imu660rb_gyro_x=7,imu660rb_gyro_y=-10,imu660rb_gyro_z=4;
static unsigned camera_calls,display_calls,telemetry_calls,samples,pwm_duty,direction_level,pwm_init_calls,dir_init_calls;
static unsigned wifi_polls,wifi_inits,wireless_calls;
static int16 last_left,last_right,encoder_count;
static int keys[4];
static uint8 imu_fail,release_cpu0,clock_ready,timers_ready,inside_output,uart_blocked;
static unsigned imu_init_calls,imu_failures_remaining,key_scans,status_logs,frozen_status_logs,start_refused_logs,boot_wifi_logs;
static unsigned delayed_boot_spins,diagnostic_writes,tx_frames,query_frames;
static uint32 host_stm_ticks,pwm_shadow,pwm_compare,shadow_since,dir_since;
static uint8 pwm_enabled;
static uint8 cas_contention_once;
static uint8 rx_frame[7],rx_pos,rx_size;
int MODULE_STM0;

static void advance_us(uint32 us)
{
    host_stm_ticks+=us;
    if((uint32)(host_stm_ticks-shadow_since)>=59u)pwm_compare=pwm_shadow;
    pwm_duty=pwm_enabled?pwm_compare:0;
}
static void output_tick(void)
{
    inside_output=1;safety_output_tick();inside_output=0;
}
float IfxStm_getFrequency(void *stm) {(void)stm;return 1000000.0f;}
uint32 IfxStm_getLower(void *stm) {(void)stm;return host_stm_ticks;}
uint32 host_compare_swap(unsigned int *address,uint32 value,uint32 condition)
{uint32 old;if(cas_contention_once){(*address)++;cas_contention_once=0;}old=*address;if(old==condition)*address=value;return old;}
uint8 wifi_link_online(void) {return 0;}
static void record_uart(const uint8 *data,uint32 length)
{
    unsigned i;uint8 sum=0;
    assert(length==7 && data[0]==0xA5);
    for(i=0;i<6;i++)sum+=data[i];assert(sum==data[6]);
    if(data[1]==1){last_left=(int16)((data[2]<<8)|data[3]);last_right=(int16)((data[4]<<8)|data[5]);tx_frames++;}
    else {assert(data[1]==2);query_frames++;}
}
void uart_write_buffer(int index,const uint8 *data,uint32 length)
{
    assert(clock_ready && data && length);
    if(index==DEBUG_UART_INDEX)diagnostic_writes++;
    else {assert(index==MOMENTUM_UART && !timers_ready);record_uart(data,length);}
}
uint8 uart_try_write_buffer(int index,const uint8 *data,uint32 length)
{assert(index==MOMENTUM_UART && inside_output);if(uart_blocked)return 0;record_uart(data,length);return 1;}
void uart_init(int index,uint32 baud,int tx,int rx)
{assert(index==MOMENTUM_UART&&baud==460800&&tx==MOMENTUM_UART_TX&&rx==MOMENTUM_UART_RX);}
void uart_rx_interrupt(int index,uint32 enabled) {assert(index==MOMENTUM_UART&&enabled);}
uint8 uart_query_byte(int index,uint8 *data)
{assert(index==MOMENTUM_UART);if(rx_pos>=rx_size)return 0;*data=rx_frame[rx_pos++];return 1;}
static void feed_speed(int16 left,int16 right,uint8 correct)
{
    unsigned i;rx_frame[0]=0xA5;rx_frame[1]=2;
    rx_frame[2]=(uint8)((uint16)left>>8);rx_frame[3]=(uint8)left;
    rx_frame[4]=(uint8)((uint16)right>>8);rx_frame[5]=(uint8)right;
    rx_frame[6]=0;for(i=0;i<6;i++)rx_frame[6]+=rx_frame[i];
    if(!correct)rx_frame[6]++;
    rx_pos=0;rx_size=7;for(i=0;i<7;i++)uart_control_callback();
}
void host_dsync(void)
{
    if(release_cpu0){assert(app_cpu1_stage==1&&!pwm_init_calls&&!dir_init_calls);release_cpu0=0;delayed_boot_spins++;app_cpu0_init();}
}
float func_limit_ab(float v,float low,float high) {return v<low?low:(v>high?high:v);}
float func_abs(float v) {return v<0?-v:v;}
void clock_init(void) {clock_ready=1;}
void debug_init(void) {}
void interrupt_global_enable(int p) {(void)p;}
void disable_Watchdog(void) {}
void pwm_init(int p,uint32 f,uint32 d)
{assert(clock_ready&&app_cpu0_ready&&p==MOTOR_PWM_PIN&&f==17000&&d==0);pwm_init_calls++;pwm_shadow=pwm_compare=pwm_duty=0;}
void pwm_set_duty(int p,uint32 d)
{assert(p==MOTOR_PWM_PIN&&d<=10000&&(!timers_ready||inside_output));pwm_shadow=d;shadow_since=host_stm_ticks;}
void pwm_output_enable(int p,uint8 enabled)
{
    assert(p==MOTOR_PWM_PIN&&(!timers_ready||inside_output));
    if(enabled){assert((uint32)(host_stm_ticks-dir_since)>=MOTOR_DIR_SETUP_US);assert(pwm_compare>0);}
    pwm_enabled=enabled;pwm_duty=enabled?pwm_compare:0;
}
void gpio_init(int p,int direction,int level,int mode)
{assert(p==MOTOR_DIR_PIN&&direction==GPO&&mode==GPO_PUSH_PULL&&!pwm_enabled);direction_level=level;dir_init_calls++;dir_since=host_stm_ticks;}
void gpio_set_level(int p,int level)
{assert(p==MOTOR_DIR_PIN&&inside_output&&!pwm_enabled&&!pwm_compare);direction_level=level;dir_since=host_stm_ticks;}
void encoder_dir_init(int a,int b,int c) {assert(a==TIM2_ENCODER&&b==TIM2_ENCODER_CH1_P33_7&&c==TIM2_ENCODER_CH2_P33_6);}
int16 encoder_get_count(int i) {(void)i;return encoder_count;}
void encoder_clear_count(int i) {(void)i;encoder_count=0;}
void key_init(uint32 p) {assert(p==APP_KEY_SCAN_PERIOD_MS);}
void key_scanner(void) {key_scans++;}
int key_get_state(int k) {return keys[k];}
void key_clear_all_state(void) {memset(keys,0,sizeof(keys));}
void wireless_uart_init(void) {wireless_calls++;}
void ips200_init(int i) {(void)i;display_calls++;}
uint8 mt9v03x_init(void) {camera_calls++;return 0;}
uint8 imu660rb_init(void)
{imu_init_calls++;if(imu_fail)return 1;if(imu_failures_remaining){imu_failures_remaining--;return 1;}return 0;}
void imu660rb_get_acc(void) {assert(app_imu_ready);samples++;}
void imu660rb_get_gyro(void) {}
void pit_ms_init(int c,uint32 p) {assert(p==1&&(c==CCU60_CH0||c==CCU61_CH0));if(c==CCU60_CH0)timers_ready=1;}
void system_delay_ms(uint32 ms) {if(!timers_ready)advance_us(ms*1000u);}
void system_delay_us(uint32 us) {advance_us(us);}
void send_two_floats(float a,float b,float c) {(void)a;(void)b;(void)c;}
void ips200_show_gray_image(int x,int y,const uint8*p,int w,int h,int dw,int dh,int t)
{(void)x;(void)y;(void)p;(void)w;(void)h;(void)dw;(void)dh;(void)t;display_calls++;}
void ips200_draw_point(int x,int y,int c) {(void)x;(void)y;(void)c;}
void ips200_draw_line(int x,int y,int a,int b,int c) {(void)x;(void)y;(void)a;(void)b;(void)c;}
void ips200_show_float(int x,int y,float v,int a,int b) {(void)x;(void)y;(void)v;(void)a;(void)b;}
void wifi_link_init(void) {wifi_inits++;}
void wifi_link_service(void) {wifi_polls++;}
void wifi_link_log(const char *format,...)
{char message[128];va_list args;va_start(args,format);vsnprintf(message,sizeof(message),format,args);va_end(args);if(strstr(message,"[BOOT]"))boot_wifi_logs++;}
void wifi_link_cpu1_log(const char *text)
{if(strstr(text,"[APP] st=")){status_logs++;if(strstr(text,"dtick=0 "))frozen_status_logs++;}if(strstr(text,"[APP] start refused"))start_refused_logs++;}
void wifi_link_cpu1_service(uint8 frame) {assert(!frame||APP_CAMERA_ENABLE);telemetry_calls++;}
float wifi_vision_erro(float v) {return v;}

static void boot(void)
{
    timers_ready=inside_output=uart_blocked=0;system_count=run_flag=0;host_stm_ticks=0;
    pwm_enabled=0;pwm_shadow=pwm_compare=pwm_duty=0;shadow_since=dir_since=0;
    fault_latch=stop_generation=processed_stop_epoch=start_epoch=0;
    start_pending=stop_pending=was_running=0;momentum_left_duty=momentum_right_duty=0;
    pwm_init_calls=dir_init_calls=0;app_cpu0_ready=clock_ready=0;
    release_cpu0=1;memset(keys,0,sizeof(keys));app_init();
    assert(app_cpu0_ready&&app_cpu1_stage==9&&!run_flag&&!pwm_duty&&!last_left&&!last_right);
}
static void step(void)
{
    advance_us(1000);
    if(system_count%20==0)feed_speed(0,0,1);
    pit_isr_callback();output_tick();
}
static void warmup(void) {unsigned i;for(i=0;i<2000;i++)step();}
static void publish_manual(int16 left,int16 right,int travel)
{
    run_flag=1;bujin_motor_contron(travel);
    safety_output_publish(left,right,motor_requested_output(),safety_output_epoch());
    output_tick();
}
static void settle_manual(int16 left,int16 right,int travel)
{unsigned i;for(i=0;i<4;i++){advance_us(1000);feed_speed(0,0,1);publish_manual(left,right,travel);}advance_us(1000);}

static void test_pid(void)
{
    pid_cycle_struct p;
    unsigned i;
    memset(&p,0,sizeof(p));p.p=2;p.out_max=100;
    pid_control_incremental(&p,3,0);assert(p.out==6);
    pid_control_incremental(&p,3,0);assert(p.out==6);
    pid_control_incremental(&p,1,0);assert(p.out==2);
    memset(&p,0,sizeof(p));p.d=1;p.out_max=100;
    pid_control_incremental(&p,1,0);assert(p.out==1);
    pid_control_incremental(&p,1,0);assert(p.out==0);
    memset(&p,0,sizeof(p));p.i=.023f;p.out_max=100;
    for(i=0;i<100;i++)pid_control_incremental(&p,1,0);
    assert(fabs(p.out-2.3f)<.0001);
    memset(&p,0,sizeof(p));p.p=10000;p.out_max=10000;
    pid_control(&p,10,0);assert(p.out==10000);
}

static void test_frozen_tick_diagnostics(void)
{
    unsigned i, scans=key_scans, frames=telemetry_calls, logs=status_logs;
    unsigned zero_logs=frozen_status_logs;
    uint32 saved_ticks, saved_last, before_ms;
    assert(system_count==0 && !run_flag);
    for(i=1;i<=2000;i++)
    {
        host_stm_ticks+=1000u;
        app_buttons_poll();app_telemetry_poll();app_status_log_poll();
    }
    assert(system_count==0 && !run_flag && !pwm_duty && !last_left && !last_right);
    assert(key_scans-scans==2000/APP_KEY_SCAN_PERIOD_MS);
    assert(telemetry_calls-frames==2000/APP_TELEMETRY_PERIOD_MS);
    assert(status_logs-logs==2000/APP_STATUS_LOG_PERIOD_MS);
    assert(frozen_status_logs-zero_logs==status_logs-logs);
    assert(!control_request_start()); /* Background diagnostics cannot bypass warmup. */
    keys[KEY_1]=KEY_SHORT_PRESS;
    host_stm_ticks+=APP_KEY_SCAN_PERIOD_MS*1000u;
    app_buttons_poll();
    assert(!keys[KEY_1] && start_refused_logs && !run_flag && !start_pending);
    /* Lower STM word wraps independently of the stalled control tick. */
    before_ms=app_cpu1_now_ms();saved_ticks=host_stm_ticks;saved_last=cpu1_last_ticks;
    cpu1_last_ticks=0xffffffffu-500u;host_stm_ticks=499u;
    assert(app_cpu1_now_ms()==before_ms+1u && !system_count);
    host_stm_ticks=saved_ticks;cpu1_last_ticks=saved_last;
    puts("PASS: frozen PIT keeps timed telemetry, status and key scan; start remains protected; STM wrap");
}


int main(void)
{
    unsigned i,before;uint32 saved_epoch;float gain;
    test_pid();boot();app_cpu0_poll();assert(boot_wifi_logs&&diagnostic_writes);
    test_frozen_tick_diagnostics();
#ifdef HOST_TIMING_VARIANT
    puts("PASS: independent background period overrides");return 0;
#endif
    boot();warmup();
    assert(!run_flag&&!pwm_duty&&!last_left&&!last_right&&safety_output_can_start());
#if APP_DRIVE_ENABLE
    /* The actual local button -> request -> CPU0 control -> CPU1 output route. */
    advance_us(APP_KEY_SCAN_PERIOD_MS*1000u);feed_speed(0,0,1);
    pit_isr_callback();keys[KEY_1]=KEY_SHORT_PRESS;app_buttons_poll();
    assert(start_pending&&!run_flag);step();assert(run_flag&&!keys[KEY_1]);
    saved_epoch=safety_output_epoch();control_request_stop();
    output_tick();assert(!pwm_duty&&!last_left&&!last_right);
    pit_isr_callback();start_epoch=saved_epoch;start_pending=1;step();assert(!run_flag);

    /* Staging must never issue hardware writes from the control context. */
    boot();
    for(i=0;i<2000;i++){advance_us(1000);pit_isr_callback();output_tick();}
#if CONTROL_ROLL_ENABLE || CONTROL_YAW_ENABLE
    assert(!control_request_start()&&!safety_output_feedback_ok());
#endif
    feed_speed(0,0,1);assert(safety_output_feedback_ok());
    warmup();gain=balance_cascade.angular_speed_cycle.i;
    balance_cascade.cascade_value.filtering_angle=PROTECT_ROLL_RAW+1;
    run_flag=1;dynamic_value_control();output_tick();assert(!run_flag&&!pwm_duty);
    assert(balance_cascade.angular_speed_cycle.i==gain);
    balance_cascade.cascade_value.filtering_angle=0;
    balance_cascade.pitch_cascade_value.filtering_angle=0;
    settle_manual(-500,500,1000);
#if !CONTROL_ROLL_ENABLE && !CONTROL_YAW_ENABLE
    assert(!last_left&&!last_right);
#endif
#if !CONTROL_PITCH_ENABLE && !CONTROL_MOTOR_ENABLE
    assert(!pwm_enabled&&!pwm_duty);
#endif
#if CONTROL_PITCH_ENABLE || CONTROL_MOTOR_ENABLE
    assert(pwm_duty==1400&&bujin_last_duty==1400*MOTOR_DIRECTION_SIGN);
    assert(direction_level==(MOTOR_DIRECTION_SIGN>0?MOTOR_DIR_FORWARD_LEVEL:MOTOR_DIR_REVERSE_LEVEL));
    advance_us(1000);publish_manual(-500,500,-1000);assert(output_phase==1&&!pwm_enabled);
    control_request_stop();output_tick();assert(!pwm_enabled&&output_phase==0);
    advance_us(2000);output_tick();assert(!pwm_enabled); /* Old frame cannot resume after stop. */
    settle_manual(-500,500,1000);
    advance_us(1000);publish_manual(-500,500,-1000);
    advance_us(1000);publish_manual(-500,500,-1000);assert(output_phase==2&&!pwm_enabled);
    control_request_stop();output_tick();assert(!pwm_enabled&&output_phase==0);
    settle_manual(-500,500,1000);
    advance_us(1000);publish_manual(-500,500,-1000);assert(!pwm_enabled&&!pwm_duty);
    advance_us(20);publish_manual(-500,500,-1000);assert(!pwm_enabled);
    advance_us(1000);publish_manual(-500,500,-1000);assert(!pwm_enabled);
    advance_us(1000);publish_manual(-500,500,-1000);assert(pwm_enabled);
    advance_us(1000);assert(pwm_duty==1400&&bujin_last_duty==-1400*MOTOR_DIRECTION_SIGN);
#endif
    /* Stop while CPU0 never returns to its ISR. Only CPU1's output tick runs. */
    control_request_stop();output_tick();assert(!pwm_enabled&&!last_left&&!last_right);
    assert(!control_request_start());
    saved_epoch=safety_output_epoch();cas_contention_once=1;control_request_stop();
    assert(safety_output_epoch()==saved_epoch+2u); /* Concurrent stop tickets are not lost. */

    boot();warmup();settle_manual(-500,500,1000);
    for(i=0;i<=CONTROL_HEARTBEAT_TIMEOUT_MS;i++){advance_us(1000);system_count++;output_tick();}
    assert(safety_output_faults()&SAFETY_FAULT_CONTROL_TIMEOUT);
    assert(!run_flag&&!pwm_enabled&&!last_left&&!last_right&&!control_request_start());

    boot();warmup();settle_manual(-500,500,1000);
    for(i=0;i<=MOMENTUM_FEEDBACK_TIMEOUT_MS;i++)
    {advance_us(1000);safety_output_publish(-500,500,motor_requested_output(),safety_output_epoch());output_tick();}
#if CONTROL_ROLL_ENABLE || CONTROL_YAW_ENABLE
    assert(safety_output_faults()&SAFETY_FAULT_FEEDBACK);
    assert(!run_flag&&!pwm_enabled&&!last_left&&!last_right);
    feed_speed(0,0,1);assert(!control_request_start()); /* Fault cannot self-rearm. */
#else
    assert(!(safety_output_faults()&SAFETY_FAULT_FEEDBACK));
#endif

    boot();warmup();settle_manual(-500,500,1000);uart_blocked=1;
    for(i=0;i<=MOTOR_UART_TX_TIMEOUT_MS+1;i++)
    {advance_us(1000);safety_output_publish(-500,500,motor_requested_output(),safety_output_epoch());output_tick();}
    assert(safety_output_faults()&SAFETY_FAULT_UART_TX);assert(!run_flag&&!pwm_enabled);
    uart_blocked=0;output_tick();assert(!last_left&&!last_right);

    boot();warmup();settle_manual(-500,500,1000);
    mailbox.sequence|=1u; /* Producer hangs in the middle of publishing. */
    for(i=0;i<=CONTROL_HEARTBEAT_TIMEOUT_MS;i++){advance_us(1000);output_tick();}
    assert(safety_output_faults()&SAFETY_FAULT_CONTROL_TIMEOUT);assert(!pwm_enabled&&!last_left&&!last_right);
#else
    assert(!control_request_start());
#endif
    /* Freshness comes only from complete, checksum-valid feedback frames. */
    boot();feed_speed(-123,456,1);assert(motor_value.receive_left_speed_data==-123&&motor_value.receive_right_speed_data==456);
    before=feedback_ticks;advance_us(2000);feed_speed(789,-987,0);assert(feedback_ticks==before);
    assert(motor_value.receive_left_speed_data==-123);
    advance_us(MOMENTUM_FEEDBACK_TIMEOUT_MS*1000u);assert(!small_driver_feedback_fresh(host_stm_ticks,1000));
    feed_speed(0,0,1);assert(small_driver_feedback_fresh(host_stm_ticks,1000));
    host_stm_ticks=0xfffff000u;feed_speed(0,0,1);advance_us(10000);
    assert(small_driver_feedback_fresh(host_stm_ticks,1000));
    advance_us(MOMENTUM_FEEDBACK_TIMEOUT_MS*1000u);assert(!small_driver_feedback_fresh(host_stm_ticks,1000));

    /* Positive, negative and non-finite filter state must fault, not look level. */
    for(i=0;i<4;i++)
    {
        boot();balance_cascade.cascade_value.angle_temp=i==0?6000.0f:(i==1?-6000.0f:(i==2?(float)HUGE_VAL:0.0f/0.0f));
        first_order_complementary_filtering(&balance_cascade.cascade_value,0,i==1?-6000:6000);
        assert(safety_output_faults()&SAFETY_FAULT_FILTER);
        assert(func_abs(balance_cascade.cascade_value.filtering_angle)==FILTER_STATE_LIMIT_RAW);
        output_tick();assert(!run_flag&&!pwm_enabled&&!last_left&&!last_right&&!control_request_start());
    }
    boot();imu_fail=1;before=imu_init_calls;timers_ready=0;app_init();
    assert(imu_init_calls-before==APP_IMU_INIT_ATTEMPTS&&!app_imu_ready&&!control_request_start());
    before=samples;for(i=0;i<25;i++)step();assert(samples==before);
    imu_fail=0;imu_failures_remaining=2;before=imu_init_calls;timers_ready=0;app_init();
    assert(app_imu_ready&&imu_init_calls-before==3&&!run_flag&&!pwm_enabled);
    puts("PASS: independent CPU1 output, real UART protocol/freshness, stopped CPU0, latched faults and physical PWM gating/reversal model");
    return 0;
}
