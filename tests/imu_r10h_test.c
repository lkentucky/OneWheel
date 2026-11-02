#include "imu.h"
#include "config.h"
#include "attitude_algorithm.h"
#include "zf_device_imu660rb.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int host_stm;
static uint32 ticks;
static uint8 registers[128],ready=3,cs=1,init_result;
static int irq,fail_spi,bad_filter_readback;
static int16 raw_a[3]={4098,0,0},raw_g[3]={14,-28,42};
int16 imu660rb_acc_x,imu660rb_acc_y,imu660rb_acc_z;
int16 imu660rb_gyro_x,imu660rb_gyro_y,imu660rb_gyro_z;
float imu660rb_transition_factor[2]={4098,14.3f};
uint32 IfxStm_getLower(int *s) {(void)s; return ticks;}
float IfxStm_getFrequency(int *s) {(void)s; return 100000000.0f;}
uint8 imu660rb_init(void) {registers[0x12]=0x44;registers[0x0f]=0x6b; return init_result;}
void gpio_high(int p) {assert(p==13 && !cs);cs=1;}
void gpio_low(int p) {assert(p==13 && cs);cs=0;}
void system_delay_ms(uint32 ms) {ticks+=100000u*ms;}
uint32 interrupt_global_disable(void) {assert(!irq);irq=1;return 0;}
void interrupt_global_enable(uint32 p) {assert(irq && !p);irq=0;}
void spi_write_8bit_register(int s,uint8 r,uint8 v) {assert(s==0 && !cs && r<128);registers[r]=v;}
uint8 spi_read_8bit_register(int s,uint8 r) {assert(s==0 && !cs && (r&128));r&=127;return r==0x1e?ready:(registers[r]^((bad_filter_readback && r==0x15)?1:0));}
void spi_read_8bit_registers(int s,uint8 r,uint8 *out,uint32 n) {
    assert(s==0 && !cs && r==0xa2 && n==12);
    for(unsigned i=0;i<3;++i) {
        out[2*i]=(uint8)raw_g[i];out[2*i+1]=(uint8)((uint16)raw_g[i]>>8);
        out[6+2*i]=(uint8)raw_a[i];out[7+2*i]=(uint8)((uint16)raw_a[i]>>8);
    }
}
uint8 spi_transfer_8bit_timeout(int spi,const uint8 *tx,uint8 *rx,uint32 n,uint32 timeout) {
    assert(spi==0 && !cs && timeout==1);
    if(fail_spi)return 1;
    if(n==2) {
        if(tx[0]&128) {assert(rx);rx[0]=0;rx[1]=spi_read_8bit_register(spi,tx[0]);}
        else spi_write_8bit_register(spi,tx[0],tx[1]);
    } else {assert(n==13 && rx);rx[0]=0;spi_read_8bit_registers(spi,tx[0],rx+1,12);}
    return 0;
}
static void tick(void) {ticks+=100u*APP_IMU_PERIOD_US;imu_update();assert(cs);}

uint8 r10h_host_bus_read(uint8 reg,uint8 *data,uint8 n)
{
 uint8 tx[13]={0},rx[13];uint8 failed;assert(n<=12);tx[0]=reg|128;
 IMU660RB_CS(0);failed=spi_transfer_8bit_timeout(0,tx,rx,n+1,1);IMU660RB_CS(1);
 if(failed)return 0;
 memcpy(data,rx+1,n);return 1;
}
static void ready_core(void){for(unsigned i=0;i<CALIBRATION_SAMPLES+ASC_ATT_READY_SAMPLES+3u && imu_attitude_status==0;++i)tick();assert(imu_attitude_status==1);}
int main(void){imu_navigation_sample_t frame;uint32 seq;
 ticks=0xfff00000u;imu_init();assert(imu_attitude_method==5 && imu_attitude_status==0);
 assert(registers[0x10]==0x8c&&registers[0x11]==0x8c);
 raw_g[1]=1000;for(int i=0;i<20;++i)tick();assert(imu_calibration_percent==0);
 raw_g[1]=-28;for(unsigned i=0;i<CALIBRATION_SAMPLES;++i)tick();
 assert(imu_attitude_status==0);for(unsigned i=0;i<ASC_ATT_READY_SAMPLES-1u;++i)tick();assert(imu_attitude_status==0);
 tick();assert(imu_attitude_status==1);assert(imu_get_navigation_sample(&frame));
 assert(fabsf(attitude_state.gyro.x)<.01f&&fabsf(attitude_state.gyro.y)<.01f&&fabsf(attitude_state.gyro.z)<.01f);
 assert(fabsf(imu_asc_roll_deg)<.1f&&fabsf(imu_asc_pitch_deg)<.1f);
 raw_g[1]-=143;tick();assert(imu_get_navigation_sample(&frame));
 assert(fabsf(frame.gyro_dps[1]-10)<.01f&&fabsf(attitude_state.gyro.y-10)<.01f); /* no 2ms gyro LPF */
 for(int i=0;i<99;++i){tick();}
 assert(imu_asc_roll_deg>.5f&&imu_pitch_deg>.5f);
 raw_g[1]=-28;imu_request_recalibration();imu_service();ready_core();
 raw_g[2]-=143;for(int i=0;i<100;++i)tick();assert(imu_asc_pitch_deg>.5f&&imu_roll_deg<-.5f);
 raw_g[2]=42;imu_request_recalibration();imu_service();ready_core();
 seq=imu_sample_sequence;ticks+=100000u;tick();assert(imu_attitude_status==IMU_ATTITUDE_BAD_DT);
 for(int i=0;i<10;++i){tick();}
 assert(imu_sample_sequence==seq);assert(!imu_get_navigation_sample(&frame));
 imu_recover_after_stall();imu_service();assert(imu_attitude_status==0);ready_core();
 seq=imu_sample_sequence;ready=0;tick();assert(imu_attitude_status==IMU_ATTITUDE_TIMEOUT);
 ready=3;for(int i=0;i<10;++i){tick();}
 assert(imu_sample_sequence==seq);
 imu_prepare_start();assert(imu_attitude_status==0);ready_core();
 seq=imu_sample_sequence;raw_g[0]=32767;tick();assert(imu_attitude_status==IMU_ATTITUDE_TIMEOUT&&imu_gyro_clips>0);assert(imu_sample_sequence==seq);
 raw_g[0]=14;imu_request_recalibration();imu_service();ready_core();
 registers[0x0f]=0xff;for(int i=0;i<101;++i)tick();assert(imu_attitude_status==IMU_ATTITUDE_TIMEOUT);
 puts("PASS: real r10h pipeline, 6000+3000 gate, proper frame rotation, raw/control bias separation, no gyro LPF, 1ms timing, STM wrap, loss/clip/WHO fault latch and static restart");return 0;
}
