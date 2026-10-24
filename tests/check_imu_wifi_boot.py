"""Actual boot entry: IMU timer before WiFi, auto stream/retry, no control timer or nonzero motor output."""
from pathlib import Path
import re
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
s=(root/"user/cpu0_main.c").read_text("utf-8")
s=re.sub(r'^#include.*$', '', s, flags=re.M)
s=s.replace('int core0_main(void)','int firmware_main(void)')
prefix=r"""
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <setjmp.h>
#include <assert.h>
#include "qcar_config.h"
typedef uint32_t uint32; typedef int32_t int32;
#define TRUE 1
#define IPS200_PORTAIT 0
#define RGB565_WHITE 1
#define RGB565_BLACK 0
#define IPS200_TYPE_SPI 0
#define IPS200_6X8_FONT 0
#define IMU_ATTITUDE_INIT_FAILED -1
#define CCU60_CH1 1
static jmp_buf done;
static int MODULE_STM0;
static uint32 ticks,imu_sample_sequence,imu_wifi_init_attempts,imu_wifi_tx_packets;
static uint32 wifi_telemetry_channel_count=3,wifi_telemetry_stream_enabled=1;
static uint32 wifi_spi_config_command,wifi_spi_config_error,wifi_spi_config_reply,wifi_spi_config_reply_length;
static float imu_roll_deg,imu_pitch_deg,imu_yaw_deg,imu_calibration_percent;
static int32 imu_attitude_status;
static int imu_wifi_status,imu_wifi_last_error,imu_wifi_module_result;
static char wifi_spi_version[12]="V2.0.1",wifi_spi_ip_addr_port[25]="192.168.0.120:5001";
static int cam_flag,tele_flag,turn_flag,shacheflag,key1_count;
static int imu_calls,imu_timer,net_calls,ready,announcements,retry_test,imu_bad;
static int ticks_in_menu,auto_menu_calls;
static void (*hook)(void);
static void clock_init(void) {}
static void debug_init(void) {}
static void ips200_set_dir(int x) {(void)x;}
static void ips200_set_color(int x,int y) {(void)x;(void)y;}
static void ips200_init(int x) {(void)x;}
static void ips200_set_font(int x) {(void)x;}
static void ips200_show_string(int x,int y,const char *s) {(void)x;(void)y;(void)s;}
static uint32 interrupt_global_disable(void) {return 0;}
static void interrupt_global_enable(uint32 x) {(void)x;}
static void motor_int(void) {}
static void motor_m_ctrl(int x) {assert(x==0);}
static void small_driver_uart_init(void) {}
static void small_driver_set_duty(int x,int y) {assert(!x && !y);}
static void roll_rate_test_stop(void) {}
static void imu_init(void) {imu_calls++;imu_attitude_status=imu_bad?-1:0;}
static void cpu_init_notify(void) {}
static void cpu_wait_event_ready(void) {}
static void pit_init(int ch,int us) {assert(imu_calls && ch==CCU60_CH1 && us==5000);imu_timer++;}
static float IfxStm_getFrequency(int *s) {(void)s;return 1000;}
static uint32 IfxStm_getLower(int *s) {(void)s;return ticks;}
static void imu_service(void) {if(imu_timer){imu_sample_sequence++;imu_attitude_status=1;}}
static void wifispi_set_progress_hook(void (*p)(void)) {hook=p;}
static void wifispi_telemetry_init(void) {
    assert(imu_calls && (imu_bad || imu_timer));net_calls++;imu_wifi_init_attempts++;
    imu_wifi_status=2;if(hook)hook();
    ready=!(retry_test && net_calls==1);imu_wifi_status=ready?1:-2;
}
static int wifispi_is_ready(void) {return ready;}
static void wifispi_telemetry_announce(void) {assert(ready);announcements++;}
static void wifispi_telemetry_service(void) {if(ready)imu_wifi_tx_packets++;}
static void system_delay_ms(int ms) {
    ticks+=(uint32)ms;
    if(ready && imu_wifi_tx_packets>=5)longjmp(done,1);
    assert(ticks<10000 || ticks>0xffff0000u); /* bounded test even on failure */
}
static void monotroch_init(void) {ticks_in_menu++;}
static void menu_auto_telemetry_init(void) {auto_menu_calls++;}
static void menu(int x) {(void)x;longjmp(done,1);}
"""
main=r"""
int main(void) {
#if QCAR_BOOT_IMU_WIFI
    if(!setjmp(done))firmware_main();
    assert(imu_calls==1 && imu_timer==1 && net_calls==1 && announcements>=1);
    assert(imu_wifi_boot_stage==6 && imu_wifi_tx_packets>=5 && imu_sample_sequence>0);
    assert(!ticks_in_menu && !cam_flag && !shacheflag);
    imu_calls=imu_timer=net_calls=ready=announcements=0;imu_wifi_tx_packets=0;retry_test=1;ticks=0xfffffff0u;
    if(!setjmp(done))firmware_main();
    assert(net_calls==2 && imu_timer==1 && announcements>=1 && imu_wifi_boot_stage==6);
    imu_calls=imu_timer=net_calls=ready=announcements=0;imu_wifi_tx_packets=0;retry_test=0;imu_bad=1;ticks=0;
    if(!setjmp(done))firmware_main();
    assert(imu_calls==1 && imu_timer==0 && net_calls==1 && imu_attitude_status==-1);
    puts("PASS: IMU init/timer precedes automatic WiFi; stream and timed retry/wrap; motor zero and no control timer");
#else
    if(!setjmp(done))firmware_main();
    assert(ticks_in_menu==1 && auto_menu_calls==1 && !imu_calls && !net_calls);
    puts("PASS: boot flag 0 restores original menu entry");
#endif
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-auto-imu-") as directory:
    p=Path(directory);src=p/"test.c";src.write_text(prefix+s+main,encoding="utf-8")
    for enabled in [1,0]:
        exe=p/("boot"+str(enabled)+".exe")
        subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror","-Wno-unused-function","-Wno-unused-variable",f"-DQCAR_BOOT_IMU_WIFI={enabled}","-I"+str(root/"code"),str(src),"-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=5)
