"""Actual menu startup: automatic IMU+WiFi, stopped motors, streaming after Stop."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=(root/"code/menu.c").read_text("utf-8")
def function(signature):
 a=s.index(signature);b=s.index("{",a)+1;depth=1
 while depth:
  depth+=(s[b]=="{")-(s[b]=="}");b+=1
 return s[a:b]
prefix=r"""
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
typedef uint8_t uint8;typedef uint32_t uint32;
#define QCAR_BOOT_IMU_INIT 1
#define CCU60_CH0 0
#define CCU60_CH1 1
#define IMU_ATTITUDE_INIT_FAILED -1
static int MODULE_STM0;
static uint32 now;
static int wheel_ready,driver_ready,imu_ready,imu_attitude_status;
static bool control_timers_ready,wheel_switch,wifi_connecting;
static void *wifi_deferred_item;
static int cam_flag,tele_flag,turn_flag,shacheflag,wifi_spi_flag,running,preview,redraw;
static struct {int setspeed;} mono_params;
static const char *message;
static int imu_running,net_ready,net_calls,announces,imu_inits,flash_writes,prepare_calls;
static int imu_wifi_status,run_mode=4;
static void (*hook)(void);
static void motor_int(void) {}
static void motor_m_ctrl(int pwm) {assert(pwm==0);}
static void small_driver_uart_init(void) {}
static void small_driver_set_duty(int a,int b) {assert(!a&&!b);}
static void roll_rate_test_stop(void) {}
static void clear_all_cascade_out(void) {}
static void imu_init(void) {imu_inits++;imu_attitude_status=0;}
static void pit_disable(int ch) {if(ch==1)imu_running=0;}
static void pit_init(int ch,int us) {assert(ch==1&&us==5000);imu_running=1;}
static void wifi_connect_progress(void) {}
static void wifispi_set_progress_hook(void (*p)(void)) {hook=p;}
static void wifispi_telemetry_init(void) {assert(imu_ready&&imu_running&&!shacheflag);net_calls++;net_ready=1;}
static int wifispi_is_ready(void) {return net_ready;}
static void wifispi_telemetry_dual_profile(void) {}
static void wifispi_telemetry_startup_profile(void) {}
static void wifispi_telemetry_announce(void) {assert(net_ready);announces++;}
static void menu_wifi_announce(void) {wifispi_telemetry_announce();}
static uint32 IfxStm_getLower(int *p) {(void)p;return now;}
static float IfxStm_getFrequency(int *p) {(void)p;return 1000;}
static void write_flash(void) {assert(!imu_running);flash_writes++;}
static void imu_prepare_start(void) {prepare_calls++;}
"""
parts="\n".join(function(x) for x in ["static void ensure_wheel(void)","static uint8 ensure_imu(void)","static void stop_car(void)","static void save_params(void)"])
a=s.index("static uint32 auto_wifi_per_ms");b=s.index("static bool Menu_Handle_Control_Bool",a)
main=r"""
int main(void) {
 menu_auto_telemetry_init();
 assert(imu_inits==1&&imu_running&&net_calls==0&&announces==0&&!hook);
 menu_wifi_connect();assert(net_calls==1&&announces==1);
 assert(!running&&!preview&&!shacheflag);
 stop_car();assert(imu_running&&net_ready&&!shacheflag);
 now=1000;menu_auto_wifi_service();assert(announces==2);
 net_ready=0;running=1;now=6000;menu_auto_wifi_service();assert(net_calls==1);
 running=0;preview=3;menu_auto_wifi_service();assert(net_calls==1);
 preview=0;menu_auto_wifi_service();assert(net_calls==1&&imu_running);
 menu_wifi_connect();assert(net_calls==2);
 save_params();assert(flash_writes==1&&prepare_calls==1&&imu_running);
 puts("PASS: real menu auto-starts IMU, manual WiFi with motors zero; Stop keeps IMU/network; no background reconnect; Flash save pauses sampling");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-menu-auto-") as d:
 p=Path(d);src=p/"test.c";exe=p/"test.exe";src.write_text(prefix+parts+s[a:b]+main,encoding="utf-8")
 subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
 subprocess.run([str(exe)],check=True,timeout=5)
assert '"Connect", &wifi_switch' in s
