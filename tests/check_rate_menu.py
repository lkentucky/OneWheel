"""Verify actual menu Start/Stop dispatch for isolated rate mode."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root / "code/menu.c").read_text(encoding="utf-8")
def function(signature):
    pos=s.index(signature); start=s.index("{",pos); end=start+1; depth=1
    while depth:
        depth+=(s[end]=="{")-(s[end]=="}"); end+=1
    return s[pos:end]
prefix=r"""
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#define UART_3 3
#define QCAR_DRIVER_UART UART_3
#define UART3_TX_P15_6 6
#define UART3_RX_P15_7 7
#define CCU60_CH0 0
#define CCU60_CH1 1
static int imu_ready;
static int control_timers_ready,cam_flag,tele_flag,turn_flag,shacheflag;
static int wifi_spi_flag,running,preview,wheel_ready,driver_ready,remote_ready;
static int run_mode=3,redraw,roll_rate_test_enabled,imu_ok=1,camera_calls,timers;
static bool wheel_switch;
static struct { int setspeed; } mono_params;
static const char *message;
static void pit_disable(int ch) { (void)ch; }
static void roll_rate_test_stop(void) { roll_rate_test_enabled=0; }
static void roll_rate_only_arm(void) {assert(run_mode==6);roll_rate_test_enabled=1;}
static void ground_rate_only_arm(void) {assert(run_mode==7);roll_rate_test_enabled=1;}
static void dual_balance_arm(void) {assert(run_mode==5);roll_rate_test_enabled=1;}
static int wifispi_is_ready(void) {return 0;}
static void wifispi_telemetry_dual_profile(void) {}
static void wifispi_telemetry_startup_profile(void) {}
static void ground_rate_test_arm(void) {assert(run_mode==4);roll_rate_test_enabled=1;}
static void roll_rate_test_arm(void) { assert(shacheflag && run_mode==3); roll_rate_test_enabled=1; }
static void motor_m_ctrl(int v) { assert(v==0); }
static void small_driver_set_duty(int a,int b) { assert(a==0 && b==0); }
static void clear_all_cascade_out(void) {}
static int ensure_imu(void) { return imu_ok; }
static int ensure_camera(void) { camera_calls++; return 1; }
static void ensure_wheel(void) { wheel_ready=1; }
static void small_driver_uart_init(void) {}
static void uart_init(int a,int b,int c,int d) { (void)a;(void)b;(void)c;(void)d; assert(0); }
static void uart_rx_interrupt(int a,int b) { (void)a;(void)b; assert(0); }
static void turn_info_init(void) {}
static void imu_prepare_start(void) {}
static void pit_init(int ch,int us) { assert(roll_rate_test_enabled); assert(us==(ch==CCU60_CH1?5000:1000)); timers++; }
"""
main=r"""
int main(void) {
    start_car();
    assert(running && roll_rate_test_enabled && shacheflag && driver_ready);
    assert(!cam_flag && !tele_flag && !turn_flag && !camera_calls && timers==2);
    stop_car(); assert(!running && !roll_rate_test_enabled && !shacheflag);
    run_mode=4;start_car();assert(running && roll_rate_test_enabled && !turn_flag && !cam_flag && !tele_flag);stop_car();
    run_mode=5;start_car();assert(running && !turn_flag && !tele_flag && !cam_flag);stop_car();
    imu_ok=0; start_car(); assert(!running && !roll_rate_test_enabled && timers==6);
    imu_ok=1; run_mode=2; start_car(); assert(!running && !roll_rate_test_enabled && !remote_ready);
    puts("PASS: mode 3 arms only gyro path, starts 5ms IMU/1ms dispatch, Stop disarms, IMU failure/UART3 remote conflict refuse start");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-rate-menu-") as folder:
    folder=Path(folder); src=folder/"test.c"; exe=folder/"test.exe"
    src.write_text(prefix+function("static void stop_car(void)")+function("static void start_car(void)")+main,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
