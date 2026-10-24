"""Connection diagnostics must not be hidden by an active gyro or IMU page."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
s=(root/"code/menu.c").read_text("utf-8")
a=s.index("static void Menu_Show_Attitude(void)");b=s.index("void Key_Up(void)",a)
source=s[s.index("static const char *control_error_text"):a]+s[a:b]
prefix=r"""
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
typedef uint32_t uint32; typedef int32_t int32;
#define QCAR_WIFI_ENABLED 1
#define IMU_ATTITUDE_CALIBRATING 0
#define IMU_ATTITUDE_RUNNING 1
#define IMU_ATTITUDE_INIT_FAILED -1
#define IMU_ATTITUDE_TIMEOUT -2
#define IMU_ATTITUDE_BAD_DT -3
#define IMU_ATTITUDE_FILTER_ERROR -4
struct Item { const char *name; struct Item *Father; };
static struct Item wifi={"WiFi",0},gyro={"Gyro",0},item={"Connect",&wifi},*pointer=&item;
static const char *message="Stop before WiFi setup";
static int imu_wifi_module_result=-1,imu_wifi_status,imu_wifi_last_error,imu_ready=1;
static uint32 imu_wifi_init_attempts,imu_wifi_tx_packets;
static uint32 wifi_spi_config_error=3,wifi_spi_config_command=16,wifi_spi_config_reply,wifi_spi_config_reply_length;
static char wifi_spi_version[12]="V2.0";
static float imu_roll_deg,imu_pitch_deg,imu_calibration_percent,imu_sample_dt_ms;
static int32 imu_attitude_status=1;
static int32 control_fault_reason,control_fault_imu_status;
static uint32 imu_sample_sequence;
static int roll_rate_test_enabled=1,ground_rate_test_enabled,dual_balance_enabled,rate_only_test;
static float roll_rate_test_actual,roll_rate_test_pwm;
static float dual_rate[2],dual_pwm[2];
static int32 roll_rate_test_state=2;
static char rows[3][40];
static uint32 interrupt_global_disable(void) { return 0; }
static void interrupt_global_enable(uint32 x) { (void)x; }
static void ips200_show_string(int x,int y,const char *text) { (void)x;assert(y==152||y==172||y==192);snprintf(rows[(y-152)/20],40,"%s",text); }
"""
main=r"""
int main(void) {
    Menu_Show_Attitude(); assert(strstr(rows[2],"Stop before WiFi setup"));
    imu_wifi_status=imu_wifi_last_error=-2;imu_wifi_init_attempts=3;message="WiFi hotspot join failed";
    Menu_Show_Attitude(); assert(strstr(rows[0],"WiFi:-2 Err:-2 Try:3"));
    assert(strstr(rows[1],"FW:V2.0")); assert(strstr(rows[2],"Cmd:10 Fail:3 Ack:00 Len:0"));
    imu_wifi_status=1;imu_wifi_tx_packets=23;message="WiFi UDP connected";
    Menu_Show_Attitude(); assert(strstr(rows[0],"WiFi:1") && strstr(rows[1],"Tx:23"));
    item.Father=&gyro;Menu_Show_Attitude();assert(strstr(rows[0],"Rate:") && !strstr(rows[2],"Cmd:"));
    dual_balance_enabled=1;Menu_Show_Attitude();assert(strstr(rows[0],"Rate R:") && strstr(rows[2],"PWM R:"));
    item.Father=&wifi;roll_rate_test_state=-2;Menu_Show_Attitude();assert(strstr(rows[2],"ERR -2:"));
    roll_rate_test_state=-3;Menu_Show_Attitude();assert(strstr(rows[2],"KP/limit/sign/zero"));
    roll_rate_test_state=-1;imu_attitude_status=-3;Menu_Show_Attitude();assert(strstr(rows[2],"IMU dt too long"));
    imu_attitude_status=1;Menu_Show_Attitude();assert(strstr(rows[2],"stale/no sample"));
    item.Father=&gyro;roll_rate_test_state=1;Menu_Show_Attitude();assert(strstr(rows[2],"CAL"));
    puts("PASS: blocked Connect and failed join diagnostics stay visible over gyro/IMU status; normal gyro footer preserved");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-wifi-menu-") as directory:
    p=Path(directory);test=p/"test.c";exe=p/"test.exe"
    test.write_text(prefix+source+main,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(test),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
