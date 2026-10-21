"""Exercise the actual IMU menu renderer with captured LCD output."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'code/menu.c').read_text(encoding='utf-8')
renderer = source[source.index('static void Menu_Show_Attitude(void)'):source.index('void Key_Up(void)')]
prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "imu.h"
static int imu_ready, running, preview, irq_disabled;
static const char *message = "Ready";
volatile int32 imu_attitude_status = IMU_ATTITUDE_INIT_FAILED;
volatile float imu_roll_deg, imu_pitch_deg, imu_calibration_percent, imu_sample_dt_ms;
volatile uint32 imu_sample_sequence;
static char angle_line[40], status_line[40], message_line[40];
static uint32 interrupt_global_disable(void) { assert(!irq_disabled); irq_disabled=1; return 0; }
static void interrupt_global_enable(uint32 irq) { assert(!irq && irq_disabled); irq_disabled=0; }
static void ips200_show_string(int x, int y, const char *s) {
    assert(!irq_disabled && !x && strlen(s)<40);
    if(y==152) strcpy(angle_line,s);
    else if(y==172) strcpy(status_line,s);
    else { assert(y==192); strcpy(message_line,s); }
}
'''
main = r'''
int main(void) {
    Menu_Show_Attitude(); assert(strstr(status_line,"OFF"));
    imu_ready=1; preview=2; imu_attitude_status=IMU_ATTITUDE_CALIBRATING;
    imu_calibration_percent=37.5f; Menu_Show_Attitude();
    assert(strstr(status_line,"CAL") && strstr(status_line,"37.5%"));
    assert(strstr(message_line,"init OK") && strstr(message_line,"keep still"));
    imu_calibration_percent=0; Menu_Show_Attitude(); assert(strstr(status_line,"0.0%"));
    imu_attitude_status=IMU_ATTITUDE_RUNNING; imu_sample_sequence=200;
    imu_roll_deg=12.25f; imu_pitch_deg=-8.5f; imu_sample_dt_ms=5;
    Menu_Show_Attitude();
    assert(strstr(status_line,"RUN Seq:200") && strstr(status_line,"5.0ms"));
    assert(strstr(angle_line,"12.25") && strstr(angle_line,"-8.50"));
    assert(strstr(message_line,"calibration OK") && !strstr(message_line,"keep still"));
    imu_attitude_status=IMU_ATTITUDE_TIMEOUT; Menu_Show_Attitude();
    assert(strstr(status_line,"TIMEOUT") && strstr(message_line,"IMU error"));
    preview=0; message="Stopped"; Menu_Show_Attitude();
    assert(strstr(status_line,"STOP") && strstr(message_line,"Stopped"));
    imu_ready=0; message="IMU init failed"; Menu_Show_Attitude();
    assert(strstr(message_line,"init failed") && !strstr(message_line,"init OK"));
    puts("PASS: IMU disabled, init success, calibration progress/reset, running angles, fault and stop messages");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-imu-display-') as directory:
    work = Path(directory)
    (work / 'zf_common_typedef.h').write_text(
        '#include <stdint.h>\ntypedef uint8_t uint8; typedef uint32_t uint32; typedef int32_t int32;\n')
    test = work / 'display.c'
    test.write_text(prefix + renderer + main, encoding='utf-8')
    exe = work / 'display.exe'
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-I'+str(work),
                    '-I'+str(root/'code'), str(test), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=5)
