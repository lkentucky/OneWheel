"""Exercise the actual commissioning state machine without hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "tests/motor_uart_pulse.c.in").read_text(encoding="utf-8")
source = source.replace('#include "zf_common_headfile.h"', '').replace('#include "IfxStm.h"', '')
source = source.replace('int core0_main(void)', 'int firmware_main(void)')
prefix = r"""
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
typedef uint32_t uint32;
#define left 1
#define right 2
#define TRUE 1
#define IPS200_PORTAIT 0
#define RGB565_WHITE 65535
#define RGB565_BLACK 0
#define IPS200_TYPE_SPI 0
#define IPS200_6X8_FONT 0
int MODULE_STM0;
uint32 small_driver_speed_frames, small_driver_rx_bytes;
struct { int receive_left_speed_data,receive_right_speed_data; } motor_value;
static int sent_left, sent_right;
void small_driver_set_duty(int a,int b) { sent_left=a; sent_right=b; }
void clock_init(void); void debug_init(void);
void ips200_set_dir(int); void ips200_set_color(int,int); void ips200_init(int);
void ips200_set_font(int); void ips200_clear(void); void ips200_show_string(int,int,const char *);
void small_driver_uart_init(void); void cpu_wait_event_ready(void);
uint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);
float IfxStm_getFrequency(int *); uint32 IfxStm_getLower(int *);
void small_driver_get_speed(void); void system_delay_ms(int);
"""
main = r"""
static void step(uint32 now,uint32 frames) { motor_test_service(now,frames); }
int main(void) {
    motor_test_ticks_per_ms=1; motor_test_duty=1000; motor_test_duration_ms=500;
    step(0,0); assert(!sent_left && !sent_right);
    motor_test_request=1; step(1,0); assert(motor_test_result==6 && !sent_left);
    step(1001,0); assert(motor_test_result==3 && !sent_left);
    motor_test_request=1; step(10,1);
    assert(sent_left==1000 && !sent_right && motor_test_result==1 && motor_test_request==0);
    motor_test_request=2; step(200,2); assert(sent_left==1000 && !sent_right);
    step(509,3); assert(sent_left==1000);
    step(510,4); assert(!sent_left && !sent_right && motor_test_result==2);
    step(511,5); assert(!sent_left && !sent_right); /* no automatic repeat */
    motor_test_duty=-9000; motor_test_duration_ms=99999; motor_test_request=2;
    step(520,6); assert(!sent_left && sent_right==-2000 && motor_test_duration_ms==1000);
    motor_test_request=9; step(521,7); assert(!sent_left && !sent_right && motor_test_result==0);
    motor_test_request=1; step(522,8); step(1022,8);
    assert(motor_test_result==5 && !sent_left); /* feedback lost */
    motor_test_request=1; step(1023,8); assert(motor_test_result==6 && !sent_left);
    step(1024,9); assert(motor_test_result==1 && sent_left==-2000);
    motor_test_request=9; step(1025,10); assert(!sent_left);
    motor_test_request=42; step(1026,11); assert(motor_test_result==4 && !sent_left);
    motor_test_duration_ms=0; motor_test_duty=500; motor_test_request=2;
    step(UINT32_MAX-50,12); assert(sent_right==500 && motor_test_duration_ms==100);
    step(48,13); assert(sent_right==500);
    step(49,14); assert(!sent_right && motor_test_result==2); /* timer wrap */
    puts("PASS: idle, no link, left/right selection, repeated send, bounded duration/duty, stop, stale feedback, timer wrap");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-uart-pulse-") as folder:
    folder = Path(folder)
    test = folder / "pulse.c"
    test.write_text(prefix + source + main, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-fsyntax-only", str(test)], check=True)
    test.write_text(prefix + source[:source.index("int firmware_main(void)")] + main, encoding="utf-8")
    exe = folder / "pulse.exe"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections", str(test), "-Wl,--gc-sections", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=5)
