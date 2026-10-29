"""Exercise actual menu open-loop transitions with hardware calls mocked."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'code/menu.c').read_text(encoding='utf-8')

def function(signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

prefix = r''' 
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "menu_tree.h"
typedef int16_t int16;
typedef uint8_t uint8;
#define PWM_DUTY_MAX 10000
#define CCU60_CH0 0
#define CCU60_CH1 1
static int imu_ready;
static int wheel_ready, driver_ready, shacheflag;
static int running, preview, wheel_test_pwm, redraw;
static int imu_wifi_status;
static float small_driver_test_left,small_driver_test_right;
static bool wheel_switch, control_timers_ready;
static struct { int setspeed; } mono_params;
static const char *message;
static bool wifi_connecting;
static Menu_Item *wifi_deferred_item;
static bool wifi_deferred_value;
static int output, writes, initializations, driver_stops, disabled;
static void motor_int(void) { initializations++; output = 0; }
static void motor_m_ctrl(int pwm) { output = pwm; writes++; }
static void pit_disable(int ch) { assert(control_timers_ready); disabled |= 1 << ch; }
static void small_driver_set_duty(int a,int b) { assert(!a && !b); driver_stops++; }
static void clear_all_cascade_out(void) {}
static void roll_rate_test_stop(void) {}
static void imu_service(void) {}
static bool start_switch, imu_switch, stop_switch, wifi_switch, save_switch, load_switch;
static uint8_t run_mode;
typedef enum { KEY_1, KEY_2, KEY_3, KEY_4 } key_index_enum;
typedef enum { KEY_RELEASE, KEY_SHORT_PRESS, KEY_LONG_PRESS } key_state_enum;
static key_state_enum states[4];
static Menu_Item *pointer;
static float steps[] = {.01f,.1f,1,10,100};
static uint8_t step_index = 2;
static void Menu_Request_Redraw(void) { redraw = 1; }
static void ips200_show_string(int x,int y,const char *s) {}
static key_state_enum key_get_state(key_index_enum k) { return states[k]; }
static void key_consume_state(key_index_enum k,key_state_enum e) { assert(states[k] == e); states[k]=KEY_RELEASE; }
static void start_car(void) { assert(0); }
static void imu_preview(void) { assert(0); }
static void save_params(void) { assert(0); }
static void load_params(void) { assert(0); }
static void wifispi_cancel_init(void) {}
static void wifispi_telemetry_init(void) { assert(0); }
static void wifi_connect_progress(void) {}
static void wifispi_set_progress_hook(void (*hook)(void)) { (void)hook; }
static void wifispi_telemetry_stop(void) { assert(0); }
static int wifispi_is_ready(void) { return 0; }
static void wifispi_telemetry_service(void) {}
static void menu_auto_wifi_service(void) {}
static void menu_wifi_connect(void) { assert(0); }
static void menu_wifi_poll(void) {}
static void menu_apply_deferred_control(void) {}
static void Menu_Show(void) {}
static void system_delay_ms(int ms) {}
'''
main = r'''
int main(void) {
    Menu_Item open_loop = {0};
    open_loop.kind=bool_Box; open_loop.data=&wheel_switch; pointer=&open_loop;
    states[KEY_3]=KEY_SHORT_PRESS; menu_poll(); assert(pointer->selected);
    states[KEY_4]=KEY_SHORT_PRESS; menu_poll(); assert(!pointer->selected);
    states[KEY_3]=KEY_SHORT_PRESS; menu_poll(); assert(pointer->selected);
    wheel_test_pwm=600;
    states[KEY_1]=KEY_SHORT_PRESS; menu_poll();
    assert(wheel_switch && preview==3 && output==600 && pointer->selected);
    assert(!control_timers_ready && disabled==0);
    states[KEY_4]=KEY_SHORT_PRESS; menu_poll(); assert(!pointer->selected && output==600);
    states[KEY_3]=KEY_SHORT_PRESS; menu_poll();
    states[KEY_2]=KEY_SHORT_PRESS; menu_poll(); assert(!wheel_switch && output==0);
    states[KEY_4]=KEY_SHORT_PRESS; menu_poll(); assert(!pointer->selected);
    initializations=wheel_ready=0;
    control_timers_ready=true;
    wheel_test_pwm = 1200;
    running = shacheflag = 1;
    /* 重构后 preview 是唯一的模式仲裁者：视觉/遥控/角速度测试已删除，
       cam_flag/tele_flag/turn_flag 不再存在，对应状态由 preview==3 表达。 */
    wheel_pwm_test(); menu_poll();
    assert(output == 1200 && wheel_switch && preview == 3);
    assert(!running && !shacheflag);
    assert(disabled == 3 && initializations == 1 && driver_stops == 0);
    int old_writes = writes;
    wheel_pwm_test(); assert(writes == old_writes); menu_poll();
    assert(output == 1200 && initializations == 1);
    wheel_test_pwm = -2300; menu_poll(); assert(output == -2300);
    wheel_test_pwm = 12000; menu_poll(); assert(output == 10000 && wheel_test_pwm == 10000);
    wheel_test_pwm = -12000; menu_poll(); assert(output == -10000 && wheel_test_pwm == -10000);
    wheel_test_pwm = 0; menu_poll(); assert(output == 0 && wheel_switch);
    wheel_test_pwm = 700; menu_poll(); stop_car();
    assert(output == 0 && !wheel_switch && preview == 0 && wheel_test_pwm == 700);
    old_writes = writes; menu_poll(); assert(output == 0 && writes == old_writes);
    wheel_pwm_test(); menu_poll(); assert(output == 700);
    driver_ready = 1; small_driver_test_left=500;small_driver_test_right=-500;stop_car(); assert(output == 0 && driver_stops == 1 && small_driver_test_left==0 && small_driver_test_right==0);
    wheel_test_pwm = 20000; menu_poll(); assert(output == 0 && wheel_test_pwm == 10000);
    run_mode=2;wheel_test_pwm=600;
    wheel_pwm_test();menu_poll();assert(!preview && !wheel_switch && !output);
    assert(strstr(message,"use Start"));
    small_driver_test_left=500;small_driver_test_right=-500;menu_poll();
    assert(!small_driver_test_left && !small_driver_test_right && !output);
    puts("PASS: preset enable, live PWM, reverse, clamp, zero, repeated enable, stop/re-enable; Mode2 blocks ground/flywheel open-loop commands");
}
'''
parts = [function(s) for s in ('static void ensure_wheel(void)',
         'static void stop_car(void)', 'static void wheel_pwm_test(void)',
         'static bool Menu_Handle_Control_Bool(', 'static bool Menu_Handle_Control_Uint8(',
         'void menu_poll(void)')]
with tempfile.TemporaryDirectory(prefix='qcar-open-loop-') as folder:
    work = Path(folder)
    test = work / 'open_loop.c'
    keys = source[source.index('void Key_Up(void)'):source.index('void Menu_Show(void)')]
    test.write_text(prefix + '\n'.join(parts[:-1]) + keys + parts[-1] + main, encoding='utf-8')
    exe = work / 'open_loop.exe'
    subprocess.run(['gcc', '-std=c99', '-I'+str(root/'code'), str(test), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=5)
