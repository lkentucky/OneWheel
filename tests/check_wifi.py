"""Exercise the actual ported transport with simulated STM and WiFi datagrams."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='qcar-wifi-') as directory:
    work = Path(directory)
    shutil.copy2(root/'code/wifispi.c', work/'wifispi.c')
    (work/'zf_common_typedef.h').write_text('''
#ifndef HOST_TYPES
#define HOST_TYPES
#include <stdint.h>
#include <stdbool.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int16_t int16; typedef int32_t int32;
#endif
''')
    (work/'IfxStm.h').write_text('''
#include "zf_common_typedef.h"
extern int host_stm;
#define MODULE_STM0 host_stm
uint32 IfxStm_getLower(int *stm); float IfxStm_getFrequency(int *stm);
''')
    (work/'zf_common_interrupt.h').write_text('''
#include "zf_common_typedef.h"
uint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);
''')
    (work/'zf_driver_delay.h').write_text('#include "zf_common_typedef.h"\nvoid system_delay_ms(uint32);\n')
    (work/'pid.h').write_text('''
#include "zf_common_typedef.h"
#include "qcar_types.h"
extern int16_t real_speed;
extern Mono_params mono_params;
extern float cascade_vel_out, cascade_vel_out_p, cascade_vel_out_y;
extern float cascade_angle_out, cascade_angle_out_p, cascade_angle_out_y;
extern float cascade_angle_vel_out, cascade_angle_vel_out_p, cascade_angle_vel_out_y;
extern pid_param_t cascade_vel_roll, cascade_vel_pitch, cascade_vel_yaw;
extern pid_param_t cascade_ang_roll, cascade_ang_pitch, cascade_ang_yaw;
extern pid_param_t cascade_ang_vel_roll, cascade_ang_vel_pitch, cascade_ang_vel_yaw;
''')
    for enabled in (1, 0):
        exe = work/f'wifi-{enabled}.exe'
        subprocess.run(['gcc', '-x', 'c', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                        f'-DQCAR_WIFI_ENABLED={enabled}', '-I'+str(work), '-I'+str(root/'code'),
                        '-I'+str(root/'libraries/zf_device'), str(work/'wifispi.c'),
                        str(root/'tests/wifi_transport_test.c.in'), '-lm', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True, timeout=10)
