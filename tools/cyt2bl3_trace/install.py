"""Install passive trace hooks, preserving original encoding and full backups."""
from pathlib import Path
import argparse,hashlib,json,time
ROOT=Path(__file__).resolve().parents[2]
def load(p):
    b=p.read_bytes()
    try:s=b.decode('utf-8-sig');enc='utf-8-sig' if b.startswith(b'\xef\xbb\xbf') else 'utf-8'
    except UnicodeDecodeError:s=b.decode('gbk');enc='gbk'
    return s.replace('\r\n','\n'),enc,'\r\n' if b'\r\n' in b else '\n'
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,required=True);ap.add_argument('--check',action='store_true');a=ap.parse_args();project=a.project
    updates=[]
    def edit(rel,fn):
        p=project/rel;s,enc,nl=load(p)
        if '#include "driver_trace.h"' not in s and rel!='iar/project_config/cyt2bl3.ewp':s=fn(s)
        elif rel=='iar/project_config/cyt2bl3.ewp' and 'code\\driver_trace.c' not in s:s=fn(s)
        updates.append((p,s.replace('\n',nl).encode(enc)))
    def motor(s):
        s=s.replace('#include "motor_control.h"','#include "motor_control.h"\n#include "driver_trace.h"',1)
        s=s.replace('        motor_left.menc15a_reduction_integral += motor_left.menc15a_value_offset;', '        motor_left.menc15a_reduction_integral += motor_left.menc15a_value_offset;\n        driver_trace_on_encoder(motor_left.menc15a_value_now,motor_left.menc15a_value_offset);',1)
        pos=s.index('        fast_foc_calculate(&motor_left_foc_driver,');stop=s.index('\n        if(motor_left.motor_protect_state',pos)
        part=s[pos:stop]
        part=part.replace('motor_left.motor_duty','trace_duty')
        s=s[:pos]+'''        uint32 trace_irq=__get_PRIMASK();
        uint16 trace_generation;
        float trace_duty;
        __disable_irq();
        trace_generation=driver_trace_generation();trace_duty=motor_left.motor_duty;
        __set_PRIMASK(trace_irq);
'''+part+s[stop:]
        needle='            motor_left_duty_set(motor_left_foc_driver.ouput_duty[0], motor_left_foc_driver.ouput_duty[1], motor_left_foc_driver.ouput_duty[2]);'
        pos=s.index(needle);end=s.index('\n        }',pos)+len('\n        }');s=s[:end]+'\n        driver_trace_on_pwm_written(trace_generation);'+s[end:]
        pos=s.index('void driver_cmd_forthwith(void)');end=s.index('void driver_cmd_loop(void)',pos);part=s[pos:end]
        needle='case SET_DUTY:';pos2=part.index(needle);b=part.index('{',pos2)+1
        part=part[:b]+'''\n                uint32 trace_irq=__get_PRIMASK();
                __disable_irq();
'''+part[b:]
        needle='                motor_right.lose_control_protect_count = 0;';assert needle in part
        part=part.replace(needle,needle+'''
                driver_trace_on_command((int16)func_limit_ab(motor_value.receive_left_duty_data,-10000,10000),
                                        (int16)func_limit_ab(motor_value.receive_right_duty_data,-10000,10000));
                __set_PRIMASK(trace_irq);''',1)
        return s[:pos]+part+s[end:]
    def uart(s):
        s=s.replace('#include "motor_driver_uart_control.h"','#include "motor_driver_uart_control.h"\n#include "driver_trace.h"',1)
        pos=s.index('void motor_driver_parse_statement(');b=s.index('{',pos)+1
        s=s[:b]+'\n    if(device_value->cmd_type==STRING_TYPE && driver_trace_text(statement_buffer)) return;\n'+s[b:]
        needle='        fifo_write_element(&motor_driver_fifo, receive_data);';assert needle in s
        return s.replace(needle,'        driver_trace_on_rx_byte(receive_data);\n'+needle,1)
    def main_file(s):
        s=s.replace('#include "zf_common_headfile.h"','#include "zf_common_headfile.h"\n#include "driver_trace.h"',1)
        s=s.replace('    motor_control_init();','    driver_trace_init(); /* FAST_FOC mode check uses the compile-time mode below. */\n    motor_control_init();',1)
        # Init must happen before PWM IRQs can be enabled, but mode lives in
        # the config macro until motor_control_init initializes motor_left.
        s=s.replace('    driver_trace_init(); /* FAST_FOC mode check uses the compile-time mode below. */\n    motor_control_init();','    motor_control_init();',1)
        # All global interrupts are still disabled here.
        needle='    interrupt_global_enable(0);';assert needle in s
        s=s.replace(needle,'    driver_trace_init();\n'+needle,1)
        return s.replace('        driver_cmd_loop();','        driver_trace_service();\n        driver_cmd_loop();',1)
    def ewp(s):
        needle='        <file>\n            <name>$PROJ_DIR$\\..\\..\\code\\motor_control.c</name>'
        assert needle in s
        return s.replace(needle,'        <file>\n            <name>$PROJ_DIR$\\..\\..\\code\\driver_trace.c</name>\n        </file>\n        <file>\n            <name>$PROJ_DIR$\\..\\..\\code\\driver_trace.h</name>\n        </file>\n'+needle,1)
    edit('code/motor_control.c',motor);edit('code/motor_driver_uart_control.c',uart);edit('user/main_cm4.c',main_file);edit('iar/project_config/cyt2bl3.ewp',ewp)
    for n in ('driver_trace.c','driver_trace.h'):
        updates.append((project/'code'/n,(Path(__file__).parent/'code'/n).read_text('utf-8-sig').replace('\n','\r\n').encode('ascii')))
    changed=[(p,b) for p,b in updates if not p.exists() or p.read_bytes()!=b]
    if a.check:
        print('Pending files:',len(changed));return
    backup=ROOT/'validation'/('driver-trace-backup-'+time.strftime('%Y%m%d-%H%M%S'));backup.mkdir(parents=True)
    manifest=[]
    for p,b in changed:
        rel=p.relative_to(project);old=p.read_bytes() if p.exists() else None
        if old is not None:
            q=backup/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(old)
        manifest.append(dict(path=str(p),before_sha256=hashlib.sha256(old).hexdigest() if old else None,after_sha256=hashlib.sha256(b).hexdigest()))
        p.write_bytes(b);print('Updated',p)
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8');print('Backup',backup)
if __name__=='__main__':main()
