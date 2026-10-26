"""Native Qcar UI checks; only dedicated loopback listener, no vehicle traffic."""
from argparse import Namespace
import ctypes
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
import dearpygui.dearpygui as dpg
import app
import qcar_gui
from qcar_firmware import PROFILES
from qcar_parameters import DEBUG_PARAMETERS, PWM_PROFILE
from transport import Packet
from tools.check_field_layout import render


def check(width,height,output):
    case=output/str(width);case.mkdir(parents=True,exist_ok=True)
    app.STORE=case/'state'
    args=Namespace(profile='qcar',test_ports=True,listen_test=False,page='params',exit_after=0,startup_report=None,test_report=None)
    monitor=qcar_gui.QcarMonitor(args)
    sent=[]
    original=qcar_gui.send_udp
    qcar_gui.send_udp=lambda data,ip,port:sent.append((data,ip,port)) or len(data)
    peer=('127.0.0.1',5001)
    def inject(payload,address=peer):
        monitor.multiplex.receiver.queue.put(Packet(time.time(),address,payload))
        render(monitor,3)
    def schema(name):return ('MCAR SUB '+','.join(PROFILES[name])+'\n').encode()
    try:
        dpg.configure_viewport(0,width=width,height=height,x_pos=0,y_pos=0)
        dpg.show_viewport();dpg.set_value('bind_ip','127.0.0.1')
        monitor.start();render(monitor)
        assert dpg.get_value('remote_port')==5001 and not dpg.get_value('remote_ip')
        assert dpg.get_item_configuration('nav_camera')['enabled'] is False
        startup=(ROOT/'tests/fixtures/qcar_startup.bin').read_bytes()
        dual=(ROOT/'tests/fixtures/qcar_dual.bin').read_bytes()
        inject(startup)
        assert monitor.waiting_packets==1 and monitor.data_records==0
        inject(schema('startup')+b'MCAR RATE 20\nMCAR STREAM 1\n')
        assert monitor.peer==peer and dpg.get_value('remote_ip')==peer[0]
        assert monitor.protocol.period_ms==20 and monitor.protocol.stream==1
        inject(startup)
        assert monitor.latest['fault_reason']==4 and monitor.data_records==1
        inject(schema('dual'));inject(dual)
        assert len(monitor.rules)==45 and monitor.latest['dual_pitch_pwm']==-3000
        inject(schema('dual'))
        assert monitor.latest['dual_pitch_pwm']==-3000 # periodic announcement preserves live data
        monitor.show_page('plot');monitor._last_plot=0;render(monitor)
        assert len(dpg.get_value('series_dual_roll_actual_deg')[0])==1
        assert math.isclose(dpg.get_value('series_dual_pitch_actual_deg')[1][-1],3.9,rel_tol=1e-6)
        longest=max(monitor.rules,key=lambda r:dpg.get_text_size(r.name)[0]).name
        gap=dpg.get_item_state('value_'+longest)['rect_min'][0]-dpg.get_item_state('field_name_'+longest)['rect_min'][0]
        assert gap>=dpg.get_text_size(longest)[0]+8
        width_before=dpg.get_item_rect_size('plot_controls')[0]
        dpg.set_value('plot_sidebar_width',460);render(monitor)
        assert dpg.get_item_rect_size('plot_controls')[0]<width_before
        dpg.set_value('plot_sidebar_width',560);render(monitor)
        inject(dual,('127.0.0.2',5001));assert monitor.foreign_packets==1
        prior=monitor.parse_errors
        inject(dual[:-1]);inject(dual[:-4]+b'bad!')
        nan=bytearray(dual);struct.pack_into('<f',nan,0,float('nan'));inject(bytes(nan))
        assert monitor.parse_errors==prior+3 and monitor.latest['dual_pitch_pwm']==-3000
        monitor.show_page('tuning');render(monitor)
        dpg.set_value('qcar_edit_rate_kp',120)
        monitor.apply_qcar('rate_kp')
        assert sent[-1]==(b'[slider,rate_kp,120]\n',peer[0],peer[1])
        inject(b'MCAR SLIDER rate_kp 121\n')
        assert 'rate_kp' not in monitor.confirmed
        inject(b'MCAR SLIDER rate_kp 120\n',('127.0.0.1',5002))
        assert 'rate_kp' not in monitor.confirmed
        inject(b'MCAR SLIDER rate_kp 120\n')
        assert monitor.confirmed['rate_kp']==120 and not monitor.pending
        inject(b'MCAR SLIDER rate_pwm_limit 500\n')
        assert 'rate_pwm_limit' not in monitor.confirmed # unsolicited replies are not user-write confirmations
        dpg.set_value('qcar_edit_rate_sign',0)
        try:monitor.apply_qcar('rate_sign')
        except ValueError:pass
        else:raise AssertionError('Sign=0 accepted')
        dpg.set_value('qcar_edit_rate_sign',1)
        monitor.apply_qcar('rate_sign');monitor.pending['rate_sign']['time']-=3
        render(monitor)
        assert '超时' in monitor.qcar_states['rate_sign'] and not monitor.pending
        dpg.set_value('qcar_profile','双轴直立 · 45通道');monitor.subscribe_profile()
        assert sent[-1][0].startswith(b'SUB telemetry_ms,') and sent[-1][0].endswith(b'\nRATE 20\n')
        assert not any(b'ALIVE' in packet[0] or b'STOP' in packet[0] for packet in sent)
        dpg.set_value('qcar_edit_debug_fly_left_pwm',300)
        monitor.apply_qcar('debug_fly_left_pwm')
        assert sent[-1][0]==b'[slider,debug_fly_left_pwm,300]\n'
        assert set(monitor.pending)=={'debug_fly_left_pwm'}
        inject(b'MCAR SLIDER debug_fly_left_pwm 300\n')
        dpg.set_value('qcar_edit_debug_fly_right_pwm',-700)
        monitor.apply_qcar('debug_fly_right_pwm')
        assert sent[-1][0]==b'[slider,debug_fly_right_pwm,-700]\n'
        inject(b'MCAR SLIDER debug_fly_right_pwm -700\n')
        monitor.zero_pwm_debug()
        assert set(monitor.pending)==set(DEBUG_PARAMETERS)
        assert sent[-1][0]==b''.join(f'[slider,{name},0]\n'.encode() for name in DEBUG_PARAMETERS)
        inject(b''.join(f'MCAR SLIDER {name} 0\n'.encode() for name in DEBUG_PARAMETERS))
        assert not monitor.pending and all(monitor.confirmed[name]==0 for name in DEBUG_PARAMETERS)
        monitor.subscribe_pwm_debug()
        assert sent[-1][0]==('SUB '+','.join(PWM_PROFILE)+'\nRATE 20\n').encode()
        inject(('MCAR SUB '+','.join(PWM_PROFILE)+'\n').encode())
        values=dict.fromkeys(PWM_PROFILE,0)
        values.update(pwm_debug_enabled=1,rate_state=2,debug_fly_left_pwm=300,
                      debug_fly_right_pwm=-700,motor_left_pwm=300,motor_right_pwm=-700)
        inject(struct.pack('<15f',*(values[name] for name in PWM_PROFILE))+b'\x00\x00\x80\x7f')
        assert 'Mode=8' in dpg.get_value('qcar_state')
        assert '右下发: -700' in dpg.get_value('qcar_pwm_diagnostic')
        assert dpg.get_value('qcar_edit_debug_fly_left_pwm')==0 # telemetry never applies editor commands
        monitor.save_qcar_scheme()
        dpg.set_value('qcar_edit_balance_angle_x',0);monitor.load_qcar_scheme()
        assert dpg.get_value('qcar_edit_balance_angle_x')==3.9
        saved=json.loads(monitor._scheme_path().read_text('utf-8'))
        assert saved['version']=='QCAR_JUSTFLOAT_2' and len(saved['values'])==15
        legacy={key:saved['values'][key] for key in qcar_gui.LEGACY_PARAMETERS}
        monitor._scheme_path().write_text(json.dumps(dict(version='QCAR_JUSTFLOAT_1',values=legacy)),encoding='utf-8')
        monitor.load_qcar_scheme()
        assert dpg.get_value('qcar_edit_debug_fly_right_pwm')==0
        monitor.show_page('imu');render(monitor)
        assert '不含完整三轴' in dpg.get_value('angles')
        inject(schema('attitude'));inject(struct.pack('<3f',1,2,3)+b'\x00\x00\x80\x7f')
        assert monitor.latest['roll']==1 and monitor.latest['pitch']==2 and monitor.latest['yaw']==3
        # Changing the destination invalidates both schema and prior confirmations.
        dpg.set_value('remote_ip','127.0.0.2');render(monitor)
        assert not monitor.confirmed and not monitor.protocol.fields
        monitor.save_csv()
        return dict(viewport=[width,height],auto_schema=True,dual_plot=True,matched_ack=True,
                    timeout_rejected=True,range_checked=True,foreign_peer_rejected=True,
                    full_names=True,local_scheme=True,independent_pwm=True,pwm_zero_ack=True,
                    pwm_diagnostics=True,vehicle_contacted=False)
    finally:
        qcar_gui.send_udp=original
        monitor.stop();monitor.log_file.close();monitor.data_file.close();dpg.destroy_context()


if __name__=='__main__':
    output=Path(sys.argv[1]).resolve()
    results=[check(w,h,output) for w,h in [(1800,1120),(1250,820)]]
    (output/'report.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(results,ensure_ascii=False))
