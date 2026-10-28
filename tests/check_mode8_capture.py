"""Loopback-only integration of capture, trigger, cancel and plot; no hardware."""
from pathlib import Path
import importlib.util
import json
import socket
import struct
import subprocess
import sys
import tempfile
import time

root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('step_capture',root/'tools/mode8_step_capture.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.decode(b'bad') is None
bad=[0.0]*len(m.NAMES);bad[0]=float('nan')
assert m.decode(struct.pack('<%df'%len(bad),*bad)+m.TAIL) is None

# Test against an isolated 127.0.0.1 peer. Simulated frames stay in temp dirs.
for old_firmware in (True,False):
    with tempfile.TemporaryDirectory(prefix='qcar-step-loopback-') as d:
        output=Path(d)/'loopback.csv'
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',0));s.settimeout(.005)
        cmd=[sys.executable,str(root/'tools/mode8_step_capture.py'),'--module-ip','127.0.0.1','--port','0','--module-port',str(s.getsockname()[1]),'--wait','20','--execute','--output',str(output)]
        p=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8')
        started=time.monotonic();run_at=None;commands=[];stream=False;sub=False;host=('127.0.0.1',18081);last_emit=0;seq=0
        while p.poll() is None and time.monotonic()-started<23:
            try:
                data,addr=s.recvfrom(8192);text=data.decode().strip();commands.append(text);host=addr
                if text.startswith('SUB ') and old_firmware:
                    s.sendto(b'MCAR ERR unknown variable\n',addr);continue
                if text.startswith('SUB '):sub=True
                if text=='STREAM 1':stream=True
                if text=='STREAM 0':stream=False
                if text=='[slider,debug_step_run,1]':
                    assert run_at is None;run_at=time.monotonic()
                    s.sendto(b'MCAR SLIDER debug_step_run 1\n',addr)
                elif text=='[slider,debug_step_run,0]':
                    s.sendto(b'MCAR SLIDER debug_step_run 0\n',addr)
                else:s.sendto(('MCAR '+text+'\n').encode(),addr)
            except (socket.timeout,ConnectionResetError):pass
            now=time.monotonic()
            if sub and stream and now-last_emit>=.005:
                seq+=1;last_emit=now
                t=min(7500,int((now-run_at)*1000)//5*5) if run_at else 0
                stage=0 if not run_at else 1 if t<1000 else 2 if t<2500 else 3 if t<4000 else 4 if t<5500 else 5
                pwm=300 if stage in (2,4) else 600 if stage==3 else 0
                r=dict.fromkeys(m.NAMES,0.0)
                r.update(telemetry_ms=(now-started)*1000,rate_seq=seq,pwm_debug_enabled=1,rate_state=2,startup_scale=1,
                         step_state=(2 if t==7500 else 1) if run_at else 0,step_ms=t,step_stage=stage,
                         debug_fly_left_pwm=pwm,motor_left_pwm=-pwm,driver_speed_frames=seq,
                         driver_feedback_fresh=1,driver_feedback_age_ms=1,flywheel_left_speed_raw=-pwm/2)
                s.sendto(struct.pack('<%df'%len(m.NAMES),*(r[n] for n in m.NAMES))+m.TAIL,host)
        if p.poll() is None:p.kill()
        log=p.communicate(timeout=5)[0];s.close()
        if old_firmware:
            assert p.returncode==2,log
            assert '[slider,debug_step_run,1]' not in commands
            assert commands[-1]=='STREAM 1'
        else:
            assert p.returncode==0,log
            assert commands.count('[slider,debug_step_run,1]')==1
            assert commands.count('[slider,debug_step_run,0]')==3
            report=json.loads(output.with_suffix('.json').read_text('utf-8'))
            assert report['completed'] and len(report['stages'])==5 and not report['error']
            assert output.with_suffix('.png').stat().st_size>1000
print('PASS: loopback-only old-firmware rejection, no accidental trigger, exact-once sequence, CSV/PNG/JSON and final cancel')
