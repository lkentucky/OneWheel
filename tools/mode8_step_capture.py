"""Run one finite, left-only Mode8 sequence and capture real UDP telemetry.
Requires the matching TC264 firmware. Does not remotely Start or change mode.
"""
from pathlib import Path
import argparse
import csv
import json
import math
import socket
import statistics
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('telemetry_ms', 'rate_seq', 'pwm_debug_enabled', 'rate_state',
         'startup_scale', 'fault_reason', 'step_state', 'step_ms', 'step_stage',
         'debug_fly_left_pwm', 'debug_fly_right_pwm', 'debug_ground_pwm',
         'motor_left_pwm', 'motor_right_pwm', 'motor_ground_pwm',
         'flywheel_left_speed_raw', 'flywheel_right_speed_raw',
         'driver_speed_frames', 'driver_feedback_age_ms', 'driver_feedback_fresh')
TAIL = b'\x00\x00\x80\x7f'


def decode(packet):
    if len(packet) != 4 * len(NAMES) + 4 or not packet.endswith(TAIL):
        return None
    values = struct.unpack('<%df' % len(NAMES), packet[:-4])
    return dict(zip(NAMES, values)) if all(map(math.isfinite, values)) else None


def summarize(rows, output, error, events):
    report = {'real_board_data': bool(rows), 'error': error, 'frames': len(rows),
              'csv': str(output), 'events': events,
              'limits': ['PWM fields are TC264 commands, not physical PWM measurements.',
                         'Speed returned by stock CYT2BL3 is low-pass filtered (about 19.5ms time constant).',
                         'No phase current/Iq measured; no definitive torque-delay conclusion.',
                         'step_ms counts nominal controller ticks; no UART TX timestamp captured.']}
    if rows:
        times = [r['telemetry_ms'] for r in rows]
        gaps = [b-a for a,b in zip(times,times[1:]) if b>a]
        report.update(median_sample_ms=statistics.median(gaps) if gaps else None,
                      max_sample_gap_ms=max(gaps) if gaps else None,
                      max_feedback_age_ms=max(r['driver_feedback_age_ms'] for r in rows),
                      unique_driver_frames=len(set(r['driver_speed_frames'] for r in rows)),
                      completed=any(r['step_state']==2 for r in rows))
        # Estimate slopes only from distinct driver feedback frames, using board
        # receive-time estimates. This remains FILTERED speed acceleration.
        fresh = []; last = None
        for r in rows:
            if r['driver_feedback_fresh']==1 and r['driver_speed_frames']!=last:
                fresh.append(r); last=r['driver_speed_frames']
        accel=[]; at=[]
        for a,b in zip(fresh,fresh[1:]):
            dt=(b['telemetry_ms']-b['driver_feedback_age_ms']-
                a['telemetry_ms']+a['driver_feedback_age_ms'])/1000
            if dt>0:
                accel.append((b['flywheel_left_speed_raw']-a['flywheel_left_speed_raw'])/dt)
                at.append((b['telemetry_ms']-times[0])/1000)
        segments=[]
        for stage in range(1,6):
            rs=[r for r in rows if r['step_stage']==stage and r['step_state'] in (1,2)]
            if rs:
                segments.append(dict(stage=stage, first_step_ms=rs[0]['step_ms'],
                    command_pwm=rs[0]['motor_left_pwm'], samples=len(rs),
                    speed_first_rpm=rs[0]['flywheel_left_speed_raw'],
                    speed_last_rpm=rs[-1]['flywheel_left_speed_raw']))
        report['stages']=segments
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        x=[(v-times[0])/1000 for v in times]
        fig,ax=plt.subplots(4,1,figsize=(12,10),sharex=True)
        ax[0].step(x,[r['motor_left_pwm'] for r in rows],where='post',label='Left command')
        ax[0].plot(x,[r['motor_right_pwm'] for r in rows],label='Right command',alpha=.7)
        ax[0].plot(x,[r['motor_ground_pwm'] for r in rows],label='Ground command',alpha=.7)
        ax[0].set_ylabel('Command / 10000');ax[0].legend(loc='best')
        ax[1].plot(x,[r['flywheel_left_speed_raw'] for r in rows],label='Left filtered speed')
        ax[1].plot(x,[r['flywheel_right_speed_raw'] for r in rows],label='Right filtered speed',alpha=.6)
        ax[1].set_ylabel('RPM');ax[1].legend(loc='best')
        ax[2].plot(at,accel,'.-',markersize=2);ax[2].set_ylabel('Filtered RPM/s')
        ax[3].plot(x,[r['driver_feedback_age_ms'] for r in rows],label='Feedback age')
        ax[3].set_ylabel('ms');ax[3].set_xlabel('Board telemetry time / s')
        for a in ax:a.grid(alpha=.25)
        fig.suptitle('Real Mode8 step capture - TC264 commands and filtered driver feedback')
        fig.text(.5,.01,'No physical PWM or phase current captured. WiFi/driver filtering limits delay estimates.',ha='center',fontsize=9)
        fig.tight_layout(rect=(0,.025,1,.96))
        fig.savefig(output.with_suffix('.png'),dpi=160);plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='events'},indent=2,ensure_ascii=False),flush=True)
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--driver-trace',action='store_true',help='Arm/download passive driver recorder; requires both updated firmwares')
    ap.add_argument('--execute',action='store_true',help='Authorize one fixed 0/300/600/300/0 sequence')
    ap.add_argument('--wait',type=float,default=120)
    ap.add_argument('--port',type=int,default=8081)
    ap.add_argument('--module-ip')
    ap.add_argument('--module-port',type=int,default=5001)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    output=a.output or ROOT/'validation'/('mode8-step-'+time.strftime('%Y%m%d-%H%M%S')+'.csv')
    output.parent.mkdir(parents=True,exist_ok=True)
    rows=[];events=[];error='';peer=None;triggered=False;supports_step=False
    trace_client=None;driver_report=None
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.settimeout(.1)
    # Exclusive bind: never compete with an existing GUI receiver on 8081.
    s.bind(('0.0.0.0',a.port))
    def send(text):
        s.sendto((text+'\n').encode('ascii'),(peer,a.module_port))
        events.append(dict(host_monotonic=time.monotonic(),tx=text))
        if not text.startswith('DTRACE ROW '):print('TX',text,flush=True)
    def recv():
        try:
            p,addr=s.recvfrom(8192)
            if peer and addr[0]!=peer:return None
            return p,addr
        except socket.timeout:return None
    def confirm(command):
        expected=('MCAR '+command+'\n').encode('ascii')
        for retry in range(3):
            send(command);end=time.monotonic()+1
            while time.monotonic()<end:
                item=recv()
                if not item:continue
                p,_=item
                if p==expected:return
                if p.startswith(b'MCAR ERR '):raise RuntimeError(p.decode('ascii').strip())
        raise RuntimeError('No matching acknowledgement: '+command)
    print('LISTEN UDP',s.getsockname()[1],'; no motor command until firmware and Mode8 verified',flush=True)
    try:
        deadline=time.monotonic()+a.wait
        if a.module_ip:peer=a.module_ip
        while not peer and time.monotonic()<deadline:
            item=recv()
            if item:
                p,addr=item
                if p.startswith(b'MCAR ') or p.endswith(TAIL):peer=addr[0]
        if not peer:raise RuntimeError('No board datagram received on UDP 8081')
        print('BOARD',peer,flush=True)
        confirm('STREAM 0');confirm('SUB '+','.join(NAMES));supports_step=True
        confirm('RATE 5');confirm('STREAM 1')
        ready_since=None;last_data=time.monotonic();trigger_time=None;done_at=None;last_board=None
        with output.open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=['host_monotonic_s',*NAMES]);writer.writeheader()
            while time.monotonic()<deadline:
                now=time.monotonic()
                if triggered and now-last_data>.5:raise RuntimeError('Telemetry lost >500ms; cancel sent; local sequence has finite duration')
                if triggered and now-trigger_time>12:raise RuntimeError('Sequence did not complete within 12s')
                item=recv()
                if not item:continue
                p,_=item
                if p.startswith(b'MCAR '):
                    text=p.decode('ascii',errors='replace').strip();events.append(dict(rx=text,host_monotonic=now))
                    print('RX',text,flush=True)
                    if text.startswith('MCAR ERR '):raise RuntimeError(text)
                    continue
                r=decode(p)
                if r is None:raise RuntimeError('Unexpected telemetry schema; possible Start/mode change')
                if last_board is not None and r['telemetry_ms']<=last_board:continue
                last_board=r['telemetry_ms'];last_data=now
                r['host_monotonic_s']=now;rows.append(r);writer.writerow(r)
                if len(rows)%100==0:f.flush();print('FRAMES',len(rows),'stage',r['step_stage'],'PWM',r['motor_left_pwm'],'RPM',r['flywheel_left_speed_raw'],flush=True)
                active=(r['pwm_debug_enabled']==1 and r['rate_state']==2 and r['fault_reason']==0)
                if triggered:
                    if not active:raise RuntimeError('Mode8 stopped/faulted; test aborted')
                    if r['driver_feedback_fresh']!=1 or r['driver_feedback_age_ms']>100:raise RuntimeError('Driver feedback stale')
                    if r['motor_right_pwm']!=0 or r['motor_ground_pwm']!=0:raise RuntimeError('Unexpected nonzero untested motor command')
                    if abs(r['motor_left_pwm'])>600:raise RuntimeError('Unexpected command exceeds fixed sequence')
                    if r['step_state']==-1:raise RuntimeError('Board aborted sequence')
                    if r['step_state']==2:
                        if r['motor_left_pwm']!=0:raise RuntimeError('Completion without zero command')
                        if done_at is None:done_at=now
                        if now-done_at>=.5:break
                    elif now-trigger_time>1 and r['step_state']!=1:raise RuntimeError('Trigger not applied')
                else:
                    zeros=all(r[n]==0 for n in ('debug_fly_left_pwm','debug_fly_right_pwm','debug_ground_pwm','motor_left_pwm','motor_right_pwm','motor_ground_pwm'))
                    ready=(active and zeros and r['startup_scale']==1 and r['step_state']!=1 and r['driver_feedback_fresh']==1 and r['driver_feedback_age_ms']<50 and abs(r['flywheel_left_speed_raw'])<30 and abs(r['flywheel_right_speed_raw'])<30)
                    ready_since=now if ready and ready_since is None else ready_since if ready else None
                    if ready_since is not None and now-ready_since>=.5:
                        if not a.execute:print('READY; capture-only mode, no trigger',flush=True);break
                        if a.driver_trace:
                            from driver_trace_client import DriverTraceClient
                            trace_client=DriverTraceClient(send,recv);trace_client.arm()
                        trigger_time=time.monotonic();triggered=True
                        last_data=trigger_time
                        send('[slider,debug_step_run,1]') # Exactly once; never retrigger a completed sequence.
            else:raise RuntimeError('Timed out waiting for ready Mode8/sequence')
        if trace_client is not None:
            driver_report=trace_client.download(output)
            subprocess.run([sys.executable,str(ROOT/'tools/analyze_driver_trace.py'),
                str(output.with_name(output.stem+'-driver.json'))],check=True)
    except (OSError,RuntimeError,KeyboardInterrupt,ValueError,subprocess.CalledProcessError) as exc:
        error=str(exc) or type(exc).__name__;print('STOP:',error,flush=True)
    finally:
        if peer and supports_step and triggered:
            for i in range(3):
                try:send('[slider,debug_step_run,0]')
                except OSError:pass
        if peer and error and trace_client is not None:
            try:send('DTRACE CANCEL')
            except OSError:pass
        if peer and error:
            try:send('STREAM 1')
            except OSError:pass
        s.close()
        report=summarize(rows,output,error,events)
        if driver_report is not None:
            report['driver_trace']=driver_report
            output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        elif a.driver_trace and trace_client is not None:
            print('Driver trace not downloaded; capture remains on driver until rearm/reset.',flush=True)
    return 0 if not error and (not a.execute or report.get('completed')) else 2

if __name__=='__main__':raise SystemExit(main())
