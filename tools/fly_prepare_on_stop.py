"""Prepare/observe one supported flywheel tuning stage; never remotely Start."""
import argparse,json,socket,struct,time,math
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--prepare',action='store_true');ap.add_argument('--supported-running',action='store_true');ap.add_argument('--zero-ki',action='store_true');ap.add_argument('--isolate-flight',action='store_true');ap.add_argument('--angle-kp',type=float);ap.add_argument('--seconds',type=float,default=40);args=ap.parse_args()
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',8081));s.settimeout(.05);s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
peer=None;old=None;oldrate=None;oldstream=None;names=None;latest=None;rows=[];events=[];baseline=None;error='';updates={'fly_vel_kp':0,'fly_vel_kd':0}
if args.zero_ki:updates={'fly_rate_ki':0}
if args.isolate_flight:updates={'fly_vel_kp':0,'fly_vel_kd':0,'fly_rate_ki':0}
if args.angle_kp is not None:
 if not math.isfinite(args.angle_kp) or not 0<=args.angle_kp<=150:raise SystemExit('Invalid angle Kp')
 updates={'fly_angle_kp':args.angle_kp}
channels='telemetry_ms,control_mode,rate_state,fault_reason,imu_status,imu_seq,fly_angle_kp,fly_rate_kp,fly_rate_ki,fly_vel_kp,fly_angle_kd,fly_vel_kd,fly_sign,dual_roll_actual_deg,dual_roll_target_deg,dual_roll_error_deg,dual_roll_target_rate_dps,dual_roll_rate_dps,dual_roll_pwm,motor_left_pwm,motor_right_pwm,motor_ground_pwm,flywheel_left_speed_raw,flywheel_right_speed_raw,driver_feedback_fresh,driver_feedback_age_ms'
def send(c):
 s.sendto((c+'\n').encode(),peer);events.append({'t':time.monotonic(),'tx':c})
def receive():
 global peer,old,oldrate,oldstream,names,latest
 try:p,a=s.recvfrom(8192)
 except socket.timeout:return None
 if peer and a[0]!=peer[0]:return None
 if p.startswith(b'MCAR '):
  peer=(a[0],5001);v=p.decode().strip();events.append({'t':time.monotonic(),'rx':v})
  if v.startswith('MCAR SUB '):
   if old is None:old=v[9:]
   names=v[9:].split(',')
  if v.startswith('MCAR RATE ') and oldrate is None:oldrate=v.split()[-1]
  if v.startswith('MCAR STREAM ') and oldstream is None:oldstream=v.split()[-1]
  if v.startswith('MCAR ERR'):raise RuntimeError(v)
  return v
 if names==channels.split(',') and len(p)==4*(len(names)+1) and p[-4:]==b'\x00\x00\x80\x7f':
  vals=struct.unpack('<%df'%len(names),p[:-4])
  if not all(map(math.isfinite,vals)):raise RuntimeError('Nonfinite data')
  latest=dict(zip(names,vals));latest['host_monotonic']=time.monotonic();rows.append(latest)
 return None
def sample(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:receive()
def write(k,v):
 for attempt in range(3):
  send(f'[slider,{k},{v}]');end=time.monotonic()+.7
  while time.monotonic()<end:
   reply=receive()
   if reply and reply.startswith('MCAR SLIDER '+k+' ') and abs(float(reply.split()[-1])-v)<1e-5:return
 raise RuntimeError('Write not confirmed: '+k)
try:
 for ip in ['192.168.0.114','192.168.0.113','192.168.0.119','192.168.0.255']:s.sendto(b'GET?\n',(ip,5001))
 end=time.monotonic()+5
 while time.monotonic()<end:
  receive()
  if old is not None and oldrate is not None and oldstream is not None:break
 if old is None:raise RuntimeError('No module reply')
 send('SUB '+channels);send('RATE 10');send('STREAM 1');sample(.7)
 if latest is None or time.monotonic()-latest['host_monotonic']>.3:raise RuntimeError('No fresh telemetry')
 baseline=dict(latest)
 if args.prepare:
  if args.supported_running:
   deadline=time.monotonic()+30;steady_since=None
   while time.monotonic()<deadline:
    receive()
    fresh=time.monotonic()-latest['host_monotonic']<.2
    near=False and fresh and latest['imu_status']==1 and latest['fault_reason']==0 and latest['driver_feedback_fresh']==1 and latest['control_mode']==10 and latest['rate_state']==2 and abs(latest['dual_roll_error_deg'])<3 and abs(latest['dual_roll_rate_dps'])<15 and abs(latest['dual_roll_pwm'])<1500
    stopped_now=fresh and latest['control_mode']==0 and latest['rate_state']==0 and all(latest[k]==0 for k in ['motor_left_pwm','motor_right_pwm','motor_ground_pwm'])
    if near or stopped_now:
     if steady_since is None:steady_since=time.monotonic()
     if stopped_now or time.monotonic()-steady_since>=.4:break
    else:steady_since=None
   else:raise RuntimeError('No supported steady window; no writes made')
  baseline=dict(latest)
  stopped=latest['control_mode']==0 and latest['rate_state']==0 and all(latest[k]==0 for k in ['motor_left_pwm','motor_right_pwm','motor_ground_pwm'])
  supported=args.supported_running and latest['control_mode']==10 and latest['rate_state']==2 and latest['fault_reason']==0 and latest['driver_feedback_fresh']==1 and abs(latest['dual_roll_error_deg'])<3 and abs(latest['dual_roll_rate_dps'])<20
  latched=latest['control_mode']==10 and latest['rate_state']<0 and all(latest[k]==0 for k in ['motor_left_pwm','motor_right_pwm','motor_ground_pwm'])
  if not(stopped or supported or latched):raise RuntimeError('Neither stopped, zero-output fault, nor supported small-error running; no writes made')
  if latest['imu_status']!=1:raise RuntimeError('IMU not ready; no writes made')
  for k,v in updates.items():write(k,v)
  sample(.5)
  if any(abs(latest[k]-v)>1e-5 for k,v in updates.items()):raise RuntimeError('Parameter readback mismatch')
  print('PREPARED',json.dumps(latest),flush=True)
 else:
  print('OBSERVING; waiting for physical Mode10 Start, no parameter writes',flush=True)
  end=time.monotonic()+args.seconds;last_print=0
  while time.monotonic()<end:
   receive()
   if latest and time.monotonic()-last_print>=2:
    print(json.dumps({k:latest[k] for k in ['control_mode','rate_state','fault_reason','dual_roll_error_deg','dual_roll_rate_dps','dual_roll_pwm','fly_vel_kp','fly_vel_kd','fly_rate_ki']}),flush=True);last_print=time.monotonic()
   if latest and time.monotonic()-latest['host_monotonic']>.5:raise RuntimeError('Telemetry lost: use physical Stop')
except Exception as e:error=str(e);print('ERROR',error,flush=True)
finally:
 if peer and old:send('SUB '+old)
 if peer and oldrate:send('RATE '+oldrate)
 if peer and oldstream:send('STREAM '+oldstream)
 s.close();out=Path(__file__).resolve().parents[1]/'validation'/('fly-balance-'+('prepare-' if args.prepare else 'observe-')+time.strftime('%Y%m%d-%H%M%S')+'.json');out.write_text(json.dumps({'baseline':baseline,'latest':latest,'error':error,'rows':rows,'events':events},indent=2),encoding='utf-8');print('SAVED',out,flush=True)
