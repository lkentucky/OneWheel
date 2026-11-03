"""Three supported Mode10 angle-gain trials. Never remotely starts motors."""
import json,socket,struct,time,math
from pathlib import Path
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',8081));s.settimeout(.03);s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
peer=None;old=None;oldrate=None;oldstream=None;names=None;latest=None;rows=[];events=[];stage='baseline';error='';baseline=None
channels='telemetry_ms,control_mode,rate_state,fault_reason,imu_status,imu_seq,fly_angle_kp,fly_angle_kd,fly_rate_kp,fly_rate_ki,fly_vel_kp,fly_vel_kd,dual_roll_actual_deg,dual_roll_target_deg,dual_roll_error_deg,dual_roll_target_rate_dps,dual_roll_rate_dps,dual_roll_pwm,motor_left_pwm,motor_right_pwm,motor_ground_pwm,flywheel_left_speed_raw,flywheel_right_speed_raw,driver_feedback_fresh,driver_feedback_age_ms'
root=Path(__file__).resolve().parents[1];out=root/'validation'/('fly-batch-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
def send(c):
 s.sendto((c+'\n').encode(),peer);events.append({'t':time.monotonic(),'stage':stage,'tx':c})
def receive():
 global peer,old,oldrate,oldstream,names,latest
 try:p,a=s.recvfrom(8192)
 except socket.timeout:return None
 if peer and a[0]!=peer[0]:return None
 if p.startswith(b'MCAR '):
  peer=(a[0],5001);v=p.decode().strip();events.append({'t':time.monotonic(),'stage':stage,'rx':v})
  if v.startswith('MCAR SUB '):
   if old is None:old=v[9:]
   names=v[9:].split(',')
  if v.startswith('MCAR RATE ') and oldrate is None:oldrate=v.split()[-1]
  if v.startswith('MCAR STREAM ') and oldstream is None:oldstream=v.split()[-1]
  if v.startswith('MCAR ERR'):raise RuntimeError(v)
  return v
 if names==channels.split(',') and len(p)==4*(len(names)+1) and p[-4:]==b'\x00\x00\x80\x7f':
  vals=struct.unpack('<%df'%len(names),p[:-4])
  if not all(map(math.isfinite,vals)):raise RuntimeError('Nonfinite telemetry')
  latest=dict(zip(names,vals));latest.update(host_monotonic=time.monotonic(),stage=stage);rows.append(latest)
 return None
def guard():
 if latest is None or time.monotonic()-latest['host_monotonic']>.5:raise RuntimeError('Telemetry stale; PHYSICAL STOP REQUIRED')
 r=latest
 if r['control_mode']!=10 or r['rate_state']!=2 or r['fault_reason']!=0 or r['imu_status']!=1 or r['driver_feedback_fresh']!=1:raise RuntimeError('Mode/state/fault/feedback changed; no more gain changes')
 if abs(r['dual_roll_error_deg'])>6 or abs(r['dual_roll_rate_dps'])>80 or abs(r['dual_roll_pwm'])>6500:raise RuntimeError('Response outside trial envelope; PHYSICAL STOP REQUIRED')
 if any(abs(r[k])>1e-6 for k in ['fly_vel_kp','fly_vel_kd','fly_rate_ki']):raise RuntimeError('Other parameters changed')
 if baseline and any(abs(r[k]-baseline[k])>1e-5 for k in ['fly_rate_kp','fly_angle_kd']):raise RuntimeError('Concurrent gain change')
def sample(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:receive();guard()
def write(k,v):
 for i in range(3):
  guard();send(f'[slider,{k},{v}]');end=time.monotonic()+.5
  while time.monotonic()<end:
   reply=receive();guard()
   if reply and reply.startswith('MCAR SLIDER '+k+' ') and abs(float(reply.split()[-1])-v)<1e-5:return
 raise RuntimeError('Parameter acknowledgement missing')
try:
 for ip in ['192.168.0.114','192.168.0.113','192.168.0.119','192.168.0.255']:s.sendto(b'GET?\n',(ip,5001))
 end=time.monotonic()+5
 while time.monotonic()<end:
  receive()
  if old and oldrate and oldstream:break
 if old is None:raise RuntimeError('No board reply')
 send('SUB '+channels);send('RATE 10');send('STREAM 1')
 end=time.monotonic()+1
 while time.monotonic()<end:receive()
 stage='waiting_supported';deadline=time.monotonic()+30;steady=None;printed=0
 while time.monotonic()<deadline:
  receive()
  if latest is not None:
   r=latest;fresh=time.monotonic()-r['host_monotonic']<.2
   ready=fresh and r['control_mode']==10 and r['rate_state']==2 and r['fault_reason']==0 and r['imu_status']==1 and r['driver_feedback_fresh']==1 and abs(r['dual_roll_error_deg'])<2 and abs(r['dual_roll_rate_dps'])<15 and abs(r['dual_roll_pwm'])<1500
   if time.monotonic()-printed>3:
    print('WAIT',json.dumps({k:r[k] for k in ['control_mode','rate_state','dual_roll_error_deg','dual_roll_rate_dps','motor_ground_pwm']}),flush=True);printed=time.monotonic()
   if ready:
    if steady is None:steady=time.monotonic()
    if time.monotonic()-steady>.4:break
   else:steady=None
 else:raise RuntimeError('No supported near-upright window; no gains changed')
 guard();baseline=dict(latest);sample(1)
 for gain in [20,15,10]:
  stage='settle_'+str(gain);end=time.monotonic()+8;steady=None
  while time.monotonic()<end:
   receive();guard()
   if abs(latest['dual_roll_error_deg'])<2 and abs(latest['dual_roll_rate_dps'])<15 and abs(latest['dual_roll_pwm'])<1500:
    if steady is None:steady=time.monotonic()
    if time.monotonic()-steady>.3:break
   else:steady=None
  else:raise RuntimeError('No supported quiet transition window')
  write('fly_angle_kp',gain);sample(.4);stage='Kp_'+str(gain)
  print('TRIAL',stage,'8 seconds',flush=True);sample(8)
  if abs(latest['fly_angle_kp']-gain)>1e-5:raise RuntimeError('Readback changed')
  print('COMPLETE',stage,flush=True)
except Exception as e:error=str(e);print('ABORT',error,flush=True)
finally:
 if peer and old:send('SUB '+old)
 if peer and oldrate:send('RATE '+oldrate)
 if peer and oldstream:send('STREAM '+oldstream)
 s.close();out.write_text(json.dumps({'baseline':baseline,'latest':latest,'error':error,'rows':rows,'events':events},indent=2),encoding='utf-8');print('SAVED',out,flush=True)
