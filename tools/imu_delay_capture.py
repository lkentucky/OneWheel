"""Read-only IMU timing capture. Restores telemetry subscription/rate/stream."""
import socket,time,struct,json,math
from pathlib import Path
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',8081));s.settimeout(.1)
peer=('192.168.0.113',5001);old=None;oldrate=None;oldstream=None;names=None;rows=[];events=[];confirmed=False
channels='telemetry_ms,imu_seq,imu_dt_ms,imu_solve_us,imu_status,roll_deg,pitch_deg,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,imu_pre_ax_g,imu_pre_ay_g,imu_pre_az_g,imu_pre_gx_dps,imu_pre_gy_dps,imu_accel_used,ekf_nis,imu_r_scale,imu_accel_vib_g,imu_reacquiring,control_mode,rate_state,fault_reason,motor_left_pwm,motor_right_pwm,motor_ground_pwm'
def send(t):s.sendto((t+'\n').encode(),peer)
def receive(capture=False):
 global old,oldrate,oldstream,names,confirmed,peer
 try:p,a=s.recvfrom(8192)
 except socket.timeout:return
 if old is None and p.startswith(b'MCAR '):peer=(a[0],5001)
 if a[0]!=peer[0]:return
 if p.startswith(b'MCAR '):
  t=p.decode().strip();events.append(t)
  if t.startswith('MCAR SUB '):
   if old is None:old=t[9:]
   names=t[9:].split(',');confirmed=t[9:]==channels
  if not capture and t.startswith('MCAR RATE '):oldrate=t.split()[-1]
  if not capture and t.startswith('MCAR STREAM '):oldstream=t.split()[-1]
  if t.startswith('MCAR ERR'):raise RuntimeError(t)
 elif capture and confirmed and len(p)==4*(len(names)+1) and p[-4:]==b'\x00\x00\x80\x7f':
  v=struct.unpack('<%df'%len(names),p[:-4])
  if all(map(math.isfinite,v)):
   r=dict(zip(names,v));r['host_monotonic']=time.monotonic();rows.append(r)
try:
 s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
 for ip in ['192.168.0.113','192.168.0.119','192.168.0.255']:s.sendto(b'GET?\n',(ip,5001))
 end=time.monotonic()+8
 while time.monotonic()<end:
  receive()
  if old is not None and oldrate is not None and oldstream is not None:break
 if old is None:raise RuntimeError('No board reply')
 send('SUB '+channels);send('RATE 5');send('STREAM 1')
 print('CAPTURE START: 30 seconds; no motor/control writes',flush=True)
 end=time.monotonic()+30;last=time.monotonic()
 while time.monotonic()<end:
  receive(True)
  if time.monotonic()-last>5:
   print('FRAMES',len(rows),'LATEST', {k:rows[-1][k] for k in ['roll_deg','pitch_deg','imu_dt_ms','control_mode','rate_state']} if rows else {},flush=True);last=time.monotonic()
finally:
 if old:send('SUB '+old)
 if oldrate:send('RATE '+oldrate)
 if oldstream:send('STREAM '+oldstream)
 s.close()
 out=Path(__file__).resolve().parents[1]/'validation'/('imu-delay-'+time.strftime('%Y%m%d-%H%M%S')+'.json');out.write_text(json.dumps({'rows':rows,'events':events},indent=2),encoding='utf-8');print('SAVED',out,flush=True)
