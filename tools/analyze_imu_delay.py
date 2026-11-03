import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]) if len(sys.argv)>1 else max((p for p in (Path(__file__).resolve().parents[1]/'validation').glob('imu-delay-*.json') if '.analysis.' not in p.name),key=lambda p:p.stat().st_mtime)
rows=json.loads(p.read_text())['rows'];d=[]
for r in rows:
 if not d or r['imu_seq']!=d[-1]['imu_seq']:d.append(r)
if len(d)<50:raise SystemExit('Insufficient samples')
a={k:np.array([r[k] for r in d]) for k in d[0]};t=(a['telemetry_ms']-a['telemetry_ms'][0])*.001
dt=np.median(np.diff(t));grid=np.arange(t[0],t[-1],dt)
# Positive lag: EKF follows accelerometer-derived angle.
def lag(x,y):
 x=np.interp(grid,t,x);y=np.interp(grid,t,y);scores=[]
 for n in range(-int(.3/dt),int(.3/dt)+1):
  xx=x[:-n] if n>0 else x[-n:] if n<0 else x
  yy=y[n:] if n>0 else y[:n] if n<0 else y
  scores.append((float(np.corrcoef(xx,yy)[0,1]),float(n*dt*1000)))
 return max(scores)
accroll=np.degrees(np.arctan2(a['accel_y_g'],a['accel_z_g']))
accpitch=np.degrees(np.arctan2(-a['accel_x_g'],np.hypot(a['accel_y_g'],a['accel_z_g'])))
norm=np.sqrt(sum(a[k]**2 for k in ['accel_x_g','accel_y_g','accel_z_g']))
report={'frames':len(rows),'unique_imu_samples':len(d),'duration_s':float(t[-1]),'telemetry_interval_ms_median':float(dt*1000),'telemetry_gap_ms_max':float(np.max(np.diff(t))*1000),'imu_dt_ms_median':float(np.median(a['imu_dt_ms'])),'imu_dt_ms_max':float(np.max(a['imu_dt_ms'])),'solve_us_p95':float(np.percentile(a['imu_solve_us'],95)),'accel_used_fraction':float(np.mean(a['imu_accel_used']==1)),'r_scale_median':float(np.median(a['imu_r_scale'])),'accel_norm_deviation_p95_g':float(np.percentile(abs(norm-1),95)),'modes':np.unique(a['control_mode']).tolist(),'states':np.unique(a['rate_state']).tolist(),'axes':{}}
fig,axs=plt.subplots(4,1,figsize=(12,11),sharex=True)
for i,(name,aa,gkey) in enumerate([('roll_deg',accroll,'gyro_x_dps'),('pitch_deg',accpitch,'gyro_y_dps')]):
 angle=a[name];corr,delay=lag(aa,angle)
 deriv=np.gradient(angle,t);gc,gd=lag(a[gkey],deriv)
 gyro=a[gkey]-np.median(a[gkey][:min(50,len(t))])
 integral=angle[0]+np.r_[0,np.cumsum((gyro[1:]+gyro[:-1])*.5*np.diff(t))]
 report['axes'][name]={'ekf_span_deg':float(np.ptp(angle)),'accel_span_deg':float(np.ptp(aa)),'accel_vs_ekf_best_correlation':corr,'dynamic_excitation_sufficient':bool(np.ptp(angle)>=2),'ekf_lag_vs_accel_ms':delay if np.ptp(angle)>=2 and corr>.8 else None,'gyro_vs_angle_derivative_correlation':gc,'angle_derivative_lag_vs_gyro_ms':gd if np.ptp(angle)>=2 and gc>.8 else None,'initial_bias_removed_gyro_integration_residual_end_deg':float(angle[-1]-integral[-1]),'accel_ekf_rmse_deg':float(np.sqrt(np.mean((aa-angle)**2)))}
 axs[i].plot(t,aa,label='Raw accel tilt',alpha=.65);axs[i].plot(t,angle,label='EKF '+name);axs[i].plot(t,integral,label='Gyro integral (initial bias removed; diagnostic only)',alpha=.65);axs[i].set_ylabel('deg');axs[i].legend()
axs[2].plot(t,norm,label='Accel norm / g');axs[2].plot(t,a['imu_r_scale'],label='EKF R scale');axs[2].legend()
axs[3].plot(t,a['imu_dt_ms'],label='IMU dt / ms');axs[3].plot(t,a['imu_accel_used'],label='Accel used');axs[3].set_xlabel('Board telemetry time / s');axs[3].legend()
for ax in axs:ax.grid(alpha=.3)
fig.suptitle('IMU timing comparison (telemetry resolution; acceleration includes translation)');fig.tight_layout();fig.savefig(p.with_suffix('.png'),dpi=150)
p.with_suffix('.analysis.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print('PLOT',p.with_suffix('.png'))
