"""Actuator/rate-only analysis; separate from angle-peak analysis."""
import json,sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]);d=json.loads(p.read_text());result={}
for stage in dict.fromkeys(r['stage'] for r in d['rows']):
 rs=[r for r in d['rows'] if r['stage']==stage and r['rate_state']==2]
 if not rs:continue
 u=np.array([r['dual_roll_pwm'] for r in rs]);rate=np.array([r['dual_roll_rate_dps'] for r in rs]);target=np.array([r['dual_roll_target_rate_dps'] for r in rs]);g=np.array([r['motor_ground_pwm'] for r in rs])
 signs=np.sign(u[abs(u)>100]);reversals=int(np.sum(signs[1:]!=signs[:-1]))
 result[stage]={'samples':len(rs),'fly_pwm_abs_peak':float(max(abs(u))),'fly_7500_saturation_fraction':float(np.mean(abs(u)>=7499)),'ground_7500_saturation_fraction':float(np.mean(abs(g)>=7499)),'rate_abs_peak_dps':float(max(abs(rate))),'rate_tracking_rmse_dps':float(np.sqrt(np.mean((target-rate)**2))),'pwm_sign_changes_above_100':reversals,'feedback_stale_samples':sum(r['driver_feedback_fresh']!=1 for r in rs),'qualification':'PWM is command, not measured torque/current. Ground motion and hand support can couple axes.'}
out=p.with_suffix('.output-analysis.json');out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
