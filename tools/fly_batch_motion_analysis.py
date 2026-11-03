"""Angle-only analysis; makes no motor writes."""
import json,sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]);d=json.loads(p.read_text());result={}
for stage in dict.fromkeys(r['stage'] for r in d['rows']):
 rs=[r for r in d['rows'] if r['stage']==stage and r['rate_state']==2]
 if not rs:continue
 t=np.array([r['telemetry_ms'] for r in rs])*.001;e=np.array([r['dual_roll_error_deg'] for r in rs]);t-=t[0]
 smooth=np.convolve(e,np.ones(5)/5,mode='same') if len(e)>10 else e
 peaks=[];start=0
 for i in range(1,len(e)):
  if smooth[i]*smooth[i-1]<0:
   if t[i]-t[start]>.15:peaks.append(float(np.max(abs(smooth[start:i]))))
   start=i
 growth=float(np.median(peaks[-2:])/np.median(peaks[:2])) if len(peaks)>=4 and np.median(peaks[:2])>.2 else None
 result[stage]={'duration_s':float(t[-1]),'samples':len(e),'error_abs_peak_deg':float(max(abs(e))),'error_rms_deg':float(np.sqrt(np.mean(e*e))),'halfcycle_peaks_deg':peaks,'last_two_vs_first_two_peak_ratio':growth,'assessment':'insufficient cycles' if growth is None else 'growing observed motion' if growth>1.2 else 'decaying observed motion' if growth<.8 else 'roughly sustained observed motion','qualification':'Hand support and perturbation not measured; not a stability proof.'}
out=p.with_suffix('.motion-analysis.json');out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
