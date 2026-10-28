"""Analyze an existing real Mode8 CSV; never sends control commands."""
from pathlib import Path
import argparse,csv,json,statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

p=argparse.ArgumentParser();p.add_argument('csv',type=Path);a=p.parse_args()
rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(a.csv.open(encoding='utf-8-sig'))]
run=[r for r in rows if r['step_state'] in (1,2) and r['step_stage']>0]
if not run:raise SystemExit('No real sequence rows')
f=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=f.get_name();plt.rcParams['axes.unicode_minus']=False
x=[r['step_ms']/1000 for r in run]
# Completion rows keep step_ms at 7500; use just the first completion sample.
end=next((i+1 for i,r in enumerate(run) if r['step_state']==2),len(run));run=run[:end];x=x[:end]
metrics=[]
for st,start,stop in [(2,1000,2500),(3,2500,4000),(4,4000,5500),(5,5500,7500)]:
    seg=[r for r in run if r['step_stage']==st]
    base=statistics.median(r['flywheel_left_speed_raw'] for r in run if start-250<=r['step_ms']<start)
    target=statistics.median(r['flywheel_left_speed_raw'] for r in run if stop-300<=r['step_ms']<stop)
    delta=target-base;sgn=1 if delta>=0 else -1
    metric=dict(stage=st,command=seg[0]['motor_left_pwm'],baseline_rpm=base,final_window_median_rpm=target)
    for name,frac in [('t10_ms',.1),('t50_ms',.5),('t90_first_crossing_ms',.9)]:
        hit=next((r for r in seg if sgn*(r['flywheel_left_speed_raw']-base)>=frac*abs(delta)),None)
        metric[name]=hit['step_ms']-start if hit else None
    metric['first_observed_command_ms']=seg[0]['step_ms']-start
    metric['first_observed_speed_change_ms']=next((r['step_ms']-start for r in seg if abs(r['flywheel_left_speed_raw']-base)>=max(5,.03*abs(delta))),None)
    metrics.append(metric)

fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True)
ax[0].step(x,[abs(r['motor_left_pwm'])/100 for r in run],where='post',color='#2563eb',lw=2,label='Left command magnitude')
ax[0].plot(x,[r['motor_right_pwm']/100 for r in run],color='#94a3b8',label='Right / ground = 0')
ax[0].set_ylabel('PWM command (%)');ax[0].set_ylim(-.4,7);ax[0].legend(loc='upper right')
ax[1].plot(x,[r['flywheel_left_speed_raw'] for r in run],color='#ea580c',lw=2,label='Left filtered feedback')
ax[1].plot(x,[r['flywheel_right_speed_raw'] for r in run],color='#64748b',label='Right feedback')
ax[1].set_ylabel('Driver speed (RPM)');ax[1].legend(loc='upper right')
ax[2].plot(x,[r['driver_feedback_age_ms'] for r in run],color='#16a34a',lw=1)
ax[2].set_ylabel('Feedback age (ms)');ax[2].set_xlabel('Local sequence time (s)')
for aa in ax:
    aa.grid(alpha=.25)
    for t in [1,2.5,4,5.5]:aa.axvline(t,ls='--',color='#94a3b8',lw=.8)
fig.suptitle(f'Mode8 real left-flywheel step | {len(rows)} received frames',fontsize=15)
fig.text(.5,.015,'Command magnitude shown; transmitted left command is negative. Physical PWM / current were not measured.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.04,1,.95));main=a.csv.with_name(a.csv.stem+'-overview.png');fig.savefig(main,dpi=170);plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True,sharey=True)
labels=['0 -> 3%','3% -> 6%','6% -> 3%','3% -> 0']
for ax,m,label,start,stop in zip(axes.flat,metrics,labels,[1000,2500,4000,5500],[2500,4000,5500,7500]):
    seg=[r for r in run if start-100<=r['step_ms']<min(stop,start+1000)]
    base=m['baseline_rpm'];delta=m['final_window_median_rpm']-base
    ax.plot([r['step_ms']-start for r in seg],[(r['flywheel_left_speed_raw']-base)/delta*100 for r in seg],'.-',ms=3,color='#ea580c')
    ax.axvline(0,color='#2563eb',ls='--',label='Scheduled command edge')
    ax.axhline(90,color='#94a3b8',ls=':');ax.axhline(10,color='#94a3b8',ls=':')
    for name in ['t10_ms','t90_first_crossing_ms']:
        t=m[name]
        if t is not None:
            hit=next(r for r in seg if r['step_ms']-start==t)
            ax.plot(t,(hit['flywheel_left_speed_raw']-base)/delta*100,'o',color='#2563eb')
    ax.set_title(f"{label} | t10={m['t10_ms']:.0f} ms, t90={m['t90_first_crossing_ms']:.0f} ms")
    ax.set_xlim(-100,1000);ax.set_ylim(-10,115);ax.grid(alpha=.25)
    ax.set_xlabel('Time from scheduled command edge (ms)');ax.set_ylabel('Completed speed change (%)')
fig.suptitle('Filtered speed response | First threshold crossings, not torque response or settling time',fontsize=13)
fig.text(.5,.01,'Telemetry gaps up to 34.5 ms; stock speed LPF about 19.5 ms. Do not interpret t10 as pure actuation delay.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.035,1,.94));detail=a.csv.with_name(a.csv.stem+'-response.png');fig.savefig(detail,dpi=170);plt.close(fig)
report={'source_csv':str(a.csv),'real_board_data':True,'metrics':metrics,'overview':str(main),'response':str(detail),'note':'Nominal controller tick timing; filtered returned speed. First crossings are quantized by telemetry and are not torque/PWM delay measurements.'}
a.csv.with_name(a.csv.stem+'-analysis.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
