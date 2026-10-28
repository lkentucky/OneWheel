"""Analyze a downloaded driver trace; never sends motor commands."""
from pathlib import Path
import argparse,csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('metadata',type=Path);a=p.parse_args()
meta=json.loads(a.metadata.read_text('utf-8'));src=Path(meta['rows_csv'])
rows=[{k:int(v) for k,v in r.items()} for r in csv.DictReader(src.open(encoding='utf-8-sig'))]
if not rows or meta['state'] not in (3,4) or meta['overflow']:raise SystemExit('Incomplete/overflowed capture')
hz=meta['clock_hz'];cpr=meta['encoder_cpr'];events=meta['events'];origin=events[0]['dispatched_ticks']
t=np.array([(r['ticks']-origin)/hz for r in rows]);counts=np.array([r['counts'] for r in rows],dtype=float)
period=np.diff(t)*1000
# Explicit causal 5ms count difference; no stock driver speed low-pass filter.
j=np.searchsorted(t,t-.005,side='right')-1;j=np.maximum(j,0)
dt=t-t[j];speed=np.divide((counts-counts[j])*60/cpr,dt,out=np.full(len(t),np.nan),where=dt>0)
latencies=[]
for e in events:
    def interval(end,start):return None if e[end]==4294967295 or e[start]==4294967295 else (e[end]-e[start])*1e6/hz
    latencies.append(dict(event=e['index'],left=e['left'],right=e['right'],
        uart_end_to_dispatch_us=interval('dispatched_ticks','received_ticks'),
        dispatch_to_buffer_write_us=interval('written_ticks','dispatched_ticks'),
        write_to_active_observed_us=interval('active_ticks','written_ticks'),
        uart_end_to_active_observed_us=interval('active_ticks','received_ticks')))
fig,bar=plt.subplots(figsize=(11,4.5))
y=np.arange(len(latencies));left=np.zeros(len(latencies))
colors=['#2563eb','#f59e0b','#16a34a']
for key,label,color in zip(['uart_end_to_dispatch_us','dispatch_to_buffer_write_us','write_to_active_observed_us'],
    ['UART read -> dispatch','Dispatch -> PWM buffer write','Buffer write -> active register observed'],colors):
    values=np.array([v[key] if v[key] is not None else 0 for v in latencies])
    bar.barh(y,values,left=left,label=label,color=color)
    left+=values
for i,total in enumerate(left):bar.text(total+1,i,f'{total:.1f} us',va='center',fontsize=11)
labels=[];previous=0
for e in events:
    labels.append(f"{abs(previous)/100:g}% -> {abs(e['left'])/100:g}%")
    previous=e['left']
bar.set_yticks(y,labels);bar.invert_yaxis();bar.set_xlim(0,max(110,float(max(left))+15))
bar.set_xlabel('Microseconds from UART ISR reading command newline');bar.grid(axis='x',alpha=.2)
bar.set_title('Driver-side command execution: all four steps below 0.1 ms',fontsize=14)
bar.legend(loc='upper center',bbox_to_anchor=(.5,-.15),ncol=1,fontsize=9)
fig.tight_layout();latencyplot=a.metadata.with_name(a.metadata.stem+'-latency.png');fig.savefig(latencyplot,dpi=170,bbox_inches='tight');plt.close(fig)

baseline=counts[(t>=-.08)&(t<=-.005)]
base=float(np.median(baseline)) if len(baseline) else counts[0]
mad=float(np.median(np.abs(baseline-base))) if len(baseline) else 0
threshold=max(3.0,6*mad)
onset=None
for i in range(len(t)-2):
    if 0<=t[i]<.5 and np.all(np.abs(counts[i:i+3]-base)>threshold):onset=float(t[i]*1000);break

fig,ax=plt.subplots(3,1,figsize=(13,9),sharex=True)
ax[0].step(t,[r['duty']/100 for r in rows],where='post',label='Driver accepted command',color='#2563eb');ax[0].set_ylabel('Command (%)');ax[0].legend()
for name,color in [('a','#ea580c'),('b','#16a34a'),('c','#7c3aed')]:
    ax[1].plot(t,[r['active_'+name]/40 for r in rows],lw=.7,color=color,label='Active phase '+name.upper())
ax[1].set_ylabel('Active compare width (%)');ax[1].legend(loc='upper right')
ax[2].plot(t,speed,color='#ea580c',lw=1,label='Encoder speed, causal 5ms difference');ax[2].set_ylabel('RPM');ax[2].set_xlabel('Time from first command dispatch (s)');ax[2].legend()
for q in ax:
    q.grid(alpha=.25)
    for e in events:q.axvline((e['dispatched_ticks']-origin)/hz,ls='--',color='#64748b',lw=.7)
fig.suptitle('Driver-side firmware trace | UART, PWM registers and raw encoder',fontsize=15)
fig.text(.5,.01,'Register observations are not physical PWM/current measurements. Speed uses an explicitly stated 5ms difference window.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.03,1,.95));overview=a.metadata.with_name(a.metadata.stem+'-overview.png');fig.savefig(overview,dpi=160);plt.close(fig)

fig,ax=plt.subplots(2,1,figsize=(12,7))
mask=(t>=-.01)&(t<=.03)
ax[0].plot(t[mask]*1000,(counts[mask]-base)*360/cpr,'.-',ms=3,color='#ea580c',label='Raw encoder displacement')
ax[0].axhline(threshold*360/cpr,color='#94a3b8',ls=':',label='Position-change detection threshold')
ax[0].axhline(-threshold*360/cpr,color='#94a3b8',ls=':');ax[0].set_ylabel('Mechanical degrees');ax[0].legend()
ax[0].set_xlabel('Time from first dispatch (ms), first 30 ms')
mask=(t>=-.01)&(t<=.2)
ax[1].plot(t[mask]*1000,speed[mask],'.-',ms=3,color='#16a34a');ax[1].set_ylabel('5ms-difference RPM');ax[1].set_xlabel('Time from first dispatch (ms)')
for q in ax:
    q.axvline(0,color='#2563eb',ls='--');q.grid(alpha=.25)
    if onset is not None:q.axvline(onset,color='#ea580c',ls=':')
fig.suptitle('First step: encoder motion after command dispatch',fontsize=15)
fig.tight_layout();zoom=a.metadata.with_name(a.metadata.stem+'-motion.png');fig.savefig(zoom,dpi=160);plt.close(fig)
summary=dict(metadata=str(a.metadata),row_count=len(rows),event_latencies=latencies,
    median_sample_ms=float(np.median(period)),max_sample_ms=float(np.max(period)),
    max_encoder_irq_gap_us=meta['max_irq_gap_ticks']*1e6/hz,
    nonzero_flag_rows=sum(r['flags']!=0 for r in rows),
    first_position_change_detected_ms=onset,position_threshold_counts=threshold,
    overview=str(overview),motion_plot=str(zoom),latency_plot=str(latencyplot),
    limits=['Position-change time includes inertia, friction, encoder resolution and sampling; it is not torque delay.',
            'No current measured. Active register equality does not verify gate drivers or phase voltages.'])
a.metadata.with_name(a.metadata.stem+'-analysis.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
print(json.dumps(summary,indent=2))
