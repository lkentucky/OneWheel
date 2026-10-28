"""CRC-checked driver trace transfer over the existing Qcar WiFi UART bridge."""
from pathlib import Path
import binascii,csv,json,time
ROW_NAMES=('ticks','counts','angle','duty','active_a','active_b','active_c',
           'buffer_a','buffer_b','buffer_c','flags','event')
EVENT_NAMES=('index','received_ticks','dispatched_ticks','written_ticks','active_ticks',
             'left','right','target_a','target_b','target_c')

def decode_trace(packet):
    if not packet.startswith(b'MCAR TRACE TR,'):return None
    payload=packet[len(b'MCAR TRACE '):].strip()
    body,sep,crc=payload.rpartition(b'*')
    if not sep or len(crc)!=4 or binascii.crc_hqx(body,0xffff)!=int(crc,16):
        raise ValueError('Driver trace CRC mismatch')
    chunks=body.decode('ascii').split(';');head=chunks[0].split(',')
    if head[0]!='TR' or len(head)<3:raise ValueError('Trace header')
    kind=head[1];capture=int(head[2]);out=dict(kind=kind,capture_id=capture,raw=payload.decode('ascii'))
    if kind=='A':
        if len(head)!=4:raise ValueError('Trace ARM length')
        out['state']=int(head[3])
    elif kind=='M':
        if len(head)!=10:raise ValueError('Trace metadata length')
        out.update(zip(('state','count','event_count','clock_hz','encoder_cpr','max_irq_gap_ticks','overflow'),map(int,head[3:])))
    elif kind=='E':
        if len(head)!=13:raise ValueError('Trace event length')
        out['event']=dict(zip(EVENT_NAMES,map(int,head[3:])))
    elif kind=='D':
        if len(head)!=5:raise ValueError('Trace chunk length')
        out['first']=int(head[3]);out['count']=int(head[4]);out['rows']=[]
        if not 1<=out['count']<=8 or len(chunks)!=out['count']+1:raise ValueError('Trace chunk count')
        for row in chunks[1:]:
            fields=list(map(int,row.split(',')))
            if len(fields)!=len(ROW_NAMES):raise ValueError('Trace row length')
            out['rows'].append(dict(zip(ROW_NAMES,fields)))
    else:raise ValueError('Driver rejected trace request: '+payload.decode('ascii'))
    return out

class DriverTraceClient:
    def __init__(self,send,recv):
        self.send=send;self.recv=recv;self.capture_id=None;self.raw=[];self.invalid=0
        self.last_request=0.0;self.last_received=0.0
    def request(self,command,kind,index=None,attempts=8):
        for retry in range(attempts):
            gap=.08-(time.monotonic()-self.last_received)
            if gap>0:time.sleep(gap)
            self.last_request=time.monotonic()
            self.send(command);deadline=time.monotonic()+.7+.13*retry
            debug=[]
            while time.monotonic()<deadline:
                item=self.recv()
                if not item:continue
                packet,_=item
                if packet.startswith(b'MCAR ERR '):
                    if kind!='A' and packet.startswith(b'MCAR ERR DTRACE pending'):continue
                    raise RuntimeError(packet.decode('ascii').strip())
                try:r=decode_trace(packet)
                except (ValueError,UnicodeError) as exc:
                    self.invalid+=1;debug.append(str(exc)+" bytes="+str(len(packet)));continue
                if not r or r['kind']!=kind:continue
                if self.capture_id is not None and r['capture_id']!=self.capture_id:continue
                if kind=='D' and r['first']!=index:continue
                if kind=='E' and r['event']['index']!=index:continue
                self.last_received=time.monotonic()
                self.raw.append(r['raw']);return r
            print('TRACE RETRY',command,'attempt',retry+1,'invalid',debug[-2:],flush=True)
        raise RuntimeError('No CRC-checked driver response: '+command)
    def arm(self):
        r=self.request('DTRACE ARM','A',attempts=1)
        if r['state']!=1:raise RuntimeError('Driver trace did not arm')
        self.capture_id=r['capture_id'];print('DRIVER TRACE ARMED',self.capture_id,flush=True)
    def download(self,output):
        deadline=time.monotonic()+6
        while True:
            meta=self.request('DTRACE META','M')
            if meta['state'] in (3,4):break
            if meta['state'] not in (1,2) or time.monotonic()>=deadline:raise RuntimeError('Driver trace did not complete')
            time.sleep(.1)
        if not 1<=meta['count']<=8192 or not 1<=meta['event_count']<=16 or meta['overflow']:
            raise RuntimeError('Invalid/truncated driver capture')
        if meta['clock_hz']!=8000000 or meta['encoder_cpr']!=32768:raise RuntimeError('Unsupported driver trace timebase')
        events=[]
        for i in range(meta['event_count']):events.append(self.request('DTRACE EVENT '+str(i),'E',i)['event'])
        checkpoint=output.with_name(output.stem+'-driver-checkpoint.jsonl')
        identity=dict(capture_id=self.capture_id,count=meta['count'],events=events)
        cached={}
        if checkpoint.exists():
            lines=checkpoint.read_text('utf-8').splitlines()
            if lines and json.loads(lines[0]).get('identity')==identity:
                for line in lines[1:]:
                    try:v=json.loads(line)
                    except ValueError:continue
                    cached[v['first']]=v['rows']
        if not cached:checkpoint.write_text(json.dumps(dict(identity=identity))+'\n',encoding='utf-8')
        rows=[]
        while len(rows)<meta['count']:
            first=len(rows)
            if first in cached:r=dict(count=len(cached[first]),rows=cached[first])
            else:
                r=self.request('DTRACE ROW '+str(first),'D',first)
                with checkpoint.open('a',encoding='utf-8') as f:
                    f.write(json.dumps(dict(first=first,rows=r['rows'],crc_verified=True))+'\n')
            if first+r['count']>meta['count']:raise RuntimeError('Trace extends past metadata')
            rows.extend(r['rows'])
            if len(rows)%512==0 or len(rows)==meta['count']:print('DRIVER ROWS',len(rows),'/',meta['count'],flush=True)
        if any(b['ticks']<=a['ticks'] for a,b in zip(rows,rows[1:])):raise RuntimeError('Driver trace time is not monotonic')
        completed_span=(rows[-1]['ticks']-events[0]['dispatched_ticks'])/meta['clock_hz']
        if completed_span<7.5:raise RuntimeError('Driver capture stopped before the full 7.5s posttrigger window')
        meta['completed_posttrigger_s']=completed_span
        data=output.with_name(output.stem+'-driver.csv')
        with data.open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=('index',*ROW_NAMES));w.writeheader()
            for i,row in enumerate(rows):w.writerow(dict(index=i,**row))
        eventfile=output.with_name(output.stem+'-driver-events.csv')
        with eventfile.open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=EVENT_NAMES);w.writeheader();w.writerows(events)
        meta.update(events=events,rows_csv=str(data),events_csv=str(eventfile),invalid_crc_frames=self.invalid,
                    limits=['Register observations only; no physical PWM/gate voltage or current.',
                            'RX timestamps mark UART ISR reading the newline, not its physical arrival on the wire.',
                            'Active register first match is observed on the next encoder ISR.',
                            'Counts are the unfiltered sum of existing 20kHz encoder increments.'])
        output.with_name(output.stem+'-driver.json').write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
        output.with_name(output.stem+'-driver-wire.json').write_text(json.dumps(self.raw,indent=2)+'\n',encoding='utf-8')
        print('DRIVER TRACE SAVED',data,flush=True)
        return meta
