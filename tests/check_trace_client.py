"""Validate host transfer against packets serialized by the actual driver C."""
from pathlib import Path
import importlib.util,subprocess,sys,tempfile
r=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('driver_trace_client',r/'tools/driver_trace_client.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
with tempfile.TemporaryDirectory(prefix='trace-client-test-') as d:
    w=Path(d);fixture=w/'simulated-wire.txt'
    subprocess.run([sys.executable,str(r/'tests/check_driver_trace.py'),str(fixture)],check=True)
    packets=[b'MCAR TRACE '+p+b'\n' for p in fixture.read_bytes().splitlines()]
    decoded=[]
    for p in packets:
        try:v=m.decode_trace(p)
        except ValueError:continue
        if v and v['capture_id']==1:decoded.append((v,p))
    arm=next(p for v,p in decoded if v['kind']=='A' and v['state']==1)
    meta=next(p for v,p in decoded if v['kind']=='M' and v['state']==3)
    events={v['event']['index']:p for v,p in decoded if v['kind']=='E'}
    chunks={v['first']:p for v,p in decoded if v['kind']=='D'}
    queue=[];sent=[]
    def send(command):
        sent.append(command)
        if command=='DTRACE ARM':packet=arm
        elif command=='DTRACE META':packet=meta
        elif command.startswith('DTRACE EVENT '):packet=events[int(command.split()[-1])]
        elif command.startswith('DTRACE ROW '):packet=chunks[int(command.split()[-1])]
        else:raise AssertionError(command)
        if len(sent)==1:queue.append(b'MCAR TRACE TR,A,1,1*0000\n') # Bad CRC must be ignored.
        queue.append(packet)
    def recv():return (queue.pop(0),('127.0.0.1',5001)) if queue else None
    client=m.DriverTraceClient(send,recv);client.arm();report=client.download(w/'simulated.csv')
    assert report['event_count']==4 and report['count']>7500 and client.invalid==1
    assert (w/'simulated-driver.csv').is_file() and (w/'simulated-driver-events.csv').is_file()
    assert all(v['written_ticks']>=v['dispatched_ticks']>=v['received_ticks'] for v in report['events'])
    assert len([v for v in sent if v=='DTRACE ARM'])==1
    assert m.decode_trace(b'MCAR RATE 5\n') is None
    analyzed=subprocess.run([sys.executable,str(r/'tools/analyze_driver_trace.py'),str(w/'simulated-driver.json')],check=True,capture_output=True,text=True)
    assert (w/'simulated-driver-overview.png').is_file() and (w/'simulated-driver-motion.png').is_file()

print('PASS: real C CRC frames -> host ARM/metadata/events/all chunks -> complete chronological CSV; simulation only')
