"""Recover a frozen driver capture; sends no motor/start/arm commands."""
from pathlib import Path
import argparse,json,socket,subprocess,sys,time
from driver_trace_client import DriverTraceClient

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--module-ip',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',8081));s.settimeout(.1)
    def send(command):
        s.sendto((command+'\n').encode(),(a.module_ip,5001))
        if not command.startswith('DTRACE ROW '):print('TX',command,flush=True)
    def recv():
        try:
            p,peer=s.recvfrom(8192)
            return (p,peer) if peer[0]==a.module_ip else None
        except socket.timeout:return None
    try:
        send('STREAM 0')
        end=time.monotonic()+2
        while time.monotonic()<end:
            item=recv()
            if item and item[0]==b'MCAR STREAM 0\n':break
        else:raise RuntimeError('No pause acknowledgement')
        c=DriverTraceClient(send,recv);meta=c.request('DTRACE META','M');c.capture_id=meta['capture_id']
        print('RECOVER',meta,flush=True)
        report=c.download(a.output)
        subprocess.run([sys.executable,str(Path(__file__).with_name('analyze_driver_trace.py')),str(a.output.with_name(a.output.stem+'-driver.json'))],check=True)
        if a.output.with_suffix('.json').exists():
            old=json.loads(a.output.with_suffix('.json').read_text('utf-8'))
            old['initial_transfer_error']=old.get('error','');old['error']='';old['driver_trace']=report
            a.output.with_suffix('.json').write_text(json.dumps(old,indent=2)+'\n',encoding='utf-8')
    finally:
        send('STREAM 1');s.close()
if __name__=='__main__':main()
