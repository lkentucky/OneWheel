"""Install the audited two-file driver overlay into its original IAR project."""
from pathlib import Path
import argparse,hashlib
BASELINES = {'motor_driver_uart_control.c': '95417dbd793a7f5ba36863bd303a36faf05a23f18ddd596d663aec134ba69360', 'motor_control.c': '76b579b82baaf1494199757c5d2678691b265a2a2e5d85db59749acf8f74f647'}
def canonical(data):
    try:text=data.decode('utf-8-sig')
    except UnicodeDecodeError:text=data.decode('gbk')
    return ('\n'.join(line.rstrip() for line in text.splitlines()).rstrip()+'\n').encode('utf-8')
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args();source=Path(__file__).resolve().parent/'code'
    pending=[]
    for name,baseline in BASELINES.items():
        target=args.project/'code'/name
        current=hashlib.sha256(canonical(target.read_bytes())).hexdigest()
        update=canonical((source/name).read_bytes())
        fixed=hashlib.sha256(update).hexdigest()
        if current not in (baseline,fixed):
            raise SystemExit('Refusing to overwrite an unrecognized modified file: '+str(target))
        pending.append((target,update,current==fixed))
    for target,update,done in pending:
        if not args.check and not done:
            target.write_bytes(update.decode('utf-8').replace('\n','\r\n').encode('gbk'))
        print(('already fixed: ' if done else 'compatible: ' if args.check else 'updated: ')+str(target))
if __name__=='__main__':main()
