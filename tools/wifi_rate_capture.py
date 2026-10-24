"""Collect real board telemetry using the existing MCAR host receiver/parser.
Only telemetry subscription commands are sent; this script never starts motors.
"""
from pathlib import Path
import argparse
import csv
import json
import queue
import sys
import time

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-dir",type=Path,default=Path.home()/"Desktop/imu/route_run_update")
    parser.add_argument("--module-ip")
    parser.add_argument("--profile",type=Path,default=ROOT/"tools/gyro_telemetry.json")
    parser.add_argument("--port",type=int,default=8081)
    parser.add_argument("--wait",type=float,default=120)
    parser.add_argument("--seconds",type=float,default=20)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    sys.path.insert(0,str(args.host_dir))
    from udp_receiver import UdpReceiver
    from mcar_telemetry import Subscription, control_reply, validate_frame
    from packet_parser import FieldRule, parse_packet
    profile=json.loads(args.profile.read_text("utf-8"))
    names=profile["channels"]
    rules=[FieldRule(name,"float32",4*i) for i,name in enumerate(names)]
    output=args.output or ROOT/"validation"/("wifi-rate-live-"+time.strftime("%Y%m%d-%H%M%S")+".csv")
    output.parent.mkdir(parents=True,exist_ok=True)
    receiver=UdpReceiver(); receiver.start("0.0.0.0",args.port)
    sub=Subscription(); peer=args.module_ip; started=time.monotonic(); capture_started=None
    count=invalid=0; states={}; ranges={}; unique_seq=set(); max_gap=0; last_host_time=None; fatal=""
    def send(command):
        if command:
            print("TX",command.strip(),flush=True)
            receiver.send_bound(command.encode("ascii"),peer,profile["module_port"])
    print(f"LISTEN 0.0.0.0:{args.port}; waiting for a real board datagram",flush=True)
    try:
        with output.open("w",newline="",encoding="utf-8") as file:
            writer=csv.DictWriter(file,fieldnames=["host_time_s","source_ip","source_port"]+names)
            writer.writeheader()
            if peer:send(sub.begin(",".join(names),profile["period_ms"],time.monotonic()))
            while True:
                now=time.monotonic()
                if capture_started is None and now-started>=args.wait:break
                if capture_started is not None and now-capture_started>=args.seconds:break
                retry=sub.poll(now)
                if sub.error:fatal=sub.error; break
                send(retry)
                try:kind,payload=receiver.events.get(timeout=.2)
                except queue.Empty:continue
                if kind=="error":fatal=str(payload); break
                if kind!="packet":continue
                host_time,packet,address=payload
                if peer and address[0]!=peer:continue
                if not peer:
                    if not (packet.startswith(b"MCAR ") or packet.endswith(b"\x00\x00\x80\x7f")):continue
                    peer=address[0]
                    print("BOARD",address,flush=True)
                    send(sub.begin(",".join(names),profile["period_ms"],now))
                    continue
                try:reply=control_reply(packet)
                except (ValueError,UnicodeError):invalid+=1;continue
                if reply:
                    print("RX",reply,flush=True)
                    send(sub.acknowledge(*reply,now))
                    if sub.error:fatal=sub.error;break
                    if not sub.pending and capture_started is None:
                        capture_started=now
                        print("SUBSCRIPTION CONFIRMED; collecting real frames",flush=True)
                    continue
                if sub.pending or capture_started is None:continue
                try:validate_frame(packet,len(names))
                except ValueError:invalid+=1;continue
                values,errors=parse_packet(packet,rules)
                if errors:invalid+=1;continue
                writer.writerow(dict(host_time_s=host_time,source_ip=address[0],source_port=address[1],**values))
                count+=1; unique_seq.add(values["rate_seq"])
                state=str(int(values["rate_state"]));states[state]=states.get(state,0)+1
                for name in ["rate_actual_dps","rate_pwm","roll_deg","pitch_deg","flywheel_left_speed_raw","flywheel_right_speed_raw","balance_actual_deg","balance_error_deg","rate_target_dps","dual_roll_actual_deg","dual_pitch_actual_deg","dual_roll_rate_dps","dual_pitch_rate_dps","dual_roll_pwm","dual_pitch_pwm"]:
                    if name not in values:continue
                    lo,hi=ranges.get(name,(values[name],values[name]));ranges[name]=[min(lo,values[name]),max(hi,values[name])]
                if last_host_time is not None:max_gap=max(max_gap,host_time-last_host_time)
                last_host_time=host_time
                if count==1 or count%100==0:
                    file.flush(); print("FRAMES",count,"state",values["rate_state"],"rate",values.get("rate_actual_dps",values.get("dual_pitch_rate_dps")),"PWM",values.get("rate_pwm",values.get("dual_pitch_pwm")),flush=True)
    finally:receiver.stop()
    report={"real_board_frames_verified":count>0,"feedback_direction_verified":False,
            "module_ip":peer,"valid_frames":count,"invalid_frames":invalid,
            "unique_rate_sequences":len(unique_seq),"max_host_receive_gap_s":max_gap,
            "state_counts":states,"ranges":ranges,"error":fatal,
            "csv":str(output),"channels":names}
    output.with_suffix(".json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)
    return 0 if count else 2

if __name__=="__main__":raise SystemExit(main())
