"""Actual firmware packing -> real localhost UDP -> existing MCAR host parser.
This is a protocol compatibility check, NOT an over-the-air hardware test.
"""
from pathlib import Path
import argparse
import importlib
import json
import queue
import socket
import subprocess
import sys
import tempfile
import time

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument("--host-dir",type=Path,default=Path.home()/"Desktop/imu/route_run_update")
args=parser.parse_args()
sys.path.insert(0,str(args.host_dir))
from udp_receiver import UdpReceiver
from mcar_telemetry import validate_frame, control_reply, Subscription, parse_names
from packet_parser import FieldRule, parse_packet
profile=json.loads((root/"tools/gyro_telemetry.json").read_text("utf-8"))
names=parse_names(",".join(profile["channels"]))
receiver=UdpReceiver()
with tempfile.TemporaryDirectory(prefix="qcar-host-proof-") as work:
    fixture=Path(work)/"frame.bin"
    subprocess.run([sys.executable,str(root/"tests/check_wifi.py"),"--export-fixture",str(fixture)],check=True)
    packet=fixture.read_bytes()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sink:
        sink.bind(("127.0.0.1",0)); port=sink.getsockname()[1]
    receiver.start("127.0.0.1",port)
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            subscription=Subscription(); command=subscription.begin(",".join(names),profile["period_ms"],time.monotonic())
            while command is not None:
                sender.sendto(("MCAR "+command).encode("ascii"),("127.0.0.1",port))
                while True:
                    kind,payload=receiver.events.get(timeout=2)
                    if kind=="packet":break
                reply=control_reply(payload[1]); assert reply is not None
                command=subscription.acknowledge(*reply,time.monotonic())
            assert not subscription.pending and not subscription.error
            sender.sendto(packet,("127.0.0.1",port))
            while True:
                kind,payload=receiver.events.get(timeout=2)
                if kind=="packet":break
            validate_frame(payload[1],len(names))
            values,errors=parse_packet(payload[1],[FieldRule(name,"float32",4*i) for i,name in enumerate(names)])
            assert not errors and values["rate_actual_dps"]==-12.5
            assert values["rate_error_dps"]==12.5 and values["rate_pwm"]==250
            assert values["motor_left_pwm"]==250 and values["motor_right_pwm"]==-250
            assert values["rate_state"]==2 and values["imu_status"]==1
            assert values["rate_kp"]==20 and values["rate_sign"]==1
            assert values["driver_speed_frames"]==123 and values["rate_seq"]==99
            report={"test":"firmware mock SPI -> real loopback UDP -> existing MCAR receiver/parser",
                    "hardware_radio_verified":False,"host_dir":str(args.host_dir),
                    "channel_count":len(names),"frame_bytes":len(packet),"values":values}
            (root/"validation/wifi-rate-host-check.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
            print("PASS: existing MCAR host received and decoded all 19 firmware channels over localhost UDP")
    finally:receiver.stop()
