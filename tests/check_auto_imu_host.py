"""Autostream IMU frame and unsolicited channel metadata -> actual MCAR UDP parser.
Local loopback only; no claim of real radio traffic.
"""
from pathlib import Path
import json
import socket
import subprocess
import sys
import tempfile
root=Path(__file__).resolve().parents[1]
host=Path.home()/"Desktop/imu/route_run_update"
sys.path.insert(0,str(host))
from udp_receiver import UdpReceiver
from mcar_telemetry import control_reply,parse_names,validate_frame
from packet_parser import FieldRule,parse_packet
receiver=UdpReceiver()
with tempfile.TemporaryDirectory(prefix="qcar-auto-imu-host-") as work:
    fixture=Path(work)/"imu.bin"
    subprocess.run([sys.executable,str(root/"tests/check_wifi.py"),"--export-imu-fixture",str(fixture)],check=True)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as pick:
        pick.bind(("127.0.0.1",0));port=pick.getsockname()[1]
    receiver.start("127.0.0.1",port)
    def receive():
        while True:
            kind,payload=receiver.events.get(timeout=2)
            if kind=="packet":return payload[1]
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            # Same metadata as production wifispi_telemetry_announce (checked in C test).
            sender.sendto(b"MCAR SUB roll_deg,pitch_deg,yaw_deg\n",("127.0.0.1",port))
            reply=control_reply(receive());assert reply[0]=="SUB"
            names=parse_names(reply[1]);assert names==["roll_deg","pitch_deg","yaw_deg"]
            sender.sendto(fixture.read_bytes(),("127.0.0.1",port))
            packet=receive();validate_frame(packet,len(names))
            values,errors=parse_packet(packet,[FieldRule(n,"float32",4*i) for i,n in enumerate(names)])
            assert not errors and values==dict(roll_deg=12.0,pitch_deg=-4.0,yaw_deg=30.0)
            (root/"validation/wifi-auto-imu-host-check.json").write_text(json.dumps({"host_dir":str(host),"frame_bytes":len(packet),"values":values,"hardware_radio_verified":False},indent=2)+"\n",encoding="utf-8")
            print("PASS: auto IMU metadata + real firmware 16-byte packet received/parsed by Route_Run host over localhost UDP")
    finally:receiver.stop()
