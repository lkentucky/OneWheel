"""Actual dual-axis frame -> loopback UDP -> Route_Run receiver/parser; not radio."""
from pathlib import Path
import json,socket,subprocess,sys,tempfile
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path.home()/"Desktop/imu/route_run_update"))
from udp_receiver import UdpReceiver
from mcar_telemetry import control_reply,parse_names,validate_frame
from packet_parser import FieldRule,parse_packet
names=json.loads((root/"tools/startup_telemetry.json").read_text("utf-8"))["channels"]
with tempfile.TemporaryDirectory(prefix="qcar-dual-host-") as d:
 f=Path(d)/"dual.bin";subprocess.run([sys.executable,str(root/"tests/check_wifi.py"),"--export-startup-fixture",str(f)],check=True)
 with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as pick:
  pick.bind(("127.0.0.1",0));port=pick.getsockname()[1]
 receiver=UdpReceiver();receiver.start("127.0.0.1",port)
 def receive():
  while True:
   kind,data=receiver.events.get(timeout=2)
   if kind=="packet":return data[1]
 try:
  with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
   sender.sendto(("MCAR SUB "+",".join(names)+"\n").encode(),("127.0.0.1",port));assert parse_names(control_reply(receive())[1])==names
   sender.sendto(f.read_bytes(),("127.0.0.1",port));packet=receive();validate_frame(packet,34)
   values,errors=parse_packet(packet,[FieldRule(n,"float32",4*i) for i,n in enumerate(names)])
   assert not errors and len(values)==34 and len(packet)==140
   assert values["fault_reason"]==4 and values["fault_imu_status"]==1 and values["fault_imu_dt_ms"]==15
   assert values["imu_spi_timeouts"]==2 and values["imu_not_ready"]==9 and values["startup_scale"]==.25
   (root/"validation/wifi-startup-host-check.json").write_text(json.dumps(dict(hardware_radio_verified=False,values=values,frame_bytes=len(packet)),indent=2)+"\n",encoding="utf-8")
   print("PASS: Route_Run decodes startup/fault latch and IMU diagnostics (34 fields)")
 finally:receiver.stop()
