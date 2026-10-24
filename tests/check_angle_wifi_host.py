"""Firmware angle packet -> real loopback UDP -> Route_Run host parser (not radio)."""
from pathlib import Path
import json,socket,subprocess,sys,tempfile
root=Path(__file__).resolve().parents[1]
host=Path.home()/"Desktop/imu/route_run_update";sys.path.insert(0,str(host))
from udp_receiver import UdpReceiver
from mcar_telemetry import control_reply,parse_names,validate_frame
from packet_parser import FieldRule,parse_packet
names=json.loads((root/"tools/angle_telemetry.json").read_text("utf-8"))["channels"]
with tempfile.TemporaryDirectory(prefix="qcar-angle-host-") as work:
 f=Path(work)/"angle.bin"
 subprocess.run([sys.executable,str(root/"tests/check_wifi.py"),"--export-angle-fixture",str(f)],check=True)
 with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as pick:
  pick.bind(("127.0.0.1",0));port=pick.getsockname()[1]
 receiver=UdpReceiver();receiver.start("127.0.0.1",port)
 def receive():
  while True:
   kind,data=receiver.events.get(timeout=2)
   if kind=="packet":return data[1]
 try:
  with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
   sender.sendto(("MCAR SUB "+",".join(names)+"\n").encode(),("127.0.0.1",port))
   reply=control_reply(receive());assert parse_names(reply[1])==names
   sender.sendto(f.read_bytes(),("127.0.0.1",port));packet=receive();validate_frame(packet,len(names))
   values,errors=parse_packet(packet,[FieldRule(n,"float32",4*i) for i,n in enumerate(names)])
   assert not errors and len(values)==28
   assert abs(values["balance_actual_deg"]-1.7)<1e-5 and abs(values["balance_target_deg"]+.3)<1e-5
   assert values["balance_error_deg"]==-2 and values["rate_target_dps"]==-40
   assert values["rate_pwm"]==-1000 and values["motor_right_pwm"]==1000
   assert values["balance_rate_saturated"]==1 and values["balance_pwm_saturated"]==0
   report={"test":"firmware packing -> localhost UDP -> Route_Run parser","channel_count":len(names),"frame_bytes":len(packet),"hardware_radio_verified":False,"values":values}
   (root/"validation/wifi-angle-host-check.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
   print("PASS: Route_Run host receives all 28 angle-loop fields, values/signs/saturation flags verified")
 finally:receiver.stop()
