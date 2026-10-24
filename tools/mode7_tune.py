"""Mode 7 单步采数/调参：订阅诊断通道，可选下发滑杆，采完给判读。

只做两件事：发 SUB/RATE 和老固件已有的 [slider,name,value]，以及收遥测。
不会 Start/Stop 车辆，不会写 Flash。
"""
from pathlib import Path
import argparse
import csv
import json
import queue
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
TAIL = b"\x00\x00\x80\x7f"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-dir", type=Path, default=Path.home() / "Desktop/imu/route_run_update")
    parser.add_argument("--profile", type=Path, default=ROOT / "tools/diagnose_telemetry.json")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--seconds", type=float, default=6.0, help="订阅确认后采集时长")
    parser.add_argument("--wait", type=float, default=30.0, help="等待板子第一包的最长时间")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="订阅后下发的滑杆")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    sys.path.insert(0, str(args.host_dir))
    from udp_receiver import UdpReceiver
    from mcar_telemetry import Subscription, control_reply, validate_frame
    from packet_parser import FieldRule, parse_packet

    profile = json.loads(args.profile.read_text("utf-8"))
    names = profile["channels"]
    rules = [FieldRule(n, "float32", 4 * i) for i, n in enumerate(names)]
    output = args.output or ROOT / "validation" / ("mode7-" + time.strftime("%Y%m%d-%H%M%S") + ".csv")
    output.parent.mkdir(parents=True, exist_ok=True)

    receiver = UdpReceiver()
    try:
        receiver.start("0.0.0.0", args.port)
    except OSError as exc:
        print(f"端口 {args.port} 被占用（上位机还在监听？）：{exc}", flush=True)
        return 3

    sub = Subscription()
    peer = None
    started = time.monotonic()
    subscribed_at = None
    rows, pending_sets, bad = [], [s.split("=", 1) for s in args.set], 0
    last_telem = None
    sliders_sent, slider_replies = [], []

    def send(text):
        if text:
            receiver.send_bound(text.encode("ascii"), peer, profile["module_port"])

    print(f"监听 0.0.0.0:{args.port}，等待板子数据…", flush=True)
    try:
        with output.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=["host_time_s"] + names)
            writer.writeheader()
            while True:
                now = time.monotonic()
                if subscribed_at is None and now - started > args.wait:
                    print("超时：没收到可确认的订阅应答。", flush=True)
                    break
                if subscribed_at is not None and now - subscribed_at >= args.seconds:
                    break
                send(sub.poll(now))
                try:
                    kind, payload = receiver.events.get(timeout=0.2)
                except queue.Empty:
                    continue
                if kind != "packet":
                    if kind == "error":
                        print("接收错误：", payload, flush=True)
                        break
                    continue
                host_time, packet, address = payload
                if not packet.startswith(b"MCAR ") and not packet.endswith(TAIL):
                    continue
                if peer is None and packet.endswith(TAIL):
                    peer = address[0]
                    print(f"板子 {address[0]}:{address[1]}", flush=True)
                    send(sub.begin(",".join(names), profile["period_ms"], now))
                    continue
                if peer is not None and address[0] != peer:
                    continue
                try:
                    reply = control_reply(packet)
                except ValueError:
                    continue
                if reply:
                    kinds, value = reply
                    print(f"设备：{kinds} {value}", flush=True)
                    if kinds == "SLIDER":
                        slider_replies.append(value)
                        continue
                    if kinds == "ERR":
                        sub.abort(value)
                        break
                    send(sub.acknowledge(kinds, value, now))
                    if not sub.pending and subscribed_at is None:
                        subscribed_at = time.monotonic()
                        for name, text in pending_sets:
                            send(f"[slider,{name},{text}]\n")
                            sliders_sent.append((name, text))
                            print(f"下发 {name}={text}", flush=True)
                        print(f"订阅确认，采集 {args.seconds}s", flush=True)
                    continue
                if sub.pending or subscribed_at is None:
                    continue
                if len(packet) != 4 * len(names) + 4:
                    # Start 会把通道表重置成 startup 档，长度对不上就重新订阅。
                    if not sub.pending:
                        print("通道表被设备重置，重新订阅", flush=True)
                        send(sub.begin(",".join(names), profile["period_ms"], now))
                    continue
                try:
                    validate_frame(packet, len(names))
                    values, errors = parse_packet(packet, rules)
                except ValueError:
                    bad += 1
                    continue
                if errors:
                    bad += 1
                    continue
                # 设备可能切到另一套同样 40 路的档位（Start 会切），长度相同
                # 但顺序不同；用 rate_state 取值范围和 time 单调性识别后重订阅。
                if values["rate_state"] not in (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0) or (
                        last_telem is not None and values["telemetry_ms"] < last_telem):
                    if not sub.pending:
                        print("通道表与解析不一致，重新订阅", flush=True)
                        send(sub.begin(",".join(names), profile["period_ms"], now))
                    last_telem = None
                    continue
                last_telem = values["telemetry_ms"]
                writer.writerow(dict(host_time_s=host_time, **values))
                rows.append(values)
    finally:
        receiver.stop()

    print(f"\nCSV：{output}", flush=True)
    if not rows:
        print("没有有效帧。", flush=True)
        return 2
    report(rows, bad, sliders_sent, slider_replies)
    return 0


def report(rows, bad, sliders_sent, slider_replies):
    def col(name):
        return [r[name] for r in rows if name in r]

    t = col("telemetry_ms")
    span = (t[-1] - t[0]) / 1000.0 if len(t) > 1 else 0.0
    period = statistics.median([b - a for a, b in zip(t, t[1:])]) if len(t) > 1 else 0.0
    print(f"帧数 {len(rows)}  丢弃 {bad}  时长 {span:.2f}s  周期中位 {period:.1f}ms")
    print(f"下发滑杆 {sliders_sent or '无'}")
    print(f"设备回执 {slider_replies or '无'}")
    if sliders_sent and not slider_replies:
        print("!! 滑杆没有回执：固件可能还是旧版（未知参数）或命令未送达")

    state = col("rate_state")
    fault = col("fault_reason")
    if state:
        print(f"rate_state {sorted(set(state))}   fault_reason {sorted(set(fault))}")
        if state[-1] < 0:
            print(f"!! 当前已锁存故障 F={int(fault[-1])}，需要 Stop/Start 才能继续")

    seq = col("imu_seq")
    if len(seq) > 1:
        gap = max(b - a for a, b in zip(t, t[1:]))
        i = j = 0
        flat = 0.0
        while i < len(seq):
            j = i
            while j + 1 < len(seq) and seq[j + 1] == seq[i]:
                j += 1
            flat = max(flat, t[j] - t[i])
            i = j + 1
        print(f"imu_seq 最长不变 {flat:.0f}ms (>=25ms 会触发 F=4)   遥测最大帧间隔 {gap:.0f}ms")
    nr, spi = col("imu_not_ready"), col("imu_spi_timeouts")
    if nr:
        bad_bursts = col("imu_bad_bursts")
        bursts = f"   bad_bursts +{int(bad_bursts[-1] - bad_bursts[0])}" if bad_bursts else ""
        print(f"imu_not_ready +{int(nr[-1] - nr[0])}   imu_spi_timeouts +{int(spi[-1] - spi[0])}   "
              f"Reg={int(col('imu_status_reg')[-1]):#04x}{bursts}" if col("imu_status_reg") else
              f"imu_not_ready +{int(nr[-1] - nr[0])}{bursts}")
    dt = col("imu_dt_ms")
    print(f"imu_dt {min(dt):.2f}~{max(dt):.2f}ms   imu_status {sorted(set(int(x) for x in col('imu_status')))}")

    roll, pitch = col("roll_deg"), col("pitch_deg")
    print(f"roll {min(roll):6.1f}~{max(roll):6.1f}   pitch {min(pitch):6.1f}~{max(pitch):6.1f}")
    norm = col("accel_norm_g")
    rejected = sum(1 for v in col("imu_accel_used") if v < 1)
    print(f"accel_norm {min(norm):.2f}~{max(norm):.2f}g   加速度被拒 {rejected}/{len(norm)} 帧")

    rate, pwm = col("rate_actual_dps"), col("rate_pwm")
    limit = col("rate_pwm_limit")[-1]
    mean_r, mean_p = statistics.fmean(rate), statistics.fmean(pwm)
    cov = statistics.fmean((a - mean_r) * (b - mean_p) for a, b in zip(rate, pwm))
    var_r = statistics.fmean((a - mean_r) ** 2 for a in rate) or 1e-9
    var_p = statistics.fmean((b - mean_p) ** 2 for b in pwm) or 1e-9
    corr = cov / (var_r * var_p) ** 0.5
    sat = sum(1 for b in pwm if abs(b) >= 0.98 * limit) / len(pwm) if limit else 0.0
    flips = sum(1 for a, b in zip(pwm, pwm[1:]) if (a > 0) != (b > 0))
    print(f"rate {min(rate):7.1f}~{max(rate):7.1f} dps   pwm {min(pwm):6.0f}~{max(pwm):6.0f} "
          f"(limit {limit:.0f})")
    print(f"相关性 corr(rate,pwm)={corr:+.2f}   pwm 饱和 {sat*100:.0f}%   pwm 过零 {flips} 次/{span:.1f}s")
    print(f"rate_kp={col('rate_kp')[-1]:.0f} rate_sign={col('rate_sign')[-1]:+.0f} "
          f"startup_scale={col('startup_scale')[-1]:.2f}")

    print("\n判读：")
    if corr > 0.3:
        print("  corr>0：PWM 与角速度同相 → 正反馈，Sign 反了（先把 rate_sign 拨到另一档）")
    elif corr < -0.3:
        print("  corr<0：PWM 与角速度反相 → 符号正确，是阻尼")
    else:
        print("  corr≈0：没有明显闭环作用（可能已锁存故障或输出被渐入/限幅压住）")
    if sat > 0.5:
        print(f"  pwm 饱和 {sat*100:.0f}%：限幅太小或增益太大，先降 rate_kp")
    if flips > 0 and span > 0 and flips / span > 8:
        print(f"  pwm 过零 {flips/span:.0f} 次/s：极限环抖动，降 rate_kp")
    if rejected > len(norm) * 0.5:
        print("  过半帧加速度被拒：车在剧烈振动，姿态只剩陀螺积分，F=5 可能是估计发散")


if __name__ == "__main__":
    raise SystemExit(main())
