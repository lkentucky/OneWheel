"""Measure fixed heading in ordinary Balance over station UDP."""

import argparse
import csv
from dataclasses import astuple, dataclass
import math
from pathlib import Path
import socket
import statistics
import time


@dataclass(frozen=True)
class Sample:
    ms: int
    target: float
    yaw: float
    rate: float
    rpm: float
    pwm: float

    @property
    def error(self):
        return self.target - self.yaw


class Station:
    def __init__(self, sock, peer):
        self.sock = sock
        self.peer = peer
        self.seq = time.monotonic_ns() % 60000 + 1
        self.mode = None
        self.status_ms = None
        self.imu_ready = False
        self.samples = []
        self.last_sample_at = None
        self.drive_samples = []

    def receive(self):
        try:
            data, address = self.sock.recvfrom(8192)
        except socket.timeout:
            return []
        if address != self.peer:
            return []
        lines = data.decode("ascii", errors="replace").splitlines()
        for line in lines:
            tag, _, payload = line.partition(":")
            fields = payload.split(",")
            if tag == "stat" and len(fields) == 15:
                try:
                    status_ms, imu_flags, attitude_flags = map(
                        int, (fields[0], fields[5], fields[8])
                    )
                except ValueError:
                    continue
                if self.status_ms is None or status_ms >= self.status_ms:
                    self.status_ms = status_ms
                    self.mode = tuple(fields[1:5])
                    self.imu_ready = imu_flags & 7 == 7 and attitude_flags & 3 == 1
            if tag in ("run", "fly") and len(fields) == (25 if tag == "run" else 4):
                try:
                    ms = int(fields[0])
                    drive_fields = (fields[10:14] + fields[1:4] + [fields[6]]
                                    if tag == "run" else fields[1:])
                    values = tuple(map(float, drive_fields))
                except ValueError:
                    continue
                if all(map(math.isfinite, values)):
                    self.drive_samples.append((ms, tag, values))
            if tag != "yaw" or len(fields) != 6:
                continue
            try:
                sample = Sample(int(fields[0]), *map(float, fields[1:]))
            except ValueError:
                continue
            if all(map(math.isfinite, astuple(sample))) and (
                not self.samples or sample.ms > self.samples[-1].ms
            ):
                self.samples.append(sample)
                self.last_sample_at = time.monotonic()
        return lines

    def request(self, operation, *args, retries=3):
        for _ in range(retries):
            self.seq = self.seq % 65535 + 1
            command = ",".join((operation, str(self.seq), *args))
            self.sock.sendto(f"cfg:{command}\r\n".encode("ascii"), self.peer)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                for line in self.receive():
                    if line.startswith(f"rsp:{self.seq},"):
                        _, status, *fields = line[4:].split(",")
                        if status != "ok":
                            raise RuntimeError(line)
                        return fields
        raise RuntimeError(f"No reply to {operation} {' '.join(args)}")

    def get_hold(self):
        return float(self.request("get", "yaw_hold_deg")[2])

    def set_hold(self, value):
        self.request("set", "yaw_hold_deg", str(value))
        if abs(self.get_hold() - value) > 0.001:
            raise RuntimeError("Yaw Hold readback did not match")

    def set_gain(self, key, value):
        self.request("set", key, str(value))
        if abs(float(self.request("get", key)[2]) - value) > 0.001:
            raise RuntimeError(f"{key} readback did not match")

    def set_rate_kp(self, value):
        self.set_gain("y_rate_kp", value)

    def capture(self, seconds, active_step=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.receive()
            if self.mode is not None and self.mode != ("2", "0", "0", "0"):
                raise RuntimeError(f"Balance ended: {self.mode}")
            if active_step:
                self.check_step_feedback()

    def check_step_feedback(self):
        if self.mode is not None and not self.imu_ready:
            raise RuntimeError("IMU link and converged attitude are required for a step")
        if (self.mode != ("2", "0", "0", "0") or self.last_sample_at is None
                or time.monotonic() - self.last_sample_at > 1.0):
            raise RuntimeError("Fresh Balance yaw feedback is required for a step")
        sample = self.samples[-1]
        if abs(sample.error) > 6 or abs(sample.rpm) > 3000:
            raise RuntimeError(
                f"Step interrupted at {sample.error:.1f}° error, {sample.rpm:.0f} common RPM"
            )

    def check_step_baseline(self):
        self.check_step_feedback()
        sample = self.samples[-1]
        recent_yaw = [row for row in self.samples if sample.ms - row.ms <= 500]
        recent_roll = [abs(values[7]) for ms, tag, values in self.drive_samples
                       if tag == "run" and 0 <= sample.ms - ms <= 500]
        if (len(recent_yaw) < 4 or len(recent_roll) < 8
                or any(abs(row.target - sample.target) > 0.05 for row in recent_yaw)
                or abs(sample.error) > 3 or abs(sample.rpm) > 1750
                or max(recent_roll) >= 6500):
            raise RuntimeError("Insufficient stable yaw and Roll headroom for another step")
        roll_pwm = statistics.median(recent_roll)
        print(f"Baseline error={sample.error:.2f}°, common={sample.rpm:.0f} RPM, "
              f"Roll={roll_pwm:.0f} PWM", flush=True)
        return sample

    def write_drive_capture(self, path):
        """Join yaw, roll and flywheel data by firmware snapshot timestamp."""
        rows = {}
        for ms, tag, values in self.drive_samples:
            rows.setdefault(ms, {})[tag] = values
        with path.open("x", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("ms", "gyro_dps", "yaw_pwm_raw", "yaw_pwm_applied",
                             "common_rpm", "fly_a_rpm", "fly_b_rpm", "fly_link_ok",
                             "roll_deg", "roll_target_deg", "roll_rate_dps", "roll_pwm"))
            for ms, row in sorted(rows.items()):
                run = row.get("run", ("",) * 8)
                writer.writerow((ms, *run[:4], *row.get("fly", ("",) * 3), *run[4:]))


def measure_step(samples, start_ms, start_target, offset):
    """Use the observed target ramp to identify the trial in board time."""
    final_target = start_target + offset
    direction = math.copysign(1, offset)
    rows = [s for s in samples if s.ms > start_ms]
    onset = next((i for i, s in enumerate(rows)
                  if direction * (s.target - start_target) > 0.05), None)
    if onset is None:
        raise RuntimeError("No observed target transition")
    rows = rows[onset:]
    gaps = [b.ms - a.ms for a, b in zip(rows, rows[1:])]
    if len(rows) < 10 or max(gaps, default=0) > 250:
        raise RuntimeError("Insufficient or discontinuous yaw samples")
    tail = [r for r in rows if rows[-1].ms - r.ms <= 1000]
    if any(abs(r.target - final_target) > 0.05 for r in tail):
        raise RuntimeError("Target did not reach the requested angle")
    # Require one continuous second inside the error band and no later exit.
    settled = None
    for i, sample in enumerate(rows):
        if rows[-1].ms - sample.ms < 1000:
            break
        if all(abs(r.target - final_target) <= 0.05 and
               abs(r.yaw - final_target) <= 0.5 for r in rows[i:]):
            settled = sample.ms - rows[0].ms
            break
    return {
        "target_deg": round(final_target, 3),
        "last_second_error_deg": round(statistics.median(r.error for r in tail), 3),
        "overshoot_deg": round(max(0.0, *(direction * (r.yaw - final_target) for r in rows)), 3),
        "settled_ms_from_first_observed_target_change": settled,
        "peak_rate_dps": max(abs(r.rate) for r in rows),
        "peak_common_rpm": max(abs(r.rpm) for r in rows),
        "peak_yaw_pwm": max(abs(r.pwm) for r in rows),
        "max_sample_gap_ms": max(gaps),
    }


def measure_disturbance(samples):
    """Time recovery from peak observed error for one disturbance."""
    gaps = [b.ms - a.ms for a, b in zip(samples, samples[1:])]
    if len(samples) < 10 or max(gaps, default=0) > 250:
        raise RuntimeError("Insufficient or discontinuous yaw samples")
    target = samples[0].target
    if any(abs(s.target - target) > 0.05 for s in samples):
        raise RuntimeError("Target changed during disturbance capture")
    peak = max(range(len(samples)), key=lambda i: abs(samples[i].error))
    if abs(samples[peak].error) < 1:
        raise RuntimeError("No disturbance of at least 1 degree observed")
    recovered = None
    for i in range(peak + 1, len(samples)):
        if samples[-1].ms - samples[i].ms < 1000:
            break
        if all(abs(s.error) <= 0.5 for s in samples[i:]):
            recovered = samples[i].ms - samples[peak].ms
            break
    return {
        "target_deg": target,
        "peak_signed_error_deg": round(samples[peak].error, 3),
        "recovery_ms_from_peak": recovered,
        "last_error_deg": round(samples[-1].error, 3),
        "peak_common_rpm": max(abs(s.rpm) for s in samples),
        "peak_yaw_pwm": max(abs(s.pwm) for s in samples),
        "max_sample_gap_ms": max(gaps),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Vehicle address")
    parser.add_argument("--csv", type=Path, required=True, help="New capture file")
    parser.add_argument("--drive-csv", type=Path,
                        help="Optional synchronized Roll, mixed PWM and individual flywheel RPM")
    parser.add_argument("--step", type=float, default=0,
                        help="Positive step magnitude in degrees, 0 for passive capture")
    parser.add_argument("--seconds", type=float, default=5,
                        help="Duration per plateau or passive capture")
    gain = parser.add_mutually_exclusive_group()
    gain.add_argument("--rate-kp", type=float,
                      help="Temporarily use this rate gain during a step test")
    gain.add_argument("--angle-kd", type=float, nargs="+", metavar="GAIN",
                      help="Temporarily run a step suite at each listed angle derivative gain")
    args = parser.parse_args()
    if not 0 <= args.step <= 10 or not 2 <= args.seconds <= 60:
        parser.error("Use 0–10 degrees and 2–60 seconds")
    if args.rate_kp is not None and (not args.step or not 0 < args.rate_kp <= 200):
        parser.error("--rate-kp requires a step test and a positive gain up to 200")
    if args.angle_kd is not None and (not args.step or not all(0 <= kd <= 50 for kd in args.angle_kd)):
        parser.error("--angle-kd requires a step test and gains from 0 through 50")
    gain_key = "y_angle_kd" if args.angle_kd is not None else "y_rate_kp"
    gains = args.angle_kd if args.angle_kd is not None else (args.rate_kp,)

    with args.csv.open("x", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(Sample.__annotations__)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("", 8081))
            sock.settimeout(0.08)
            station = Station(sock, (socket.gethostbyname(args.host), 5001))
            original = None
            restore = False
            original_gain = None
            restore_gain = False
            try:
                station.request("hello")
                station.capture(1.0)
                if station.mode != ("2", "0", "0", "0") or len(station.samples) < 5:
                    raise RuntimeError("Ordinary Balance with yaw telemetry is required")
                original = station.get_hold()
                for key in ("y_angle_kp", "y_angle_ki", "y_angle_kd",
                            "y_rate_kp", "y_rate_ki", "y_rate_kd"):
                    reply = station.request("get", key)
                    print(*reply, flush=True)
                    if key == gain_key:
                        original_gain = float(reply[2])
                if not args.step:
                    start = len(station.samples)
                    print("Capturing one disturbance", flush=True)
                    station.capture(args.seconds)
                    print(measure_disturbance(station.samples[start:]), flush=True)
                    return
                if abs(original) + args.step > 180:
                    raise RuntimeError("Step would exceed the Yaw Hold range")
                active_gain = original_gain
                for gain_value in gains:
                    if gain_value is not None and gain_value != active_gain:
                        station.check_step_baseline()
                        restore_gain = True
                        station.set_gain(gain_key, gain_value)
                        active_gain = gain_value
                        restore_gain = active_gain != original_gain
                        station.capture(1, active_step=True)
                    print(f"Step suite with {gain_key} = {active_gain:g}", flush=True)
                    current_offset = original
                    for offset in (original + args.step, original,
                                   original - args.step, original):
                        # Returning to the starting target should not depend on headroom.
                        if offset == original:
                            station.check_step_feedback()
                            before = station.samples[-1]
                        else:
                            before = station.check_step_baseline()
                        restore = True
                        station.set_hold(offset)
                        restore = offset != original
                        print(f"Target offset {offset:g}° at {before.ms} ms", flush=True)
                        station.capture(args.seconds, active_step=True)
                        try:
                            score = measure_step(station.samples, before.ms, before.target,
                                                 offset - current_offset)
                        except RuntimeError as exc:
                            print(f"Unscored {offset:g}°: {exc}", flush=True)
                        else:
                            print(score, flush=True)
                        current_offset = offset
            finally:
                try:
                    if restore:
                        try:
                            station.set_hold(original)
                        except (OSError, RuntimeError) as exc:
                            raise RuntimeError(
                                f"Yaw Hold restore to {original:g}° unconfirmed. "
                                "Stop the car and check Yaw Hold before the next test."
                            ) from exc
                        print(f"Verified Yaw Hold restored to {original:g}°", flush=True)
                finally:
                    try:
                        if restore_gain:
                            station.set_gain(gain_key, original_gain)
                            print(f"Verified {gain_key} restored to {original_gain:g}", flush=True)
                    finally:
                        writer.writerows(astuple(row) for row in station.samples)
                        if args.drive_csv is not None:
                            station.write_drive_capture(args.drive_csv)


if __name__ == "__main__":
    main()
