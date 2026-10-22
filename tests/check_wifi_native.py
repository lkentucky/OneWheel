"""Compile and execute production C, then decode its bytes with the real monitor.

Usage: python tests/check_wifi_native.py --cc PATH/TO/tcc.exe
       [--upper-computer PATH/TO/upper_computer] [--output OUTPUT_DIR]
Only the compiler and existing monitor source are external dependencies.
"""
import argparse
import importlib
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import zlib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True, type=Path)
    parser.add_argument('--upper-computer', type=Path,
                        default=Path.home()/'Desktop/麦轮资料/McarPVC/upper_computer')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    firmware = repo/'Seekfree_TC264_Opensource_Library'
    sys.path.insert(0, str(args.upper_computer.resolve()))
    sys.path.insert(0, str(repo/'tools'))
    tuning = importlib.import_module('live_tuning')
    profile = importlib.import_module('monitor_onewheel')
    profile.apply_profile(tuning)
    mux = importlib.import_module('multiplex_protocol')
    camera = importlib.import_module('camera_protocol')
    with tempfile.TemporaryDirectory(prefix='onewheel-native-') as temporary:
        directory = Path(temporary)
        for file in (repo/'tests/host').iterdir():
            if file.is_file():
                shutil.copy2(file, directory/file.name)
        for name in ('wifi_link.c', 'wifi_link.h', 'wifi_link_io.c', 'wifi_link_io.h',
                     'balance_control.h', 'camera.h', 'posture_control.h', 'app.h', 'config.h',
                     'app.c', 'balance_control.c', 'camera.c', 'posture_control.c', 'motor.c', 'motor.h',
                     'small_driver_uart_control.c','small_driver_uart_control.h','safety_output.c','safety_output.h'):
            shutil.copy2(firmware/'code'/name, directory/name)
        shutil.copy2(firmware/'libraries/zf_device/zf_device_wifi_spi.h', directory)
        # Extract exact wrapper definitions: SDK register types are mocked, the bodies are not.
        adapters = []
        for file, function in [('zf_driver_uart.c', 'uart_try_write_buffer'),
                               ('zf_driver_pwm.c', 'pwm_output_enable')]:
            source = (firmware/'libraries/zf_driver'/file).read_bytes().decode('gbk')
            start = source.rfind('\n', 0, source.index(function+'(')) + 1
            brace = source.index('{', start)
            depth, end = 1, brace + 1
            while depth:
                depth += (source[end] == '{') - (source[end] == '}')
                end += 1
            adapters.append(source[start:end])
        (directory/'hardware_adapter_functions.h').write_text('\n'.join(adapters), encoding='utf-8')
        # Test the public configuration template; never copy local credentials.
        shutil.copy2(firmware/'code/wifi_link_config.example.h', directory/'wifi_link_config.h')
        output = directory/'out'
        output.mkdir()
        adapters = subprocess.run([str(args.cc.resolve()), '-std=c99', '-I'+str(directory),
                        '-run', str(directory/'hardware_adapter_native.c')], capture_output=True, text=True)
        print(adapters.stdout.strip())
        if adapters.returncode:
            print(adapters.stderr, file=sys.stderr)
            adapters.check_returncode()
        # TinyCC's in-memory runner avoids creating a temporary executable.
        execution = subprocess.run([str(args.cc.resolve()), '-std=c99', '-I'+str(directory),
                        '-run', str(directory/'wifi_native.c'), str(output)],
                        capture_output=True, text=True)
        if execution.returncode:
            print(execution.stdout)
            print(execution.stderr, file=sys.stderr)
            execution.check_returncode()
        for line in execution.stdout.splitlines():
            if line.startswith('PACKET '):
                _, name, hexadecimal = line.split(' ', 2)
                assert Path(name).name == name
                (output/name).write_bytes(bytes.fromhex(hexadecimal))
            else:
                print(line)
        for mode in ('drive_enabled', 'observe_only', 'momentum_only', 'travel_only', 'reversed_travel_direction', 'background_period_overrides'):
            config = directory/'config.h'
            original = config.read_bytes()
            if mode == 'observe_only':
                assert b'#define APP_DRIVE_ENABLE                 (1)' in original
                config.write_bytes(original.replace(b'#define APP_DRIVE_ENABLE                 (1)',
                                                    b'#define APP_DRIVE_ENABLE                 (0)'))
            if mode == 'momentum_only':
                config.write_bytes(original.replace(b'#define CONTROL_PITCH_ENABLE             (1)', b'#define CONTROL_PITCH_ENABLE             (0)')
                                   .replace(b'#define CONTROL_MOTOR_ENABLE             (1)', b'#define CONTROL_MOTOR_ENABLE             (0)'))
            if mode == 'travel_only':
                config.write_bytes(original.replace(b'#define CONTROL_ROLL_ENABLE              (1)', b'#define CONTROL_ROLL_ENABLE              (0)'))
            if mode == 'reversed_travel_direction':
                assert b'#define MOTOR_DIRECTION_SIGN             (1)' in original
                config.write_bytes(original.replace(b'#define MOTOR_DIRECTION_SIGN             (1)',
                                                    b'#define MOTOR_DIRECTION_SIGN             (-1)'))
            extra = []
            if mode == 'background_period_overrides':
                updated = original.replace(b'#define APP_KEY_SCAN_PERIOD_MS           (10u)', b'#define APP_KEY_SCAN_PERIOD_MS           (20u)')
                updated = updated.replace(b'#define APP_TELEMETRY_PERIOD_MS          (10u)', b'#define APP_TELEMETRY_PERIOD_MS          (40u)')
                updated = updated.replace(b'#define APP_STATUS_LOG_PERIOD_MS        (1000u)', b'#define APP_STATUS_LOG_PERIOD_MS        (200u)')
                assert updated != original
                config.write_bytes(updated)
                extra = ['-DHOST_TIMING_VARIANT']
            result = subprocess.run([str(args.cc.resolve()), '-std=c99', '-I'+str(directory), *extra,
                        '-run', str(directory/'upright_native.c')], capture_output=True, text=True)
            config.write_bytes(original)
            print(mode+': '+result.stdout.strip())
            if result.returncode:
                print(result.stderr, file=sys.stderr)
                result.check_returncode()
        channel, data = mux.decode_packet((output/'data.bin').read_bytes())
        assert channel == 'data' and len(data) == 228
        assert struct.unpack_from('<i', data, 36)[0] == -123
        assert struct.unpack_from('<f', data, 76)[0] == -12
        assert struct.unpack_from('<f', data, 92)[0] == -1400
        assert struct.unpack_from('<f', data, 96)[0] == 350
        assert struct.unpack_from('<f', data, 100)[0] == -400
        assert abs(struct.unpack_from('<f', data, 8)[0]-120) < .001
        assert struct.unpack_from('<I', data, 132)[0] == 0
        telemetry = importlib.import_module('onewheel_profile')
        values, errors = telemetry.decode_data(data)
        assert not errors, errors
        assert values['roll_balance_raw'] == 123.5 and values['pitch_balance_raw'] == -321.25
        assert values['roll_zero_raw'] == 30 and values['pitch_zero_raw'] == -100
        assert values['roll_rate_output'] == 350.75
        assert values['pitch_rate_feedback_raw'] == 14300
        assert values['control_flags'] == 234  # ready, drive, roll/pitch/motor; stopped/warmup
        assembler = camera.CameraAssembler()
        frame = None
        for file in sorted(output.glob('camera_*.bin')):
            channel, payload = mux.decode_packet(file.read_bytes())
            assert channel == 'camera'
            packet = SimpleNamespace(data=payload, address=('192.168.1.2', 6666), stamp=0)
            frame = assembler.feed(packet) or frame
        assert frame is not None
        expected = bytes((i*7)%256 for i in range(188*120))
        assert frame.pixels == expected and frame.width == 188 and frame.height == 120
        # The actual C header's CRC must match Python's independent CRC implementation.
        _, payload = mux.decode_packet((output/'camera_00.bin').read_bytes())
        assert struct.unpack_from('<I', payload, 20)[0] == zlib.crc32(expected)
        sent = []
        client = tuning.TuningClient(lambda *args: sent.append(args))
        client.next_id = 42
        client.read_all('192.168.1.2', 6666)
        assert not client.feed_line('[PARAM] 42 DONE', '192.168.1.2')
        assert 42 in client.pending and not client.actual
        for line in (output/'getall.txt').read_text().splitlines():
            client.feed_line(line, '192.168.1.2')
        assert not client.pending, client.message
        assert set(client.actual) == set(tuning.BY_KEY), client.message
        assert len(client.actual) == 35
        scheme = directory/'scheme.json'
        tuning.save_scheme(scheme, client.actual)
        assert tuning.load_scheme(scheme) == {k:v for k,v in client.actual.items() if k in tuning.EDITABLE_BY_KEY}
        summary = dict(status='pass', compiled='production WiFi + app/PID/control/motor/UART/safety-output/vision modules',
                       control_modes=['drive_enabled', 'observe_only', 'momentum_only', 'travel_only', 'reversed_travel_direction', 'background_period_overrides'],
                       regressions=['CPU boot handoff', 'PIT frozen diagnostics and protected start',
                                    'STM lower-word wrap', 'IMU failed reads suppressed and retry recovery',
                                    'independent background period settings', 'STOP with full log queue',
                                    'CPU1 output ownership and stalled CPU0 shutdown',
                                    'completed-control timeout and stalled mailbox producer',
                                    'latched +/- filter overflow and nonfinite fault',
                                    'PWM physical low override and delayed direction enable',
                                    'real UART signed frames, invalid checksum and stale feedback',
                                    'nonblocking UART busy timeout and zero retry'],
                       native_parameters=len(client.actual), image_chunks=17,
                       camera_crc32=f'{zlib.crc32(expected):08x}',
                       hardware='mocked; no board or wireless module tested')
        inputs = [firmware/'code'/name for name in ('wifi_link.c','wifi_link_io.c','config.h',
                  'app.c','balance_control.c','posture_control.c','motor.c','camera.c',
                  'safety_output.c','safety_output.h','small_driver_uart_control.c','small_driver_uart_control.h')]
        inputs += [repo/'tools/monitor_onewheel.py',repo/'tests/host/wifi_native.c',repo/'tests/host/upright_native.c']
        inputs += [firmware/'libraries/zf_driver/zf_driver_uart.c',firmware/'libraries/zf_driver/zf_driver_pwm.c',
                   repo/'tests/host/hardware_adapter_native.c']
        summary['sha256'] = {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in inputs}
        if args.output:
            args.output.mkdir(parents=True, exist_ok=True)
            for file in output.iterdir():
                shutil.copy2(file, args.output/file.name)
            (args.output/'results.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print(json.dumps(summary))


if __name__ == '__main__':
    main()
