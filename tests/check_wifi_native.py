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
            shutil.copy2(file, directory/file.name)
        for name in ('wifi_link.c', 'wifi_link.h', 'wifi_link_io.c', 'wifi_link_io.h',
                     'balance_control.h', 'camera.h', 'posture_control.h'):
            shutil.copy2(firmware/'code'/name, directory/name)
        shutil.copy2(firmware/'libraries/zf_device/zf_device_wifi_spi.h', directory)
        # Test the public configuration template; never copy local credentials.
        shutil.copy2(firmware/'code/wifi_link_config.example.h', directory/'wifi_link_config.h')
        output = directory/'out'
        output.mkdir()
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
        channel, data = mux.decode_packet((output/'data.bin').read_bytes())
        assert channel == 'data' and len(data) == 148
        assert struct.unpack_from('<i', data, 36)[0] == -123
        assert struct.unpack_from('<f', data, 76)[0] == -12
        assert struct.unpack_from('<f', data, 92)[0] == -1400
        assert abs(struct.unpack_from('<f', data, 8)[0]-120) < .001
        assert struct.unpack_from('<I', data, 132)[0] == 2
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
        assert len(client.actual) == 32
        scheme = directory/'scheme.json'
        tuning.save_scheme(scheme, client.actual)
        assert tuning.load_scheme(scheme) == client.actual
        summary = dict(status='pass', compiled='production wifi_link.c + wifi_link_io.c',
                       native_parameters=len(client.actual), image_chunks=17,
                       camera_crc32=f'{zlib.crc32(expected):08x}',
                       hardware='mocked; no board or wireless module tested')
        inputs = [firmware/'code/wifi_link.c', firmware/'code/wifi_link_io.c',
                  repo/'tools/monitor_onewheel.py', repo/'tests/host/wifi_native.c']
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
