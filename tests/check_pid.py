"""Generic PID numerical tests; independent of TC264 SDK."""
from pathlib import Path
import subprocess,tempfile,os
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='onewheel-pid-') as d:
    test=Path(d)/'pid_test.c';test.write_bytes((root/'tests/pid_test.c.in').read_bytes())
    exe=Path(d)/'pid_test.exe'
    subprocess.run([os.environ.get('HOST_CC','gcc'),'-I'+str(root/'code'),str(root/'code/pid.c'),str(test),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('generic positional/incremental PID PASS')
