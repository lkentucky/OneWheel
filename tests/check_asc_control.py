from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='qcar-asc-control-') as d:
 exe=Path(d)/'test.exe'
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(root/'code'),str(root/'code/asc_control.c'),str(root/'code/asc_pid.c'),str(root/'tests/asc_control_test.c'),'-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True,timeout=10)
