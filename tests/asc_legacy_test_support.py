"""Link the new ASC handler into legacy-mode host tests, using hardware stubs.
No control behavior is mocked; new Mode10 is tested by check_asc_integration.
"""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
def install():
 original=subprocess.run
 if getattr(original,'asc_legacy_support',False):return
 def run(argv,*args,**kwargs):
  if isinstance(argv,(list,tuple)) and any(str(v).endswith('balance_control.c') for v in argv) and not any(str(v).endswith('asc_control.c') for v in argv):
   argv=list(argv)+[str(root/'code/asc_control.c'),str(root/'code/asc_pid.c'),str(root/'tests/asc_legacy_sensor_stub.c'),'-include',str(root/'tests/asc_legacy_compat.h')]
   if any(str(v).endswith('small_driver_uart_control.c') for v in argv):argv+=['-DASC_TEST_REAL_DRIVER=1']
  return original(argv,*args,**kwargs)
 run.asc_legacy_support=True;subprocess.run=run
