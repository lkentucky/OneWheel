"""Upstream portable core and actual 100us QSPI wait-code regressions."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];pkg=root/'code/r10h';tests=root/'tests/asc_r10h'
core=list((pkg/'core').glob('*.c'))+[pkg/'src/r10h_attitude.c']
with tempfile.TemporaryDirectory(prefix='qcar-asc-r10h-') as d:
 for name in ['api_contract','core_regression','strong_vibration','bounded_spi']:
  exe=Path(d)/(name+'.exe');sources=[tests/(name+'.c')] if name=='bounded_spi' else core+[tests/(name+'.c')]
  cmd=['gcc','-std=c99','-O2','-Wall','-Wextra','-I'+str(tests/'mocks'),'-I'+str(pkg/'include'),'-I'+str(pkg/'core/candidate'),*[str(p) for p in sources],'-lm','-o',str(exe)]
  subprocess.run(cmd,check=True);subprocess.run([str(exe)],check=True,timeout=30)
