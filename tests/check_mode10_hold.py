"""Mode10 now uses the ASC speed cascade; position hold remains Mode2."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).with_name('check_asc_integration.py')),run_name='__main__')
