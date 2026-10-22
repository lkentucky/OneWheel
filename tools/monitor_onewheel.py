"""Launch the monitor in OneWheel mode with live firmware config.h defaults."""
from pathlib import Path
import argparse
import importlib
import sys

FIRMWARE_CONFIG = Path(__file__).resolve().parents[1]/'Seekfree_TC264_Opensource_Library/code/config.h'

def apply_profile(tuning):
    from onewheel_profile import configure_tuning
    return configure_tuning(tuning,FIRMWARE_CONFIG)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upper-computer',type=Path,default=Path.home()/'Desktop/麦轮资料/McarPVC/upper_computer')
    parser.add_argument('--data-dir',type=Path,default=Path(__file__).resolve().parents[1]/'.monitor_data')
    args,remaining = parser.parse_known_args()
    if not (args.upper_computer/'onewheel_profile.py').is_file():
        parser.error('请指定已适配独轮模式的 upper_computer 源码目录')
    sys.path.insert(0,str(args.upper_computer.resolve()))
    sys.argv = ['app.py','--profile','onewheel','--firmware-config',str(FIRMWARE_CONFIG),
                '--data-dir',str(args.data_dir.resolve()),*remaining]
    importlib.import_module('app').main()

if __name__ == '__main__':
    main()
