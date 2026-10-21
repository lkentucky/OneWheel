"""Use the existing WiFiSPI_Monitor source with OneWheel's native parameters."""
from pathlib import Path
import argparse
import importlib
import sys


def apply_profile(tuning):
    P = tuning.Parameter
    params = [
        P('speed', '行进轮目标 / 计数每20ms', 160, -1000, 1000, '原生编码器单位；0不是停车指令。'),
        P('k1', '视觉权重 k1', 1.25, -10, 10, '仅停车修改。'),
        P('k2', '视觉权重 k2', .5, -10, 10, '仅停车修改。'),
    ]
    groups = [
        ('roll_angle', '平衡角度环', (.85, 0, .08)),
        ('roll_speed', '平衡速度环', (.2, .001, 0)),
        ('roll_rate', '平衡角速度环', (0, .13, 0)),
        ('pitch_angle', '行进角度环', (.58, 0, .003)),
        ('pitch_speed', '行进速度环', (5.8, 0, .001)),
        ('pitch_rate', '行进角速度环', (0, .023, 0)),
        ('motor', '行进电机环', (11.1, 0, .29)),
        ('yaw', '转向误差环', (9.4, 1.55, 0)),
        ('yaw_rate', '转向角速度环', (.62, 0, 0)),
    ]
    for prefix, label, defaults in groups:
        for gain, default in zip(('p', 'i', 'd'), defaults):
            params.append(P(f'{prefix}_{gain}', f'{label} {gain.upper()}', default,
                            -10000, 10000, '独轮原生控制参数；仅停车修改，读取车辆值后再调整。'))
    params.extend([
        P('ramp_ms', '0→160计数起步时间 / ms', 160/.03, 100, 20000,
          '定义为参考目标160的斜坡时间；仅停车修改。'),
        P('deadband_px', '视觉死区 / px', 0, 0, 10, '误差绝对值小于此值时归零。'),
    ])
    # Mutate in place so all imports use the same profile.
    tuning.PARAMETERS[:] = params
    tuning.BY_KEY.clear()
    tuning.BY_KEY.update({p.key: p for p in params})
    tuning.EDITABLE_PARAMETERS[:] = params
    tuning.EDITABLE_BY_KEY.clear()
    tuning.EDITABLE_BY_KEY.update(tuning.BY_KEY)
    tuning.BASE_KEYS.clear()
    # TuningClient accepts a legacy BASE_KEYS subset. OneWheel has no legacy
    # subset; a sentinel cannot be collected because it is absent from BY_KEY.
    # Leaving this set empty would mistakenly accept a bare DONE as complete.
    tuning.BASE_KEYS.add('__onewheel_no_legacy__')
    tuning.SPEED_LOOP_KEYS.clear()
    def validate_scheme(values):
        if set(values) != set(tuning.BY_KEY):
            raise ValueError('方案必须包含独轮的全部32项参数')
        for key, value in values.items():
            tuning.BY_KEY[key].validate(value)
    tuning.validate_curve_scheme = validate_scheme
    return params


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = Path.home() / 'Desktop/麦轮资料/McarPVC/upper_computer'
    parser.add_argument('--upper-computer', type=Path, default=default)
    parser.add_argument('--data-dir', type=Path,
                        default=Path(__file__).resolve().parents[1]/'.monitor_data',
                        help='独轮专用设置、调参方案与采集文件目录')
    args, monitor_args = parser.parse_known_args()
    source = args.upper_computer.resolve()
    if not (source / 'app.py').is_file() or not (source / 'live_tuning.py').is_file():
        parser.error('请用 --upper-computer 指向现有 WiFiSPI_Monitor 源码目录')
    sys.path.insert(0, str(source))
    apply_profile(importlib.import_module('live_tuning'))
    sys.argv = [str(source / 'app.py'), *monitor_args]
    application = importlib.import_module('app')
    # Keep OneWheel settings, tuning schemes and captures separate from Mcar.
    application.STORE = args.data_dir.resolve()
    application.main()


if __name__ == '__main__':
    main()
