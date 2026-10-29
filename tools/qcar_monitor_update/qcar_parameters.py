"""Editable RAM parameters for the current upright-only TC264 firmware."""

import math

DEBUG_PARAMETERS = ('debug_fly_left_pwm', 'debug_fly_right_pwm', 'debug_ground_pwm')
SIGN_PARAMETERS = frozenset(('rate_sign', 'fly_sign'))
LEGACY_PARAMETERS = frozenset(('rate_kp', 'rate_pwm_limit', 'rate_sign', 'balance_angle_x', 'balance_angle_y'))
MODE2_PARAMETERS = frozenset(('wheel_speed_kp', 'wheel_speed_ki', 'wheel_tilt_limit', 'wheel_i_limit'))
POSITION_PARAMETERS = frozenset(('wheel_pos_kp', 'wheel_speed_limit'))

# label, editor default, help (defaults are never treated as device readback).
PARAMETERS = {
    'debug_fly_left_pwm': ('Mode8 左飞轮 PWM', 0.0, '±4000；只发左通道；先在板端 Mode8/Start'),
    'debug_fly_right_pwm': ('Mode8 右飞轮 PWM', 0.0, '±4000；只发右通道，不自动取反'),
    'debug_ground_pwm': ('Mode8 行进轮 PWM', 0.0, '±4000；Start/Stop/切模式清零；不保存 Flash'),
    'rate_kp': ('地轮角速度 KP', 30.0, '0～500；仅地轮，Mode4/5/7'),
    'rate_pwm_limit': ('地轮 PWM 限幅', 4000.0, '0～8000；仅地轮闭环'),
    'rate_sign': ('地轮闭环方向', 1.0, '只能 +1 或 -1；不影响 Mode8'),
    'ground_angle_kp': ('地轮角度 KP', 40.0, '0～100；Mode4/5外环'),
    'ground_rate_limit_dps': ('地轮目标角速度限幅', 100.0, '0.1～200 °/s；Mode4/5'),
    'fly_rate_kp': ('飞轮角速度 KP', 36.0, '0～500；Mode3/5/6'),
    'fly_pwm_limit': ('飞轮 PWM 限幅', 8000.0, '0～8000；仅飞轮闭环'),
    'fly_sign': ('飞轮闭环方向', 1.0, '只能 +1 或 -1；不影响 Mode8'),
    'fly_angle_kp': ('飞轮角度 KP', 40.0, '0～100；Mode3/5外环'),
    'fly_rate_limit_dps': ('飞轮目标角速度限幅', 160.0, '0.1～200 °/s；Mode3/5'),
    'balance_angle_x': ('地轮机械零点 / 度', 4.2, '±20°；屏幕Pitch，内部imu_roll_deg'),
    'balance_angle_y': ('飞轮机械零点 / 度', 1.0, '±20°；屏幕Roll，内部imu_pitch_deg'),
    'wheel_speed_kp': ('Mode2 速度环 KP', 0.001, '0～0.1；仅Mode2速度环；RAM可写，暂不保存板载Flash'),
    'wheel_speed_ki': ('Mode2 速度环 KI', 0.0, '0～0.1；仅Mode2速度环；RAM可写，暂不保存板载Flash'),
    'wheel_tilt_limit': ('Mode2 倾角修正限幅 / 度', 1.0, '0～5°；仅Mode2速度环；RAM可写，暂不保存板载Flash'),
    'wheel_i_limit': ('Mode2 PI 积分限幅', 0.5, '0～5；仅Mode2速度环；RAM可写，暂不保存板载Flash'),
    'wheel_pos_kp': ('Mode2 位置环 KP', 0.5, '0～5（1/s）；仅Mode2位置保持，100ms位置环；RAM可写，不保存板载Flash'),
    'wheel_speed_limit': ('Mode2 位置环目标速度限幅', 300.0, '0～2000（计数/秒）；仅Mode2位置保持；RAM可写，不保存板载Flash'),
}

# Version sets stay exact: new position items must never leak into V3/V2/V1.
V4_PARAMETERS = frozenset(PARAMETERS)
V3_PARAMETERS = V4_PARAMETERS - POSITION_PARAMETERS
# V2 schemes (QCAR_JUSTFLOAT_2) contain the original 15 items.
V2_PARAMETERS = V3_PARAMETERS - MODE2_PARAMETERS

SLIDER_LIMITS = {
    **{name: (-4000.0, 4000.0) for name in DEBUG_PARAMETERS},
    'rate_kp': (0.0, 500.0), 'rate_pwm_limit': (0.0, 8000.0),
    'ground_angle_kp': (0.0, 100.0), 'ground_rate_limit_dps': (0.1, 200.0),
    'fly_rate_kp': (0.0, 500.0), 'fly_pwm_limit': (0.0, 8000.0),
    'fly_angle_kp': (0.0, 100.0), 'fly_rate_limit_dps': (0.1, 200.0),
    'balance_angle_x': (-20.0, 20.0), 'balance_angle_y': (-20.0, 20.0),
    'wheel_speed_kp': (0.0, 0.1), 'wheel_speed_ki': (0.0, 0.1),
    'wheel_tilt_limit': (0.0, 5.0), 'wheel_i_limit': (0.0, 5.0),
    'wheel_pos_kp': (0.0, 5.0), 'wheel_speed_limit': (0.0, 2000.0),
}


def validate_parameter(key, value):
    """Return a finite, in-range float for an editable parameter; reject anything else."""
    if key not in PARAMETERS:
        raise ValueError(f'未知参数 {key}')
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{key} 必须为有限数值')
    low, high = (-1.0, 1.0) if key in SIGN_PARAMETERS else SLIDER_LIMITS[key]
    if not low <= value <= high:
        raise ValueError(f'{key}={value:g} 超出 {low:g}～{high:g}')
    if key in SIGN_PARAMETERS and value not in (-1, 1):
        raise ValueError(f'{key} 只能为 +1 或 -1')
    return float(value)


PWM_PROFILE = [
    'telemetry_ms', 'rate_seq', 'rate_state', 'pwm_debug_enabled',
    *DEBUG_PARAMETERS, 'motor_left_pwm', 'motor_right_pwm', 'motor_ground_pwm',
    'flywheel_left_speed_raw', 'flywheel_right_speed_raw', 'driver_speed_frames',
    'fault_reason', 'startup_scale',
]
