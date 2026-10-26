# A/B angle response changes

- A: shared angle LPF tau is 0.04 s (was 0.06 s). Flywheel rate LPF remains 0.005 s.
- B: `FLY_ANGLE_FF_KP` in `code/config.h` defaults to 300 PWM/degree. Set 0 to disable; supported range 0..1000. This is a compile-time parameter, not stored in Flash or controlled by the UDP PID sliders.
- Active only for the flywheel in Modes 3/5 with nonzero angle Kp. Ground control, rate-only Modes 6/7 and raw Mode8 do not receive the compensation.
- Formula: `Sign * clamp(rate_PID + FLY_ANGLE_FF_KP * (target - angle_lpf), +/- PWM_limit * startup_scale)`. The rate PID retains its own clamp. The added term uses the same filtered angle error and output direction.
- Stop, calibration, IMU stale/failure, invalid inputs and tilt protection still gate the entire output. PID gains, mechanical zeros, signs and Flash format are unchanged.
- Rebuild and flash the TC264 to activate. Flash PID values apply only when `QCAR_BOOT_LOAD_FLASH=1` (currently 0); enabling this term does not validate physical stability.

Validation: 20/20 host regressions pass, including compensation with zero rate Kp, reversed output sign, final/ramp clamps and protection gates. Target HEX was not rebuilt: standalone TASKING reports F104 (IDE-only license), and ADS headless cannot activate CDT UI without a Workbench. Existing HEX files are older and do not contain these changes.
