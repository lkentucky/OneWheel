# ASC port validation

Reference commit: 0eed51b8fd0f4e72ab92f737642ab1900f60cbd3.

- Host suite: 32/32 passed.
- ADS/TASKING firmware compile/link: PASS, zero errors.
- Existing warnings: two menu.c volatile-pointer conversions in legacy flywheel test entries.
- Firmware source fingerprint: `b779f7323c491758d688bffeed089f62169f1a60fec3b4313741f0f2aea16b56`.

Coverage: real ASC controller/PID, reference-frame zeros and Pitch-rate projection, yaw headroom, sensor/driver/encoder/fall fault gates, actual r10h core and 100us SPI wait code, 6000+3000 startup, raw/control bias separation, Flash torn writes/CRC/schema, WiFi and all legacy-mode regressions.

Firmware artifacts are available locally under .ads/firmware-asc; binaries are not committed.

- OneWheelUpright.elf: 9918152 bytes; SHA-256 `9c3b998a328ed80a420512fc40d3bfc59dd8c0055f8295883c250a029e0ecc7c`.
- OneWheelUpright.hex: 323311 bytes; SHA-256 `dd36a5a851a22f2d565ffcc6252213af204df6adb90da817441ec2503e62217d`.
- OneWheelUpright.map: 2730491 bytes; SHA-256 `c6ef04d24db25c95657b808452c4e38795225407fdbfc5ea9f628136d01fd12b`.

No flashing or real-vehicle operation was performed for this port. Host tests and a successful target link do not prove unassisted balance, real-time ISR margins, stack high-water marks, physical motor direction/torque, ground position hold or track performance. Existing driver-board firmware was retained. Legacy EKF mathematics remain regression-tested as a library; the active estimator is r10h.

## Host checks

- check_asc_control.py: PASS
- check_asc_integration.py: PASS
- check_r10h_core.py: PASS
- check_wheel_encoder.py: PASS
- check_wheel_hold.py: PASS
- check_wheel_position.py: PASS
- check_mode10_hold.py: PASS
- check_driver_trace.py: PASS
- check_trace_bridge.py: PASS
- check_trace_client.py: PASS
- check_mode8_step.py: PASS
- check_flywheel_direction.py: PASS
- check_driver_link.py: PASS
- check_driver_protection.py: PASS
- check_pid.py: PASS
- check_balance_control.py: PASS
- check_pwm_debug_menu.py: PASS
- check_pwm_uart.py: PASS
- check_storage.py: PASS
- check_imu.py: PASS
- check_rate_filter.py: PASS
- check_angle_feedback.py: PASS
- check_angle_compensation.py: PASS
- check_calibration.py: PASS
- check_wifi.py: PASS
- check_key_debounce.py: PASS
- check_spi_timeout.py: PASS
- check_wifi_spi_driver.py: PASS
- check_menu_tree.py: PASS
- check_menu_keys.py: PASS
- check_menu_wifi_responsiveness.py: PASS
- check_motor_open_loop.py: PASS
