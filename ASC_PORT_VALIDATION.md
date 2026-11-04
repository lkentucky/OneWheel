# Complete reference-layout port validation

Reference: Kongmengwuwei/ASC_SummerTraining_Unicycle @ 0eed51b8fd0f4e72ab92f737642ab1900f60cbd3.

This supersedes the earlier flat Qcar compatibility adapter. The application now has the reference Common/HW_Drivers/Estimation/Controller/Menu/Protocols tree, original Menu/Display and CPU0 main, 8x16 LCD font, single-ISR 1kHz acquisition/control, reference parameter management and cfg/station protocol.

- Reference station + MCU extraction + Qcar UART contract: **149 passed**, no failures/errors.
- A+ real core API, dynamics/vibration, real IMU/attitude integration and actual bounded SPI code: all five upstream host groups passed, no hardware claims.
- Final local WiFi variant ADS/TASKING compile/link: **PASS, 0 errors, 0 warnings**.
- Final public UART-default variant ADS/TASKING compile/link: **PASS, 0 errors, 0 warnings**.
- Menu/menu.c, Menu/display.c and user/cpu0_main.c are LF-equivalent to the reference commit; the UI is not the old Qcar menu.
- Physical defaults, key pins, ground friction, nonblocking text UART3 and camera boot policy are explicit Qcar adaptations.

The nonblocking UART3 host contract exercises bounded FIFO pumping, complete-line latest-wins replacement and Stop, text overflow/malformed rejection, binary RPM containing LF and watchdog expiration. GUI tests use an offscreen mock device. Firmware tests use host hardware stubs. The old pre-r10h IMU extraction test was replaced with the actual r10h integration suite; historical donor-vehicle defaults remain archived, while Qcar defaults are explicitly configuration-derived.

No flashing, actual LCD photo, vehicle balance test or live station connection was performed. Runtime timing margins, stack high-water mark, physical feedback directions, board firmware/torque response and the user's specific boot error still need on-board evidence. A successful port/build is not proof of unassisted balance or that a prior boot fault disappeared.

WiFi credentials remain only in an ignored private header. WiFi-linked binaries and their detailed local logs remain under .ads; they are not committed. Public clones build UART2 by default. See ASC_PORT.md for setup and interface changes.


## 2026-11-04 boot menu/camera correction

The auto-start flag had been placed on the explicit restart command instead of the boot call. It now guards only vision_core_init; manual restart is unconditional. Camera init failure still returns failure and publishes VISION_CORE_FAILED, while default nonfatal SDK camera logs no longer clear the CPU0-owned LCD from CPU1. MT9V03X_INIT_LOG_ENABLE may be explicitly enabled only for a standalone camera diagnostic with appropriate LCD ownership.

Eight targeted host checks passed (four new boot/manual-retry/logging-policy cases, existing firmware protocol/ISR checks and Qcar UART contract). ADS/TASKING build passed with zero errors/warnings. No real LCD or on-board reboot test was performed; the hardware symptom still needs confirmation after flashing the new image. The earlier 149-case full-suite result remains historical.


## 2026-11-04 linked navigation

Parent/child/circular-sibling nodes with a bounded static pool now own navigation, parameter binding and per-directory cursor/scroll history. Folder callbacks dispatch input/rendering. Existing rendering, geometry, colors, 8x16 font and image renderer are retained; page IDs remain display-style metadata.

155 host checks passed; ADS/TASKING compilation/link passed with zero errors/warnings. New native-C tests traverse every parameter group, wrap siblings, verify parent returns and cursor history, bind parameter descriptors, enter Image/IPM/Path Map, preserve action/stop callbacks and reject pool overflow. Render-function/font/image fingerprints are checked; known independent local IMU diagnostic wording is normalized only in the visual contract. No real LCD or button test was performed.
