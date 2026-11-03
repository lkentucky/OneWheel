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
