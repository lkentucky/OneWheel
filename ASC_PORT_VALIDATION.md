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


## 2026-11-04 include IMU first-fault diagnostics

Merged the previously local IMU diagnostic API, acquisition/100us SPI hooks, menu messages and matching test harness. Regression assertions verify each sample failure reason, not-ready/chip-ID/nonfinite diagnostics, first-reason retention and reset on reinitialization. All five real r10h host test groups passed. Menu rendering/navigation and direction/Flash regressions also passed. Production source fingerprint exactly matches the preceding successful ADS build (zero errors/warnings); no repeat build was necessary because only test assertions/documentation changed after that build. Local network credentials and ADS preferences were excluded from this commit. No sensor-fault threshold, calibration matrix or motor control gain was changed by this merge. No hardware test was performed.


## 2026-11-04 adjustable menu editing steps

Each linked node now remembers a session-local step multiplier. Long Enter cycles scale; short Enter still completes editing. Integer scales below one count are skipped and direction/switch steps stay fixed. Parameter-value precision and the Path speed display follow finer steps; the footer shows effective step. No Flash schema or control gain default changed.

The full host suite passed 157 checks. Final single-read Enter-state capture was verified by four targeted checks, including actual key_event/long-hold dispatch and real step/precision functions, followed by a successful final ADS/TASKING build (zero errors/warnings). Render contracts intentionally update only the editing footer and parameter-value formatting; other render functions and image rendering remain checked unchanged. No board/LCD/button test was performed.


## 2026-11-04 faster step hold

Parameter/Path editing now lowers only the Enter key's hold threshold from 1000ms to 300ms. It restores the default outside editing; other keys retain 1000ms. Threshold changes are cached and the existing one-cycle-per-hold menu logic is retained. Six focused real scanner/menu/step tests passed, including 300ms vs 1000ms, release/short press, 7ms scan rounding, invalid thresholds, edit-only override/restoration and continuous-hold suppression. ADS/TASKING build passed with zero errors/warnings. No hardware timing/feel test performed. Existing controller tuning and private network/project edits are excluded.
