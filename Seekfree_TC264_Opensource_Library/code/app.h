#ifndef ONEWHEEL_APP_H
#define ONEWHEEL_APP_H
#include "zf_common_typedef.h"
extern volatile uint8 app_imu_ready;
extern volatile uint8 app_cpu0_ready;
extern volatile uint8 app_cpu1_stage;
extern volatile uint32 app_cpu0_poll_count;
/* CPU1: 1=wait CPU0, 2=motor, 3=driver UART, 4=PID, 5=IMU,
 * 6=keys, 7=optional peripherals, 8=PIT, 9=main loop. */
void app_cpu0_init(void);
void app_cpu0_poll(void);
void app_init(void);
void app_poll(void);
void app_buttons_poll(void);
void app_camera_poll(void);
void app_camera_display_poll(void);
void app_telemetry_poll(void);
#endif
