#ifndef QCAR_CONFIG_H
#define QCAR_CONFIG_H
/* Default TC264 wiring proposal; confirm against your actual PCB. */
#define QCAR_DRIVER_UART UART_3
#define QCAR_DRIVER_TX UART3_TX_P15_6
#define QCAR_DRIVER_RX UART3_RX_P15_7
#define QCAR_DRIVER_BAUD 460800
#define QCAR_MOTOR_PWM ATOM0_CH3_P21_5
#define QCAR_MOTOR_DIR P21_4
#define QCAR_KEY1 P20_6
#define QCAR_KEY2 P20_7
#define QCAR_KEY3 P11_2
#define QCAR_EC11_KEY P11_3
/* 1: initialize the IMU and start the 5 ms sampling timer at boot.
 * 0: leave the IMU off until Start / IMU preview. Use 0 to bisect a
 *    boot-time menu freeze between the IMU path and the rest. */
#ifndef QCAR_BOOT_IMU_INIT
#define QCAR_BOOT_IMU_INIT 1
#endif
/* 1: load saved parameters from DFlash at boot. 0: keep the compiled-in
 * defaults only. Setting 0 isolates the menu from the boot-time flash
 * read (Storage -> Load also does nothing while this is 0). */
#ifndef QCAR_BOOT_LOAD_FLASH
#define QCAR_BOOT_LOAD_FLASH 1
#endif
/* 1: read/write the gyro-test extension values in DFlash pages 77..85.
 * 0 (default): skip them; the five values keep their compiled-in defaults
 * and the main parameter block (pages 0..76) still works normally.
 * Reading page 83 stalls the CPU on the current board, so this stays off
 * until that page/sector is recovered or the values are moved. */
#ifndef QCAR_FLASH_EXT
#define QCAR_FLASH_EXT 0
#endif
/* 1: automatic IMU+WiFi diagnostic boot, 0: original menu boot. */
#ifndef QCAR_BOOT_IMU_WIFI
#define QCAR_BOOT_IMU_WIFI 0
#endif
#if QCAR_BOOT_IMU_WIFI != 0 && QCAR_BOOT_IMU_WIFI != 1
#error QCAR_BOOT_IMU_WIFI must be 0 or 1
#endif
/* UDP telemetry owns WiFi-SPI on CPU0. Set 0 to restore the legacy CPU1
 * TCP image preview (cam_flag=0, wifi_spi_flag=1). Never use both owners. */
#ifndef QCAR_WIFI_ENABLED
#define QCAR_WIFI_ENABLED 1
#endif
#if QCAR_WIFI_ENABLED != 0 && QCAR_WIFI_ENABLED != 1
#error QCAR_WIFI_ENABLED must be 0 or 1
#endif
#ifndef QCAR_WIFI_SSID
#define QCAR_WIFI_SSID "HDUASC"
#endif
#ifndef QCAR_WIFI_PASSWORD
#define QCAR_WIFI_PASSWORD "zyz520520"
#endif
#ifndef QCAR_WIFI_TARGET_IP
#define QCAR_WIFI_TARGET_IP "192.168.0.107"
#endif
#ifndef QCAR_WIFI_TARGET_PORT
#define QCAR_WIFI_TARGET_PORT "8081"
#endif
#ifndef QCAR_WIFI_LOCAL_PORT
#define QCAR_WIFI_LOCAL_PORT "5001"
#endif
#ifndef QCAR_WIFI_STARTUP_DELAY_MS
#define QCAR_WIFI_STARTUP_DELAY_MS 300u
#endif
#ifndef QCAR_WIFI_INIT_ATTEMPTS
#define QCAR_WIFI_INIT_ATTEMPTS 3u
#endif
#ifndef QCAR_WIFI_RETRY_DELAY_MS
#define QCAR_WIFI_RETRY_DELAY_MS 500u
#endif
#ifndef QCAR_WIFI_PERIOD_MS
#define QCAR_WIFI_PERIOD_MS 10u
#endif
#if QCAR_WIFI_PERIOD_MS < 2 || QCAR_WIFI_PERIOD_MS > 1000
#error QCAR_WIFI_PERIOD_MS must be between 2 and 1000
#endif
#if QCAR_WIFI_INIT_ATTEMPTS < 1
#error QCAR_WIFI_INIT_ATTEMPTS must be positive
#endif
#endif
