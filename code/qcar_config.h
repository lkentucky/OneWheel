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
#define QCAR_WIFI_PASSWORD "zyzzyz520520"
#endif
#ifndef QCAR_WIFI_TARGET_IP
#define QCAR_WIFI_TARGET_IP "192.168.0.112"
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
