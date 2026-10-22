/* 应用入口：CPU0 通信；CPU1 按键/遥测；控制计算保留在 1ms 中断。 */
#include "zf_common_headfile.h"
#include "IfxStm.h"
#include <stdio.h>

volatile uint8 app_cpu0_ready;
volatile uint8 app_cpu1_stage;
volatile uint32 app_cpu0_poll_count;
static uint32 boot_report_ticks;
static uint32 boot_report_period;
static uint8 boot_report_first = 1;

static void app_boot_diagnostics_poll(void)
{
    uint32 now;
    char message[128];
    if (!APP_BOOT_DIAGNOSTICS_ENABLE) { return; }
    now = IfxStm_getLower(&MODULE_STM0);
    if (!boot_report_first && (uint32)(now - boot_report_ticks) < boot_report_period) { return; }
    boot_report_first = 0;
    boot_report_ticks = now;
    snprintf(message, sizeof(message),
             "[BOOT] c0_polls=%lu c1_stage=%u tick=%lu imu=%u run=%u wifi=%u fault=%lu fb=%u\r\n",
             (unsigned long)app_cpu0_poll_count, (unsigned)app_cpu1_stage,
             (unsigned long)system_count, (unsigned)app_imu_ready,
             (unsigned)run_flag, (unsigned)wifi_link_online(),
             (unsigned long)safety_output_faults(), (unsigned)safety_output_feedback_ok());
    uart_write_buffer(DEBUG_UART_INDEX, (const uint8 *)message, (uint32)strlen(message));
    /* 同一条状态行也送进 WiFi 日志，没有调试串口时仍能远程判读。 */
    if (APP_WIFI_ENABLE) { wifi_link_log("%s", message); }
}

volatile uint8 app_imu_ready;
static uint8 app_new_frame;
static uint8 camera_ready;
static uint8 last_reported_run;
static uint32 cpu1_ticks_per_ms;
static uint32 cpu1_last_ticks;
static uint32 cpu1_tick_remainder;
static uint32 cpu1_wall_ms;
static uint32 last_key_ms;
static uint32 last_telemetry_ms;
static uint32 last_status_ms;
static uint32 last_status_tick;

/* Background diagnostics must not depend on the control PIT heartbeat.
 * Poll the free-running STM and retain sub-ms ticks, including lower-word wrap. */
static uint32 app_cpu1_now_ms(void)
{
    uint32 now = IfxStm_getLower(&MODULE_STM0);
    uint32 elapsed = now - cpu1_last_ticks;
    cpu1_last_ticks = now;
    cpu1_wall_ms += elapsed / cpu1_ticks_per_ms;
    cpu1_tick_remainder += elapsed % cpu1_ticks_per_ms;
    if (cpu1_tick_remainder >= cpu1_ticks_per_ms)
    {
        cpu1_wall_ms++;
        cpu1_tick_remainder -= cpu1_ticks_per_ms;
    }
    return cpu1_wall_ms;
}

void app_cpu0_init(void)
{
    clock_init();
    debug_init();
    interrupt_global_enable(0);
    if (APP_WIFI_ENABLE) { wifi_link_init(); }
    boot_report_period = (uint32)(IfxStm_getFrequency(&MODULE_STM0) / 1000.0f) * APP_BOOT_DIAGNOSTICS_PERIOD_MS;
    if (!boot_report_period) { boot_report_period = 1; }
    if (APP_BOOT_DIAGNOSTICS_ENABLE)
    {
        static const char banner[] = "[BOOT] OneWheel PWM_DIR " __DATE__ " " __TIME__ " CPU0 ready\r\n";
        uart_write_buffer(DEBUG_UART_INDEX, (const uint8 *)banner, sizeof(banner) - 1u);
    }
    /* CStart launches CPU1 before core0_main. Publish only after STM delay
     * routing, debug UART and the shared WiFi mailbox are initialized. */
    __dsync();
    app_cpu0_ready = 1;
    __dsync();
}

void app_cpu0_poll(void)
{
    app_cpu0_poll_count++;
    app_boot_diagnostics_poll();
    if (APP_WIFI_ENABLE) { wifi_link_service(); }
    else { system_delay_us(WIFI_LINK_CPU0_DELAY_US); }
}

void app_init(void)
{
    uint8 imu_attempt;
    disable_Watchdog();
    app_cpu1_stage = 1;
    /* Do not call peripheral/interrupt-backed delay code before CPU0 has
     * initialized both STM services. This wait itself needs no timer. */
    while (!app_cpu0_ready) { __dsync(); }
    __dsync();
    interrupt_global_enable(0);
    run_flag = 0;
    cpu1_ticks_per_ms = (uint32)(IfxStm_getFrequency(&MODULE_STM0) / 1000.0f);
    if (!cpu1_ticks_per_ms) { cpu1_ticks_per_ms = 1; }
    cpu1_last_ticks = IfxStm_getLower(&MODULE_STM0);
    cpu1_tick_remainder = cpu1_wall_ms = 0;
    last_key_ms = last_telemetry_ms = last_status_ms = 0;
    last_status_tick = system_count;
    app_cpu1_stage = 2;
    Motor_init();
    app_cpu1_stage = 3;
    small_driver_uart_init();
    app_cpu1_stage = 4;
    balance_cascade_init();
    app_cpu1_stage = 5;
    app_imu_ready = 0;
    for (imu_attempt = 0; imu_attempt < APP_IMU_INIT_ATTEMPTS && !app_imu_ready; imu_attempt++)
    {
        if (imu660rb_init() == 0) { app_imu_ready = 1; }
        else if (imu_attempt + 1u < APP_IMU_INIT_ATTEMPTS)
        {
            system_delay_ms(APP_IMU_RETRY_DELAY_MS);
        }
    }
    if (app_imu_ready)
    {
        imu660rb_get_acc();
        imu660rb_get_gyro();
        icm_init();
    }
    app_cpu1_stage = 6;
    if (APP_KEYS_ENABLE) { key_init(APP_KEY_SCAN_PERIOD_MS); }
    app_cpu1_stage = 7;
    if (APP_WIRELESS_UART_ENABLE) { wireless_uart_init(); }
    if (APP_IPS200_ENABLE) { ips200_init(DISPLAY_INTERFACE); }
    if (APP_CAMERA_ENABLE) { camera_ready = (mt9v03x_init() == 0) ? 1u : 0u; }
    __dsync();
    app_cpu1_stage = 8;
    safety_output_init();
    pit_ms_init(CCU61_CH0, SAFETY_OUTPUT_PERIOD_MS);
    pit_ms_init(CCU60_CH0, CONTROL_PERIOD_MS);
    app_cpu1_stage = 9;
    if (APP_WIFI_ENABLE)
    {
        wifi_link_cpu1_log(app_imu_ready ?
            "[APP] UPRIGHT: K1 start, K2/K3/K4 stop; target=0 by default" :
            "[APP] IMU init failed; motor start disabled, telemetry available");
    }
}

void app_buttons_poll(void)
{
    uint32 now = app_cpu1_now_ms();
    uint8 start, stop;
    if (!APP_KEYS_ENABLE || (uint32)(now - last_key_ms) < APP_KEY_SCAN_PERIOD_MS) { return; }
    last_key_ms = now;
    key_scanner();
    start = (key_get_state(KEY_1) == KEY_SHORT_PRESS);
    stop = (key_get_state(KEY_2) == KEY_SHORT_PRESS ||
            key_get_state(KEY_3) == KEY_SHORT_PRESS ||
            key_get_state(KEY_4) == KEY_SHORT_PRESS);
    key_clear_all_state();
    if (stop) { control_request_stop(); }
    else if (start && !control_request_start() && APP_WIFI_ENABLE)
    {
        wifi_link_cpu1_log("[APP] start refused: check drive/IMU/warmup, heartbeat, feedback and fault latch");
    }
}

void app_camera_poll(void)
{
    app_new_frame = 0;
    if (!APP_CAMERA_ENABLE || !camera_ready || !mt9v03x_finish_flag) { return; }
    mt9v03x_finish_flag = 0;
    memcpy(base_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    img_threshold = Ostu(base_image);
    if (img_threshold < CAMERA_THRESHOLD_MIN) { img_threshold = CAMERA_THRESHOLD_MIN; }
    Image_Binarization(img_threshold);
    detect_beacon(image_binary, &beacon);
    error_back = erro;
    erro_yawan_back = erro_yawan;
    app_new_frame = 1;
}

void app_camera_display_poll(void)
{
    uint16 x, y, last_x, last_y;
    if (!APP_CAMERA_DISPLAY_ENABLE || !APP_IPS200_ENABLE || !app_new_frame) { return; }
    x = (uint16)(beacon.centerX / DISPLAY_IMAGE_DIVISOR);
    y = (uint16)(beacon.centerY / DISPLAY_IMAGE_DIVISOR);
    last_x = (uint16)(beacon.last_centerX / DISPLAY_IMAGE_DIVISOR);
    last_y = (uint16)(beacon.last_centerY / DISPLAY_IMAGE_DIVISOR);
    ips200_show_gray_image(0, 0, image_binary[0], MT9V03X_W, MT9V03X_H,
                          MT9V03X_W / DISPLAY_IMAGE_DIVISOR, MT9V03X_H / DISPLAY_IMAGE_DIVISOR, 0);
    ips200_draw_point(x, y, RGB565_RED);
    ips200_draw_line(x, y, x + DISPLAY_MARKER_SIZE, y, RGB565_RED);
    ips200_draw_line(x, y, x, y + DISPLAY_MARKER_SIZE, RGB565_RED);
    ips200_draw_line(x + DISPLAY_MARKER_SIZE, y, x + DISPLAY_MARKER_SIZE, y + DISPLAY_MARKER_SIZE, RGB565_RED);
    ips200_draw_line(x, y + DISPLAY_MARKER_SIZE, x + DISPLAY_MARKER_SIZE, y + DISPLAY_MARKER_SIZE, RGB565_RED);
    ips200_draw_line(last_x, last_y, last_x + DISPLAY_MARKER_SIZE, last_y, RGB565_GREEN);
    ips200_draw_line(last_x, last_y, last_x, last_y + DISPLAY_MARKER_SIZE, RGB565_GREEN);
    ips200_draw_line(last_x + DISPLAY_MARKER_SIZE, last_y, last_x + DISPLAY_MARKER_SIZE, last_y + DISPLAY_MARKER_SIZE, RGB565_GREEN);
    ips200_draw_line(last_x, last_y + DISPLAY_MARKER_SIZE, last_x + DISPLAY_MARKER_SIZE, last_y + DISPLAY_MARKER_SIZE, RGB565_GREEN);
    ips200_show_float(DISPLAY_ERROR_X, DISPLAY_ERROR_Y, erro, 4, 2);
}

void app_telemetry_poll(void)
{
    uint32 now = app_cpu1_now_ms();
    if (!APP_WIFI_ENABLE || (uint32)(now - last_telemetry_ms) < APP_TELEMETRY_PERIOD_MS) { return; }
    last_telemetry_ms = now;
    if (run_flag != last_reported_run)
    {
        last_reported_run = run_flag;
        wifi_link_cpu1_log(run_flag ? "[APP] RUNNING" : "[APP] STOPPED");
    }
    wifi_link_cpu1_service(app_new_frame);
    if (APP_CSV_DEBUG_ENABLE && APP_WIRELESS_UART_ENABLE)
    {
        send_two_floats(balance_cascade.cascade_value.filtering_angle,
                        balance_cascade.pitch_cascade_value.filtering_angle, encoder_data_1);
    }
}

static void app_status_log_poll(void)
{
    uint32 now = app_cpu1_now_ms();
    uint32 tick;
    char message[112];
    if (!APP_WIFI_ENABLE || !APP_BOOT_DIAGNOSTICS_ENABLE ||
        (uint32)(now - last_status_ms) < APP_STATUS_LOG_PERIOD_MS) { return; }
    tick = system_count;
    snprintf(message, sizeof(message), "[APP] st=%u tick=%lu dtick=%lu dms=%lu imu=%u run=%u fault=%lu fb=%u",
             (unsigned)app_cpu1_stage, (unsigned long)tick,
             (unsigned long)(tick - last_status_tick), (unsigned long)(now - last_status_ms),
             (unsigned)app_imu_ready, (unsigned)run_flag,
             (unsigned long)safety_output_faults(), (unsigned)safety_output_feedback_ok());
    last_status_ms = now;
    last_status_tick = tick;
    wifi_link_cpu1_log(message);
}

void app_poll(void)
{
    app_new_frame = 0;
    app_buttons_poll();
    /* 暂时关闭视觉；恢复时配置 config.h 并取消下面两行注释。 */
    // app_camera_poll();
    // app_camera_display_poll();
    app_telemetry_poll();
    app_status_log_poll();
    system_delay_ms(APP_MAIN_LOOP_DELAY_MS);
}
