/* Set QCAR_BOOT_IMU_WIFI=0 to restore the menu/control entry. */
#include "zf_common_headfile.h"
#include "imu.h"
#include "wifispi.h"
#include "IfxStm.h"

#if QCAR_BOOT_IMU_WIFI
volatile uint32 imu_wifi_boot_stage;
volatile uint32 imu_wifi_boot_loops;

static void imu_wifi_boot_display(void)
{
    char line[40];
    float roll, pitch, yaw, progress;
    uint32 sequence;
    int32 status;
    uint32 irq=interrupt_global_disable();
    roll=imu_roll_deg; pitch=imu_pitch_deg; yaw=imu_yaw_deg;
    progress=imu_calibration_percent; sequence=imu_sample_sequence;
    status=imu_attitude_status;
    interrupt_global_enable(irq);
    ips200_show_string(0, 0, "AUTO IMU UDP 1023-A");
    snprintf(line,sizeof(line),"Boot:%lu IMU:%ld Cal:%5.1f%%      ",
             (unsigned long)imu_wifi_boot_stage,(long)status,progress);
    ips200_show_string(0, 22, line);
    snprintf(line,sizeof(line),"Pitch:%7.2f Roll:%7.2f   ",roll,pitch);
    ips200_show_string(0, 44, line);
    snprintf(line,sizeof(line),"Yaw:%7.2f Seq:%10lu    ",yaw,(unsigned long)sequence);
    ips200_show_string(0, 66, line);
    snprintf(line,sizeof(line),"WiFi:%d Err:%d Try:%lu         ",
             imu_wifi_status,imu_wifi_last_error,(unsigned long)imu_wifi_init_attempts);
    ips200_show_string(0, 88, line);
    snprintf(line,sizeof(line),"RC:%d FW:%.11s               ",imu_wifi_module_result,wifi_spi_version);
    ips200_show_string(0, 110, line);
    snprintf(line,sizeof(line),"Tx:%lu Fields:%lu Stream:%lu     ",
             (unsigned long)imu_wifi_tx_packets,(unsigned long)wifi_telemetry_channel_count,
             (unsigned long)wifi_telemetry_stream_enabled);
    ips200_show_string(0, 132, line);
    snprintf(line,sizeof(line),"Cmd:%02lX Fail:%lu Ack:%02lX Len:%lu   ",
             (unsigned long)wifi_spi_config_command,(unsigned long)wifi_spi_config_error,
             (unsigned long)wifi_spi_config_reply,(unsigned long)wifi_spi_config_reply_length);
    ips200_show_string(0, 154, line);
    snprintf(line,sizeof(line),"PC:%s:%s",QCAR_WIFI_TARGET_IP,QCAR_WIFI_TARGET_PORT);
    ips200_show_string(0, 176, line);
    snprintf(line,sizeof(line),"Module:%-24.24s",wifi_spi_ip_addr_port);
    ips200_show_string(0, 198, line);
    ips200_show_string(0, 220, "Auto retry; motor output OFF");
}

static void imu_wifi_boot_connect(void)
{
    imu_wifi_boot_stage=5;
    imu_wifi_boot_display();
    wifispi_set_progress_hook(imu_wifi_boot_display);
    wifispi_telemetry_init(); /* default: roll_deg,pitch_deg,yaw_deg */
    wifispi_set_progress_hook(NULL);
    imu_wifi_boot_stage=wifispi_is_ready()?6u:7u;
    if (wifispi_is_ready()) wifispi_telemetry_announce();
    imu_wifi_boot_display();
}
#endif

int core0_main(void)
{
    clock_init();
    debug_init();
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_6X8_FONT);
#if QCAR_BOOT_IMU_WIFI
    {
        uint32 per_ms,last_attempt,last_draw,last_announce;
        imu_wifi_boot_stage=1;
        imu_wifi_boot_display();
        /* Establish zero output before any sensor/network operation. */
        motor_int(); motor_m_ctrl(0);
        small_driver_uart_init(); small_driver_set_duty(0,0);
        cam_flag=tele_flag=turn_flag=shacheflag=0;
        roll_rate_test_stop();
        imu_wifi_boot_stage=3;
        imu_wifi_boot_display();
        imu_init();
        imu_wifi_boot_stage=4;
        imu_wifi_boot_display();
        cpu_wait_event_ready();
        interrupt_global_enable(0);
        /* Only start IMU sampling, NEVER the control-loop timer CH0. */
        if (imu_attitude_status!=IMU_ATTITUDE_INIT_FAILED) pit_init(CCU60_CH1,5000);
        per_ms=(uint32)(IfxStm_getFrequency(&MODULE_STM0)/1000.0f);
        imu_wifi_boot_connect();
        last_attempt=last_draw=last_announce=IfxStm_getLower(&MODULE_STM0);
        while (TRUE) {
            uint32 now=IfxStm_getLower(&MODULE_STM0);
            imu_service();
            if ((!wifispi_is_ready() || imu_wifi_status==-4) &&
                (uint32)(now-last_attempt)>=per_ms*5000u) {
                imu_wifi_boot_connect();
                now=IfxStm_getLower(&MODULE_STM0);
                last_attempt=last_announce=now;
            }
            wifispi_telemetry_service();
            now=IfxStm_getLower(&MODULE_STM0);
            if (wifispi_is_ready() && (uint32)(now-last_announce)>=per_ms*1000u) {
                wifispi_telemetry_announce(); last_announce=now;
            }
            if ((uint32)(now-last_draw)>=per_ms*200u) {
                imu_wifi_boot_display(); last_draw=now;
            }
            ++imu_wifi_boot_loops;
            system_delay_ms(10);
        }
    }
#else
    ips200_show_string(0,0,"Qcar starting");
    monotroch_init();
    /* Menus/control do not require the optional image worker to be running. */
    cpu_init_notify();
    interrupt_global_enable(0);
    menu_auto_telemetry_init();
    while (TRUE) menu(key1_count);
#endif
}
