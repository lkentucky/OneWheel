/* Derived from Seekfree TC264 dual-core example and feng0511/monotroch. */
#include "zf_common_headfile.h"
#if !QCAR_WIFI_ENABLED
static uint8 wifi_ready;
static seekfree_assistant_camera_struct camera;
#endif
static uint8 image_copy[MT9V03X_H][MT9V03X_W];
void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);
    cpu_wait_event_ready();
    while (TRUE)
    {
        if (cam_flag && mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            memcpy(image_copy, mt9v03x_image, sizeof(image_copy));
            imageinfo.image = image_copy[0];
            if (img_process_params.img_process_mode == day)
                image_process_time_pro(&imageinfo, &turn_info);
            else
                image_process_time(&imageinfo, &turn_info);
        }
#if !QCAR_WIFI_ENABLED
        else if (!cam_flag && wifi_spi_flag)
        {
            if (!wifi_ready)
            {
                if (wifi_spi_init(QCAR_WIFI_SSID, QCAR_WIFI_PASSWORD)) continue;
                if (wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP,
                    WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT)) continue;
                seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
                seekfree_assistant_camera_config(
                    &camera, SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X, MT9V03X_W, MT9V03X_H, image_copy[0]);
                wifi_ready = 1;
            }
            if (mt9v03x_finish_flag)
            {
                mt9v03x_finish_flag = 0;
                memcpy(image_copy, mt9v03x_image, sizeof(image_copy));
                seekfree_assistant_camera_send(&camera);
            }
        }
#endif
    }
}
