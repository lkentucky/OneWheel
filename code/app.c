/* CPU0 application: initialize once, service UI/network while IRQs control. */
#include "zf_common_headfile.h"
#include "app.h"
void app_init(void)
{
    clock_init();
    debug_init();
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE,RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_6X8_FONT);
    ips200_show_string(0,0,"OneWheel upright starting");
    menu_init();
    cpu_init_notify();
    interrupt_global_enable(0);
    menu_hardware_init(); /* zero outputs; IMU on; explicit Start required */
}
void app_poll(void)
{
    menu_poll(); /* IMU requests, keys, test outputs, WiFi, display, 10ms wait */
}
