/* Menu startup; select mode 3 for the isolated roll-rate loop test. */
#include "zf_common_headfile.h"

int core0_main(void)
{
    clock_init();
    debug_init();
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_6X8_FONT);
    ips200_show_string(0, 0, "Qcar starting");
    monotroch_init();
    cpu_wait_event_ready();
    while (TRUE) menu(key1_count);
}
