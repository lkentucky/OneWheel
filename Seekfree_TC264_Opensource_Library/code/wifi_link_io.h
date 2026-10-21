#ifndef ONEWHEEL_WIFI_LINK_IO_H
#define ONEWHEEL_WIFI_LINK_IO_H

#include "zf_common_typedef.h"

typedef void (*wifi_link_rx_fn)(const uint8 *data, uint32 length);
typedef enum
{
    WIFI_LINK_IO_IDLE,
    WIFI_LINK_IO_BUSY,
    WIFI_LINK_IO_SENT,
    WIFI_LINK_IO_RECEIVED,
    WIFI_LINK_IO_ERROR
} wifi_link_io_result;

void wifi_link_io_init(wifi_link_rx_fn receive, uint32 ticks_per_us);
uint8 wifi_link_io_busy(void);
uint8 wifi_link_io_send(const uint8 *data, uint16 length, uint32 now_ms);
void wifi_link_io_receive(uint32 now_ms);
wifi_link_io_result wifi_link_io_poll(uint32 now_ms);
uint32 wifi_link_io_last_us(void);
uint32 wifi_link_io_peak_us(void);

#endif
