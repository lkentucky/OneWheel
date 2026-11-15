#ifndef STATION_WIFI_H
#define STATION_WIFI_H
#include "config.h"
#include "zf_common_headfile.h"
#if STATION_WIFI_ENABLED
typedef struct
{
    uint32 wait_count[3]; /* DATA, UDP flush, flush ACK */
    uint32 wait_max_ms[3];
    uint32 poll_gap_max_ms;
    uint32 submitted;
    uint32 acknowledged;
    uint32 errors;
    uint8 phase;
    uint8 ready;
} station_wifi_diag_t;

void station_wifi_get_diag(station_wifi_diag_t *out);
#endif
/* Boot only, before the control timer. No reconnect/init in a running loop. */
uint8 station_wifi_init(void);
/* Foreground only. One bounded SPI transaction, no wait for module readiness.
 * Return bytes submitted, zero if busy/receiving/flushing, -1 on invalid reply.
 * rx must hold STATION_WIFI_RX_MAX bytes. All replies are delivered, even while
 * flushing UDP. Caller must retain tx data until a positive return.
 */
int station_wifi_step(const uint8 *tx, uint16 length, uint8 *rx, uint16 *received, uint32 now_ms);
#endif
