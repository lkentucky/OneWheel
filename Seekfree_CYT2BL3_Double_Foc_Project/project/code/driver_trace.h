/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef DRIVER_TRACE_H
#define DRIVER_TRACE_H
#include <stdint.h>
#define DRIVER_TRACE_NONE UINT16_MAX
void driver_trace_init(void);
/* Called atomically with the corresponding duty assignment. */
void driver_trace_on_rx_byte(uint8_t byte);
void driver_trace_on_command(int16_t left, int16_t right);
uint16_t driver_trace_generation(void);
void driver_trace_on_encoder(int32_t raw, int32_t delta);
void driver_trace_on_pwm_written(uint16_t generation);
/* ISR only queues text requests. Main loop formats/sends replies at zero duty. */
int driver_trace_text(const uint8_t *text);
void driver_trace_service(void);
#endif
