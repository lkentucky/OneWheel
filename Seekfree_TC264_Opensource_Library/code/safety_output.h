#ifndef ONEWHEEL_SAFETY_OUTPUT_H
#define ONEWHEEL_SAFETY_OUTPUT_H
#include "zf_common_typedef.h"

enum {
    SAFETY_FAULT_FILTER = 1u,
    SAFETY_FAULT_CONTROL_TIMEOUT = 2u,
    SAFETY_FAULT_FEEDBACK = 4u,
    SAFETY_FAULT_UART_TX = 8u,
    SAFETY_FAULT_COMMAND = 16u
};

void safety_output_init(void);
void safety_output_tick(void); /* CPU1 CCU61_CH0: sole runtime output owner. */
void safety_output_publish(int16 left, int16 right, int16 travel, uint32 epoch);
void safety_output_request_stop(void);
void safety_output_trip(uint32 faults);
uint32 safety_output_epoch(void);
uint32 safety_output_faults(void);
uint8 safety_output_can_start(void);
uint8 safety_output_feedback_ok(void);
uint32 safety_output_ticks_per_ms(void);

#endif
