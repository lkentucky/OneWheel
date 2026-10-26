/* CYT2BL3 link: stock text compatibility, binary RX and bounded parsing. */
#include "small_driver_uart_control.h"
#include <stdio.h>

volatile uint32 small_driver_rx_bytes;
volatile uint32 small_driver_speed_frames;
volatile uint32 small_driver_feedback_age_ms = 0xffffffffu;
volatile uint8 small_driver_feedback_fresh;
volatile float small_driver_test_left, small_driver_test_right;
small_device_value_struct motor_value;
static volatile uint32 driver_ms;
static uint32 last_speed_query_ms;
static char speed_line[24];
static uint8 speed_line_used, speed_line_overflow;

/* Always-on key timer: independent of control Start/Stop, never sends UART. */
void small_driver_tick_1ms(void)
{
    ++driver_ms;
    if (small_driver_feedback_age_ms != 0xffffffffu) ++small_driver_feedback_age_ms;
    if (small_driver_feedback_age_ms > QCAR_DRIVER_FEEDBACK_TIMEOUT_MS)
        small_driver_feedback_fresh = 0;
}
static void publish_speed(int16 left, int16 right)
{
    uint32 irq = interrupt_global_disable();
    motor_value.receive_left_speed_data = left;
    motor_value.receive_right_speed_data = right;
    ++small_driver_speed_frames;
    small_driver_feedback_age_ms = 0;
    small_driver_feedback_fresh = 1;
    interrupt_global_enable(irq);
}
/* Malformed text never masquerades as a zero-speed report. */
static uint8 parse_speed_number(const char **cursor, int16 *value)
{
    const char *p = *cursor;
    int negative = 0;
    int32 n = 0;
    unsigned digits = 0;
    if (*p == '-' || *p == '+') { negative = *p == '-'; ++p; }
    while (*p >= '0' && *p <= '9') {
        if (++digits > 5u) return 0;
        n = n * 10 + (*p++ - '0');
        if (n > (negative ? 32768 : 32767)) return 0;
    }
    if (!digits) return 0;
    *cursor = p; *value = (int16)(negative ? -n : n);
    return 1;
}
static void receive_speed_text(uint8 byte)
{
    if (byte == '\n') {
        const char *p = speed_line;
        int16 left, right;
        speed_line[speed_line_used] = '\0';
        if (!speed_line_overflow && parse_speed_number(&p, &left) && *p++ == ',' &&
            parse_speed_number(&p, &right)) {
            if (*p == '\r') ++p;
            if (*p == '\0') publish_speed(left, right);
        }
        speed_line_used = speed_line_overflow = 0;
    } else if (!speed_line_overflow) {
        if (speed_line_used < sizeof(speed_line) - 1u)
            speed_line[speed_line_used++] = (char)byte;
        else speed_line_overflow = 1;
    }
}
static void receive_speed_byte(uint8 byte)
{
    uint8 *frame = motor_value.receive_data_buffer;
    unsigned i;
    if (!motor_value.receive_data_count) {
        if (byte != 0xA5) { receive_speed_text(byte); return; }
        speed_line_used = speed_line_overflow = 0;
    }
    frame[motor_value.receive_data_count++] = byte;
    if (motor_value.receive_data_count < 7u) return;
    motor_value.sum_check_data = 0;
    for (i = 0; i < 6u; ++i) motor_value.sum_check_data += frame[i];
    if (frame[0] == 0xA5 && motor_value.sum_check_data == frame[6]) {
        if (frame[1] == 0x02)
            publish_speed((int16)((frame[2] << 8) | frame[3]),
                          (int16)((frame[4] << 8) | frame[5]));
        motor_value.receive_data_count = 0;
        return;
    }
    for (i = 1; i < 7u; ++i) {
        if (frame[i] == 0xA5) {
            memmove(frame, frame + i, 7u - i);
            motor_value.receive_data_count = (uint8)(7u - i);
            return;
        }
    }
    motor_value.receive_data_count = 0;
}
void uart_control_callback(void)
{
    uint8 byte;
    unsigned budget = 32u;
    while (budget-- && uart_query_byte(SMALL_DRIVER_UART, &byte)) {
        ++small_driver_rx_bytes;
        receive_speed_byte(byte);
    }
}
void small_driver_get_speed(void)
{
#if QCAR_DRIVER_TEXT_DUTY
    static const uint8 query[] = "GET-SPEED\n";
    uart_write_buffer(SMALL_DRIVER_UART, query, sizeof(query) - 1u);
#else
    static const uint8 query[7] = {0xA5, 2, 0, 0, 0, 0, 0xA7};
    uart_write_buffer(SMALL_DRIVER_UART, query, sizeof(query));
#endif
    last_speed_query_ms = driver_ms;
}
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    int32 left = left_duty, right = right_duty;
    if (left > 10000) left = 10000;
    if (left < -10000) left = -10000;
    if (right > 10000) right = 10000;
    if (right < -10000) right = -10000;
    left *= QCAR_FLYWHEEL_LEFT_SIGN;
    right *= QCAR_FLYWHEEL_RIGHT_SIGN;
#if QCAR_DRIVER_TEXT_DUTY
    char command[32];
    int length = snprintf(command, sizeof(command), "SET-DUTY,%d,%d\n", (int)left, (int)right);
    if (length > 0 && (unsigned)length < sizeof(command))
        uart_write_buffer(SMALL_DRIVER_UART, (const uint8 *)command, (uint32)length);
#else
    uint8 frame[7] = {0xA5, 1, 0, 0, 0, 0, 0};
    unsigned i;
    frame[2] = (uint8)((uint32)left >> 8); frame[3] = (uint8)left;
    frame[4] = (uint8)((uint32)right >> 8); frame[5] = (uint8)right;
    for (i = 0; i < 6u; ++i) frame[6] += frame[i];
    uart_write_buffer(SMALL_DRIVER_UART, frame, sizeof(frame));
#endif
    /* Both active control and stopped bench output renew a reset driver's query. */
    if ((uint32)(driver_ms - last_speed_query_ms) >= QCAR_DRIVER_SPEED_QUERY_MS)
        small_driver_get_speed();
}
void small_driver_uart_init(void)
{
    uint32 irq = interrupt_global_disable();
    memset(&motor_value, 0, sizeof(motor_value));
    small_driver_rx_bytes = small_driver_speed_frames = 0;
    small_driver_feedback_age_ms = 0xffffffffu;
    small_driver_feedback_fresh = 0;
    speed_line_used = speed_line_overflow = 0;
    last_speed_query_ms = driver_ms;
    interrupt_global_enable(irq);
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_TX, SMALL_DRIVER_RX);
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);
    small_driver_set_duty(0, 0);
    small_driver_get_speed();
}
