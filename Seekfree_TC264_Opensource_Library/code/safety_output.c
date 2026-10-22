/* CPU0 publishes completed calculations. CPU1 owns PWM and runtime UART TX.
 * No locks, interrupt-backed delays or polling UART writes in the output ISR. */
#include "zf_common_headfile.h"
#include "IfxStm.h"

typedef struct {
    uint32 sequence;
    uint32 ticks;
    uint32 epoch;
    int16 left, right, travel;
    uint8 valid;
} output_command;

static volatile output_command mailbox;
static output_command applied_command;
static volatile uint32 stop_generation;
static volatile uint32 fault_latch;
static uint32 ticks_per_ms;
static uint32 last_query_ticks;
static uint32 tx_busy_ticks;
static uint8 tx_was_busy;

static uint32 atomic_increment(volatile uint32 *value)
{
    uint32 old;
    do { old = *value; }
    while ((uint32)__cmpAndSwap((unsigned int *)value, old + 1u, old) != old);
    __dsync();
    return old + 1u;
}

uint32 safety_output_epoch(void) { return stop_generation; }
uint32 safety_output_faults(void) { return fault_latch; }
uint32 safety_output_ticks_per_ms(void) { return ticks_per_ms; }

void safety_output_request_stop(void)
{
    atomic_increment(&stop_generation);
    run_flag = 0;
    __dsync();
}

void safety_output_trip(uint32 faults)
{
    uint32 old;
    do {
        old = fault_latch;
        if ((old & faults) == faults) { return; }
    } while ((uint32)__cmpAndSwap((unsigned int *)&fault_latch, old | faults, old) != old);
    __dsync();
    safety_output_request_stop(); /* Latched until board reset. */
}

uint8 safety_output_feedback_ok(void)
{
    if (!(CONTROL_ROLL_ENABLE || CONTROL_YAW_ENABLE)) { return 1; }
    return small_driver_feedback_fresh(IfxStm_getLower(&MODULE_STM0), ticks_per_ms);
}

void safety_output_init(void)
{
    ticks_per_ms = (uint32)(IfxStm_getFrequency(&MODULE_STM0) / 1000.0f);
    if (!ticks_per_ms) { ticks_per_ms = 1u; }
    mailbox.valid = 0;
    mailbox.sequence = 0;
    applied_command.valid = 0;
    tx_was_busy = 0;
    last_query_ticks = IfxStm_getLower(&MODULE_STM0);
    /* Do not erase stop requests received before initialization. */
}

void safety_output_publish(int16 left, int16 right, int16 travel, uint32 epoch)
{
    mailbox.sequence++;
    __dsync();
    mailbox.left = left;
    mailbox.right = right;
    mailbox.travel = travel;
    mailbox.epoch = epoch;
    mailbox.ticks = IfxStm_getLower(&MODULE_STM0);
    mailbox.valid = 1;
    __dsync();
    mailbox.sequence++;
    __dsync();
}

uint8 safety_output_can_start(void)
{
    uint32 sequence = mailbox.sequence;
    uint32 ticks = mailbox.ticks;
    uint8 valid = mailbox.valid;
    __dsync();
    return ticks_per_ms && valid && !(sequence & 1u) && sequence == mailbox.sequence &&
        (uint32)(IfxStm_getLower(&MODULE_STM0) - ticks) < ticks_per_ms * CONTROL_HEARTBEAT_TIMEOUT_MS &&
        !fault_latch && safety_output_feedback_ok();
}

void safety_output_tick(void)
{
    uint32 now = IfxStm_getLower(&MODULE_STM0);
    uint32 sequence = mailbox.sequence;
    uint32 epoch = stop_generation;
    output_command copy;
    int16 left = 0, right = 0, travel = 0;
    uint8 running;

    if (!(sequence & 1u))
    {
        __dsync();
        copy.sequence = sequence;
        copy.left = mailbox.left; copy.right = mailbox.right; copy.travel = mailbox.travel;
        copy.epoch = mailbox.epoch; copy.ticks = mailbox.ticks; copy.valid = mailbox.valid;
        __dsync();
        if (sequence == mailbox.sequence) { applied_command = copy; }
    }
    running = (uint8)(APP_DRIVE_ENABLE && run_flag && !fault_latch && applied_command.valid && applied_command.epoch == epoch);
    if (run_flag && (!applied_command.valid ||
        (uint32)(now - applied_command.ticks) >= ticks_per_ms * CONTROL_HEARTBEAT_TIMEOUT_MS))
    {
        safety_output_trip(SAFETY_FAULT_CONTROL_TIMEOUT);
        running = 0;
    }
    if (running && !safety_output_feedback_ok())
    {
        safety_output_trip(SAFETY_FAULT_FEEDBACK);
        running = 0;
    }
    if (running && (func_abs(applied_command.left) > MOMENTUM_DUTY_LIMIT ||
        func_abs(applied_command.right) > MOMENTUM_DUTY_LIMIT ||
        func_abs(applied_command.travel) > PROTECT_TRAVEL_DUTY + MOTOR_DEADZONE_DUTY ||
        func_abs(applied_command.travel) > 10000))
    {
        safety_output_trip(SAFETY_FAULT_COMMAND);
        running = 0;
    }
    if (running && run_flag && epoch == stop_generation)
    {
        if (CONTROL_ROLL_ENABLE || CONTROL_YAW_ENABLE)
        {
            left = applied_command.left; right = applied_command.right;
        }
        if (CONTROL_PITCH_ENABLE || CONTROL_MOTOR_ENABLE) { travel = applied_command.travel; }
    }
    motor_output_service(travel, now, ticks_per_ms);
    /* Recheck after PWM work; no other runtime context writes this UART. */
    if (!run_flag || fault_latch || epoch != stop_generation) { left = right = 0; }
    if (small_driver_try_set_duty(left, right))
    {
        tx_was_busy = 0;
        if ((uint32)(now - last_query_ticks) >= ticks_per_ms * CONTROL_SPEED_PERIOD_MS &&
            small_driver_try_get_speed()) { last_query_ticks = now; }
    }
    else
    {
        if (!tx_was_busy) { tx_was_busy = 1; tx_busy_ticks = now; }
        if ((uint32)(now - tx_busy_ticks) >= ticks_per_ms * MOTOR_UART_TX_TIMEOUT_MS)
        {
            safety_output_trip(SAFETY_FAULT_UART_TX);
            motor_output_service(0, now, ticks_per_ms);
        }
    }
}
