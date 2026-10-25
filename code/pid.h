#ifndef PID_H
#define PID_H

/*
 * Simple floating-point PID controller.
 *
 * kp, ki, kd             continuous gains
 * integral_limit        symmetric magnitude limit for the integral term
 * output_min/output_max output saturation limits
 * integral              integral term (already scaled by ki)
 * error_1/error_2       most recent and previous error samples
 * output                most recently produced output
 * initialized           non-zero after the first valid update
 *
 * Call pid_reset() before the first use.  pid_reset() clears only dynamic
 * state; the tuning fields are left untouched.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_min;
    float output_max;
    float integral;
    float error_1;
    float error_2;
    float output;
    float derivative; /* Last scaled D term, for variable-dt increments. */
    unsigned char initialized;
} pid_controller_t;

void pid_reset(pid_controller_t *pid);

/* Positional form: recomputes the output from the absolute error each time. */
float pid_position_update(pid_controller_t *pid, float target, float measured, float dt);

/* Incremental form: adds an increment to the previous clamped output. */
float pid_incremental_update(pid_controller_t *pid, float target, float measured, float dt);

#endif
