#include "pid.h"

#include <float.h>
#include <stddef.h>

/* Portable finite test (works even when math.h has no isfinite). */
static int pid_is_finite(float x)
{
    return (x == x) && (x <= FLT_MAX) && (x >= -FLT_MAX);
}

static float pid_clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int pid_config_valid(const pid_controller_t *pid)
{
    if (!pid_is_finite(pid->kp) || !pid_is_finite(pid->ki) ||
        !pid_is_finite(pid->kd) || !pid_is_finite(pid->integral_limit) ||
        !pid_is_finite(pid->output_min) || !pid_is_finite(pid->output_max)) {
        return 0;
    }
    if (pid->integral_limit < 0.0f) {
        return 0;
    }
    if (pid->output_min > pid->output_max) {
        return 0;
    }
    return 1;
}

/* Clear dynamic state only, retaining the configuration. */
static void pid_clear_dynamic(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->error_1 = 0.0f;
    pid->error_2 = 0.0f;
    pid->output = 0.0f;
    pid->derivative = 0.0f;
    pid->initialized = 0;
}

void pid_reset(pid_controller_t *pid)
{
    if (pid != NULL) {
        pid_clear_dynamic(pid);
    }
}

/*
 * Shared validation.  On invalid input/config the dynamic state is reset and
 * zero is returned; the configuration fields are never modified.
 */
static int pid_update_valid(pid_controller_t *pid, float target, float measured,
                            float dt, float *error)
{
    if (pid == NULL) {
        return 0;
    }
    if (!pid_config_valid(pid) || !pid_is_finite(target) ||
        !pid_is_finite(measured) || !pid_is_finite(dt) || dt <= 0.0f ||
        !pid_is_finite(pid->integral) || !pid_is_finite(pid->output) ||
        !pid_is_finite(pid->derivative)) {
        pid_clear_dynamic(pid);
        return 0;
    }
    *error = target - measured;
    if (!pid_is_finite(*error)) {
        pid_clear_dynamic(pid);
        return 0;
    }
    return 1;
}

float pid_position_update(pid_controller_t *pid, float target, float measured,
                          float dt)
{
    float error;
    float p_term;
    float d_term;
    float candidate_integral;
    float unclamped;
    float output;

    if (!pid_update_valid(pid, target, measured, dt, &error)) {
        return 0.0f;
    }

    p_term = pid->kp * error;

    /* First derivative is defined as zero: no error history yet. */
    if (pid->initialized) {
        d_term = pid->kd * (error - pid->error_1) / dt;
    } else {
        d_term = 0.0f;
    }

    candidate_integral = pid->integral + pid->ki * error * dt;
    if (!pid_is_finite(p_term) || !pid_is_finite(d_term) ||
        !pid_is_finite(candidate_integral)) {
        pid_clear_dynamic(pid); return 0.0f;
    }
    candidate_integral = pid_clamp(candidate_integral,
                                   -pid->integral_limit, pid->integral_limit);

    unclamped = p_term + candidate_integral + d_term;
    if (!pid_is_finite(unclamped)) {pid_clear_dynamic(pid);return 0.0f;}

    /*
     * Conditional antiwindup: if the output would saturate and the current
     * error pushes further into that same saturation, freeze the integral
     * instead of accumulating it.  Errors that drive the output back toward
     * the linear range are always integrated.
     */
    if ((unclamped > pid->output_max && error > 0.0f) ||
        (unclamped < pid->output_min && error < 0.0f)) {
        candidate_integral = pid->integral;
        unclamped = p_term + candidate_integral + d_term;
    }

    pid->integral = candidate_integral;
    output = pid_clamp(unclamped, pid->output_min, pid->output_max);
    pid->output = output;
    pid->derivative = d_term;

    if (pid->initialized) {
        pid->error_2 = pid->error_1;
        pid->error_1 = error;
    } else {
        /* Seed both history slots so the next derivative increment matches
         * the zero derivative of this first sample. */
        pid->error_1 = error;
        pid->error_2 = error;
        pid->initialized = 1;
    }

    return output;
}

float pid_incremental_update(pid_controller_t *pid, float target, float measured,
                             float dt)
{
    float error;
    float p_increment;
    float d_increment;
    float d_term;
    float integral_increment;
    float candidate_integral;
    float delta;
    float candidate_output;
    float output;

    if (!pid_update_valid(pid, target, measured, dt, &error)) {
        return 0.0f;
    }

    /*
     * Incremental form:
     *   delta = Kp*(e - e1) + Ki*e*dt + Kd*(e - 2*e1 + e2)/dt
     *   output = clamp(previous output + delta)
     *
     * While unsaturated this is algebraically the same as the positional
     * form when both start from the same state.  Under saturation the two
     * deliberately differ: positional form recomputes an absolute output
     * from the current error, whereas incremental form starts from its
     * previously clamped output and only adds an increment.  Consequently
     * the incremental controller can keep an offset after a saturated
     * excursion (there is no absolute P term to pull it back), and its
     * conditional antiwindup freezes the stored integral while the clamped
     * output remains at the limit.  This is expected behavior, not a
     * mismatch in the unsaturated regime.
     */
    if (pid->initialized) {
        p_increment = pid->kp * (error - pid->error_1);
        d_term = pid->kd * (error - pid->error_1) / dt;
        d_increment = d_term - pid->derivative;
    } else {
        /* Previous output is zero and the first derivative is zero. */
        p_increment = pid->kp * error;
        d_increment = 0.0f;
        d_term = 0.0f;
    }

    integral_increment = pid->ki * error * dt;
    if (!pid_is_finite(p_increment) || !pid_is_finite(d_increment) ||
        !pid_is_finite(d_term) || !pid_is_finite(integral_increment)) {
        pid_clear_dynamic(pid); return 0.0f;
    }
    candidate_integral = pid_clamp(pid->integral + integral_increment,
                                   -pid->integral_limit, pid->integral_limit);

    /* Use the integral change that actually survives clamping. */
    delta = p_increment + (candidate_integral - pid->integral) + d_increment;
    candidate_output = pid->output + delta;
    if (!pid_is_finite(candidate_output)) {pid_clear_dynamic(pid);return 0.0f;}
    output = pid_clamp(candidate_output, pid->output_min, pid->output_max);

    if ((candidate_output > pid->output_max && error > 0.0f) ||
        (candidate_output < pid->output_min && error < 0.0f)) {
        /* Freeze the integral on further saturation; keep the clamped output. */
        candidate_integral = pid->integral;
    }

    pid->integral = candidate_integral;
    pid->output = output;
    pid->derivative = d_term;

    if (pid->initialized) {
        pid->error_2 = pid->error_1;
        pid->error_1 = error;
    } else {
        pid->error_1 = error;
        pid->error_2 = error;
        pid->initialized = 1;
    }

    return output;
}
