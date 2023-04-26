#include "pid.h"

void pid_init(pid_S *pid, float kp, float ki, float kd, float ts,
        float upper_lim, float lower_lim, float sigma) {
    // Public Members
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ts = ts;

    pid->limit.upper = upper_lim;
    pid->limit.lower = lower_lim;

    pid->sigma = sigma;

    // Private Members
    pid->private.ts = ts;
    pid->private.sigma = sigma;
    pid->private.beta = (2 * sigma - ts) / (2 * sigma - ts);
    pid->private.y0 = 0.0;

    pid->private.error.position = 0.0;
    pid->private.error.velocity = 0.0;

    pid->private.integrate = 0.0;
}

void set_ts(pid_S *pid, float val) {
    pid->private.ts = val;
    pid->private.beta = (2 * pid->private.sigma - pid->private.ts) /
        (2 * pid->private.sigma + pid->private.ts);
}

void set_sigma(pid_S *pid, float val) {
    pid->private.sigma = val;
    pid->private.beta = (2 * pid->private.sigma - pid->private.ts) /
        (2 * pid->private.sigma + pid->private.ts);
}

float saturate(pid_S *pid, float u) {
    float min = (pid->limit.upper < u) ? pid->limit.upper : u;
    return (min >= pid->limit.lower) ? min : pid->limit.lower;
}

float step(pid_S *pid, float des, float cur) {
    float err = des - cur;

    // integrate error using the trapazoidal rule
    pid->private.integrate =
        pid->private.integrate +
        ((pid->private.ts / 2) * (err + pid->private.error.position));

    // prevent unsaturation of integrator
    if (pid->ki != 0.0) {
        pid->private.integrate =
            saturate(pid, (pid->ki * pid->private.integrate)) / pid->ki;
    }

    // differentiate error
    pid->private.error.velocity = pid->private.beta * pid->private.error.velocity;
    pid->private.error.velocity += (((1 - pid->private.beta) / pid->private.ts) *
            (err - pid->private.error.position));

    // PID
    float u_unsat = (pid->kp * err) + (pid->ki * pid->private.integrate) +
        (pid->kd * pid->private.error.velocity);

    pid->private.error.position = err;
    pid->private.y0 = cur;

    return saturate(pid, u_unsat);
}

void setpoint_reset(pid_S *pid, float des, float cur) {
    pid->private.integrate = 0.0;
    pid->private.error.position = des - cur;
    pid->private.error.velocity = 0.0;
}
