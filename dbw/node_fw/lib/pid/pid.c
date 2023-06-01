#include "pid.h"

void pid_init(
    pid_S *pid,
    float  kp,
    float  ki,
    float  kd,
    float  ts,
    float  upper_limit,
    float  lower_limit,
    float  sigma)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ts = ts;

    pid->limit.upper = upper_limit;
    pid->limit.lower = lower_limit;

    pid->sigma = sigma;

    pid->private.ts    = ts;
    pid->private.sigma = sigma;
    pid->private.beta  = (2 * sigma - ts) / (2 * sigma - ts);
    pid->private.y0    = 0.0;

    pid->private.error.position = 0.0;
    pid->private.error.velocity = 0.0;

    pid->private.integrate = 0.0;
}

float pid_saturate(pid_S *pid, float u)
{
    float min = (pid->limit.upper < u) ? pid->limit.upper : u;

    return (min >= pid->limit.lower) ? min : pid->limit.lower;
}

void pid_set_sigma(pid_S *pid, float value)
{
    pid->private.sigma = value;
    pid->private.beta
        = (2 * pid->private.sigma - pid->private.ts)
        / (2 * pid->private.sigma + pid->private.ts);
}

void pid_set_ts(pid_S *pid, float value)
{
    pid->private.ts = value;
    pid->private.beta
    	= (2 * pid->private.sigma - pid->private.ts)
	/ (2 * pid->private.sigma + pid->private.ts);
}

void pid_setpoint_reset(pid_S *pid, float desired, float current)
{
    pid->private.integrate      = 0.0;
    pid->private.error.position = desired - current;
    pid->private.error.velocity = 0.0;
}

float pid_step(pid_S *pid, float desired, float current)
{
    float error = desired - current;

    // integrate error using the trapazoidal rule
    pid->private.integrate
        = pid->private.integrate
        + ((pid->private.ts / 2) * (error + pid->private.error.position));

    // prevent unsaturation of integrator
    if (pid->ki != 0.0)
        pid->private.integrate
            = pid_saturate(pid, pid->ki * pid->private.integrate)
            / pid->ki;

    // differentiate error
    pid->private.error.velocity = pid->private.beta * pid->private.error.velocity;
    pid->private.error.velocity
        += (((1 - pid->private.beta) / pid->private.ts)
        * (error - pid->private.error.position));

    // PID
    float u_unsat
        = (pid->kp * error)
        + (pid->ki * pid->private.integrate)
        + (pid->kd * pid->private.error.velocity);

    pid->private.error.position = error;
    pid->private.y0             = current;

    return pid_saturate(pid, u_unsat);
}
