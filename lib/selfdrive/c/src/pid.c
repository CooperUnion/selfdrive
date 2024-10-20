#include <selfdrive/pid.h>

float selfdrive_pid_deadband_compensation(selfdrive_pid_t *pid, float u)
{
	if (!u) return u;

	return (u > 0.0) ? pid->deadband.upper
			+ ((u / pid->limit.upper)
				* (pid->limit.upper - pid->deadband.upper))
			 : pid->deadband.lower
			+ ((u / pid->limit.lower)
				* (pid->limit.lower - pid->deadband.lower));
}

void selfdrive_pid_init(selfdrive_pid_t *pid,
	float				 kp,
	float				 ki,
	float				 kd,
	float				 ts,
	float				 lower_limit,
	float				 upper_limit,
	float				 sigma)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->ts = ts;

	pid->limit.lower = lower_limit;
	pid->limit.upper = upper_limit;

	pid->sigma = sigma;

	pid->private.ts	   = ts;
	pid->private.sigma = sigma;
	pid->private.beta  = ((2 * sigma) - ts) / ((2 * sigma) + ts);
	pid->private.y0	   = 0.0;

	pid->private.error.position = 0.0;
	pid->private.error.velocity = 0.0;

	pid->private.integrate = 0.0;
}

float selfdrive_pid_saturate(selfdrive_pid_t *pid, float u)
{
	float min = (pid->limit.upper < u) ? pid->limit.upper : u;

	return (min >= pid->limit.lower) ? min : pid->limit.lower;
}

void selfdrive_pid_set_deadbands(
	selfdrive_pid_t *pid, float upper, float lower)
{
	pid->deadband.upper = upper;
	pid->deadband.lower = lower;
}

void selfdrive_pid_set_sigma(selfdrive_pid_t *pid, float value)
{
	pid->private.sigma = value;
	pid->private.beta  = (2 * pid->private.sigma - pid->private.ts)
		/ (2 * pid->private.sigma + pid->private.ts);
}

void selfdrive_pid_set_ts(selfdrive_pid_t *pid, float value)
{
	pid->private.ts	  = value;
	pid->private.beta = (2 * pid->private.sigma - pid->private.ts)
		/ (2 * pid->private.sigma + pid->private.ts);
}

void selfdrive_pid_setpoint_reset(
	selfdrive_pid_t *pid, float desired, float current)
{
	pid->private.integrate	    = 0.0;
	pid->private.error.position = desired - current;
	pid->private.error.velocity = 0.0;
}

float selfdrive_pid_step(selfdrive_pid_t *pid, float desired, float current)
{
	float error = desired - current;

	// integrate error using the trapazoidal rule
	pid->private.integrate = pid->private.integrate
		+ ((pid->private.ts / 2)
			* (error + pid->private.error.position));

	// prevent unsaturation of integrator
	if (pid->ki != 0.0)
		pid->private.integrate
			= selfdrive_pid_saturate(
				  pid, pid->ki * pid->private.integrate)
			/ pid->ki;

	// differentiate error
	pid->private.error.velocity
		= pid->private.beta * pid->private.error.velocity;
	pid->private.error.velocity
		+= (((1 - pid->private.beta) / pid->private.ts)
			* (error - pid->private.error.position));

	// PID
	float u_unsat = (pid->kp * error) + (pid->ki * pid->private.integrate)
		+ (pid->kd * pid->private.error.velocity);

	pid->private.error.position = error;
	pid->private.y0		    = current;

	return selfdrive_pid_deadband_compensation(
		pid, selfdrive_pid_saturate(pid, u_unsat));
}
