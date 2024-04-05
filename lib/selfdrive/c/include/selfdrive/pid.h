#ifndef SELFDRIVE_PID_H
#define SELFDRIVE_PID_H

typedef struct selfdrive_pid {
	float kp;
	float ki;
	float kd;
	float ts;

	struct {
		float lower;
		float upper;
	} deadband;

	struct {
		float lower;
		float upper;
	} limit;

	float sigma;

	struct {
		float ts;
		float sigma;
		float beta;
		float y0;

		struct {
			float position;
			float velocity;
		} error;

		float integrate;
	} private;
} selfdrive_pid_t;

float selfdrive_pid_deadband_compensation(selfdrive_pid_t *pid, float u);
void  selfdrive_pid_init(selfdrive_pid_t *pid,
     float				  kp,
     float				  ki,
     float				  kd,
     float				  ts,
     float				  lower_limit,
     float				  upper_limit,
     float				  sigma);
float selfdrive_pid_saturate(selfdrive_pid_t *pid, float u);
void  selfdrive_pid_set_deadbands(
     selfdrive_pid_t *pid, float upper, float lower);
void selfdrive_pid_set_sigma(selfdrive_pid_t *pid, float value);
void selfdrive_pid_set_ts(selfdrive_pid_t *pid, float value);
void selfdrive_pid_setpoint_reset(
    selfdrive_pid_t *pid, float desired, float current);
float selfdrive_pid_step(selfdrive_pid_t *pid, float desired, float current);

#endif	// SELFDRIVE_PID_H
