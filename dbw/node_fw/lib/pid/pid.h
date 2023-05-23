#ifndef PID_H
#define PID_H

typedef struct pid {
    float kp;
    float ki;
    float kd;
    float ts;
    struct {
        float upper;
        float lower;
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
} pid_S;

void pid_init(
    pid_S *pid,
    float  kp,
    float  ki,
    float  kd,
    float  ts,
    float  upper_limit,
    float  lower_limit,
    float  sigma);
float pid_saturate(pid_S *pid, float u);
void pid_set_sigma(pid_S *pid, float value);
void pid_set_ts(pid_S *pid, float value);
void pid_setpoint_reset(pid_S *pid, float desired, float current);
float pid_step(pid_S *pid, float desired, float current);

#endif
