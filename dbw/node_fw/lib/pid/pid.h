#ifndef PID_H
#define PID_H

typedef struct {
    // Public Members
    float kp;
    float ki;
    float kd;
    float ts;
    struct {
        float upper;
        float lower;
    } limit;
    float sigma;

    // Private Members
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

void pid_init(pid_S *pid, float kp, float ki, float kd, float ts,
        float upper_lim, float lower_lim, float sigma);

void set_ts(pid_S *pid, float val);

void set_sigma(pid_S *pid, float val);

void setpoint_reset(pid_S *pid, float des, float cur);

float saturate(pid_S *pid, float u);

float step(pid_S *pid, float des, float cur);

#endif