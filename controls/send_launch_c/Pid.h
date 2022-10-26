#ifndef Pid_h
#define Pid_h

typedef struct Pid{
    float kp;
    float ki;
    float kd;
    
    float _ts;
    float upper_lim;
    float lower_lim;

    float _sigma;
    float beta;
    float y0;

    float err0;
    float err_dot;
    float _int;

    float pidout;
} Pid;


Pid newPid(float kp,float ki,float kd,float ts,float lower_lim,float upper_lim,float sigma);

float PIDController_Update(Pid *self, float des, float cur);

float tsSetter(Pid * pid, float val); 

float sigmaSetter(Pid * pid, float val); 

void setpointReset(Pid * pid, float des, float cur); 

#endif
