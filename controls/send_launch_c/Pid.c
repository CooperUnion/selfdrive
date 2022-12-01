#include "./Pid.h"
#include <math.h>
#include<stdio.h>

//created for testing
Pid startup(float kp,float ki,float kd,float lower_lim,float upper_lim,float ts,float sigma){
  return newPid(kp,ki,kd,lower_lim, upper_lim, ts, sigma);
}

//PID struct for PID tests
Pid newPid(float kp,float ki,float kd,float lower_lim,float upper_lim,float ts,float sigma) {
    Pid pid; 
    pid.kp         = kp;
    pid.ki         = ki;
    pid.kd         = kd;
    pid.lower_lim  = lower_lim;
    pid.upper_lim  = upper_lim;
    pid._ts        = ts;
    pid._sigma     = sigma;
    pid.beta      = (2 * sigma - ts) / (2 * sigma - ts);
    pid.y0        = 0.0;
    pid.err0      = 0.0;
    pid.err_dot   = 0.0;
    pid._int      = 0.0;
    return pid;
}


float min(float a, float b) {
  if (a < b) return a;
  return b;
}

float max(float a, float b) {
  if (a > b) return a;
  return b;
}

//satutes value in pidupdatecontroller 
float saturate(Pid *pid, float u){
  float sat= max(min(pid->upper_lim, u), pid->lower_lim);
  return sat;
}

void tsSetter(Pid *pid, float val){
  pid->_ts = val; 
  pid->beta = (2 * pid->_sigma - pid->_ts) / (2 * pid->_sigma + pid->_ts);
}

void sigmaSetter(Pid * pid, float val){
  pid->_sigma = val; 
  pid->beta = (2 * pid->_sigma - pid->_ts) / (2 * pid->_sigma + pid->_ts);
} 

float PIDController_Update(Pid *pid, float des, float cur) {

	//Error calc
    float error = des - cur; 
	//Proportional
    float proportional = pid->kp * error;
    //Integral 
    pid->_int +=((pid->_ts/2)*(error+pid->err0));
    //Derivative
    pid->err_dot= pid->beta *pid->err_dot;
    pid->err_dot= (((1-pid->beta)/pid->_ts)*(error - pid->err0));
    
    float val_unsat = (proportional + (pid->ki*pid->_int) + (pid->kd*pid->err_dot));
    float sat=saturate(pid,val_unsat);

    pid->err0 = error;
    pid->y0 = cur;
    

	/* Return controller output */
    return(pid->pidout);

}

void setpointReset(Pid * pid, float des, float cur){
  pid->_int    = 0.0;
  pid->err0    = des - cur;
  pid->err_dot = 0.0;
}

