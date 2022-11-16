#ifndef Switch_h
#define Switch_h
typedef struct BreakOrThrottle{
    int brake_percentage; 
    int throttle_percentage; 
} BreakOrThrottle; 

int acc_to_pedal(float acceleration);
int enc_to_velocity(float ticks, float time);
int brake_to_pedal(float brake);
BreakOrThrottle brake_or_throttle(float v_actual,float v_des,float acceleration_desired);
#endif