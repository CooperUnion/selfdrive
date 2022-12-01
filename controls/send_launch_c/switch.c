#include "switch.h"

int acc_to_pedal(float acceleration){ //mapping from acc to % throttle
        return 15.4*acceleration;
}
int brake_to_pedal(float brake){ //mapping from brake to % brake
        return (-49.13*brake);
}
int enc_to_velocity(float ticks, float time){     //velocity calculation using encoder ticks 
        int enc_ticks = 4000;
        float wheel_circumference = 1.899156;
        float meters_per_tick = wheel_circumference/enc_ticks;
        return (meters_per_tick * ticks)/time;
}
//created for testinf purposes
BreakOrThrottle newbT(float brake, float throttle) {
    BreakOrThrottle bT;
    bT.brake_percentage=brake;
    bT.throttle_percentage = throttle;
    return bT;
}
BreakOrThrottle brake_or_throttle(float v_actual,float v_des,float acceleration_desired){ //switch mode, returns (throttle, brake)
        BreakOrThrottle val; 
        if(v_actual<0){                                                        
            if (v_actual> -0.5){
                if (acceleration_desired >0 && v_des>0){
                    val.brake_percentage=0;
                    val.throttle_percentage = acc_to_pedal(acceleration_desired);
                    }      //accounts for rolling backwards on an incline when the desired motion is forward
                else{
                    val.brake_percentage=brake_to_pedal(acceleration_desired);   
                    val.throttle_percentage=0;
                }  //accounts for rolling backwards when the desired motion is to stop in order to reverse 
            }
            else{//accounts for a non-float input-- just brakes  
                val.brake_percentage= 50;//cuz there is no reverse yet, so just brake
                val.throttle_percentage=0;
            }
        }
        else if(v_actual>= 0){
            if(acceleration_desired>0){
                val.brake_percentage=0;//accounts for when the car is moving forward and the desired motion is to increase speed
                val.throttle_percentage=acc_to_pedal(acceleration_desired);
            }            
            else if(acceleration_desired<=0){
                val.brake_percentage= brake_to_pedal(acceleration_desired); //accounts for the the car is moving forward and the desired motion is to slow down or stop
                val.throttle_percentage=0;
            }          
        }
        else{
            val.brake_percentage= 50; //accounts for a non-float input -- just brakes
            val.throttle_percentage=0;
        }
    return val; 
}
