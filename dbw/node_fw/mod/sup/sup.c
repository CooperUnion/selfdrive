//header associated to .c file
//
//system headers
//
//project headers
#include "sup.h"

#include "common.h"
#include "ember_taskglue.h"
#include "opencan_rx.h"
#include "opencan_tx.h"


// ######        DEFINES        ###### //

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //
static bool authorizeBraking;
//we can't have partial situations
//the scheduler for can can prempt 1khz task can come out of nowhere can start happening during 100hz task. cant send data that you are modifying in steps you gotta send it at once
// ######          CAN          ###### //
/*
 *When sending information, you want to use temporary variables to store values until the moment that you want to send the value. For instance, the throttleAuthorized boolean value should only be defined when it is known exactly what that value should be, otherwise, use temporary variables during intermediary steps. This is because the tasking can interrupt or call the process to run at different times, which could cause the wrong value to be sent.
 */

// ######    RATE FUNCTIONS     ###### //
static void sup_100Hz();

ember_rate_funcs_S module_rf = {
    .call_100Hz = sup_100Hz,
};

static void sup_100Hz()
{
    /*
     *  tmp bool, will be or'd with the speed alarm. If the alarm 
     *  is true, authorize breaking will be false, otherwise 
     *  it will be true.
     */

    bool tmp;

    tmp  = false;
    tmp |= CANRX_get_CTRL_speedAlarm();
    authorizeBraking = !tmp;

    //is dbw_velcmd ok?
    //is alarm for speed no raised?
    //if both of those are satisified, give green light for throttle

}

// ######        CAN TX         ###### //


void CANTX_populate_SUP_SupervisorAuthorized(struct CAN_Message_SUP_SupervisorAuthorized * const m)
{
    m->SUP_brakeAuthorized    = authorizeBraking; //
    m->SUP_throttleAuthorized = false;//
    m->SUP_steeringAuthorized = false;//
}
//will be checking bool from rx.h and checking multiple flags
//in tx will be transmitting is throttle ok given the logic that is needed to determine that


