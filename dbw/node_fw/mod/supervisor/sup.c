#include "opencan_rx.h"
#include "opencan_tx.h"
#include "ember_taskglue.h"
/*Code for checking to see if Vel_Cmd from Brake node is coming in correctly
 * Create method of checking how long it has been between can messages being recieved
 * - If nothing is recieved, error.
 * - If message is recieved late, error.
 * - If recieved correctly, continue.
 */
static void checkNodeHealth_100Hz();
static void checkNodeMessages_100Hz();
static void outputSupStatus_100Hz();

ember_rate_func_S module_rf = {
    .call_100Hz = checkNodeHealth_100Hz,
    .call_100Hz = checkNodeMEssages_100Hz,
    .call_100Hz = outputSupStatus_100Hz,
};

static void checkNodeHealth_100Hz()
{
    //CAN_RX_is_node_BRAKE_ok() determines if a vel cmd has been recieved
    if(CANRX_is_node_BRAKE_ok()) 
    {

        //send out a can message that says we are ok
    }
    else
    {
        //send a brake estop message on CAN from supervisor.
        sleep(/*time for next can message*/);
    }
    
    if(CANRX_is_node_ENCF_ok()) 
    {

        //send out a can message that says we are ok
    }
    else
    {
        //send a brake estop message on CAN from supervisor.
        sleep(/*time for next can message*/);
    }
}

