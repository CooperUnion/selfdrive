#include "opencan_rx.h"
#include "opencan_tx.h"
#include <unistd.h>        
    

/*Code for checking to see if Vel_Cmd from Brake node is coming in correctly
 * Create method of checking how long it has been between can messages being recieved
 * - If nothing is recieved, error.
 * - If message is recieved late, error.
 * - If recieved correctly, continue.
 */

while(true)
{
    //CAN_RX_is_node_BRAKE_ok() determines if a vel cmd has been recieved
    if(CANRX_is_node_BRAKE_ok()) 
    {

        //send out a can message that says we are ok
        sleep(/*time for next can message */);
    }
    else
    {
        //send a brake estop message on CAN from supervisor.
        sleep(/*time for next can message*/);
    }
}

