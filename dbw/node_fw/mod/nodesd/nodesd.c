#include <driver/twai.h>

//Are these header files necessary? What is needed for basic canbus implementation?
#include "opencan_rx.h"
#include "opencan_tx.h"

//Purpose of nodesd is to look for DBW active and check to see if each node is working properly
const char* nodesName[7] = {"BRAKE", "ENCF", "ENCR", "PB", "STEER", "TEST", "THROTTLE"};

for(;;)
{
    if(sysStatus = "DBW_Enable")
    {
        twai_message_t message;
        while(sysStatus = "DBW_Enable")
        {
            if(twai_recieve(&message, /* How long of time is needed? */) == ESP_OK)
            {   
                /*
                 *  Questions I have
                 *  1. How are we "recieving" messages, what is the syntax for it?
                 *  EX: The &message stands for what aspect of the message?
                 *  2. How can we specify a component of the message, such as say the name of the node
                 *  associated with that message?
                 *  3. How are we controlling the status of DBW, how can we check what our current status
                 *  is and how can we alter that status?
                 *  4. Why are the files structred the way that they are, what is the significance of lib,
                 *  mod, include, ember, etc. - Followup, where does nodesd fit on that?
                 *  5. Is TWAI relevant, how does it relate to CAN
                 */
            }
            else
            {
                //What logic should be implemented here? !DBW_Active?
            }
        }
    }
}
