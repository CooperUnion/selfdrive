#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include "Estoptask.h"
#include "queues.h"
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <esp_task_wdt.h>
#include "MessageID.h"
#include "Nodes.h"

CAN_frame_t es_rx_frame, es_tx_frame;

void emergencyStop(){
  //Send an emergency stop message to Cantask for broadcasting
  es_tx_frame.MsgID = canmsg_ID::MSG_ESTOP;
  strcpy((char*)es_tx_frame.data.u8,"EMERSTO");
  es_tx_frame.data.u8[7] = 'P';
  xQueueSend(UrgentMsg, &es_tx_frame, portMAX_DELAY);
  printf("EMERGENCY STOP\n");
}

void checkCAN() {
  if(xQueueReceive(canToEStop,&es_rx_frame, 0)==pdTRUE){

    printf("New %s frame", (es_rx_frame.FIR.B.FF==CAN_frame_std ? "standard" : "extended"));
    if(es_rx_frame.FIR.B.RTR==CAN_RTR) printf(" RTR");
    printf(" from 0x%08x, DLC %d\r\n",es_rx_frame.MsgID,  es_rx_frame.FIR.B.DLC);
    printf("Estoptask received: ");
    for(int i = 0; i < 8; i++) printf("%c", (char)es_rx_frame.data.u8[i]);
    printf("\n");
    //selects task based on message ID
    switch (es_rx_frame.MsgID){
      case canmsg_ID::MSG_ESTOP:
        emergencyStop();
        break;
      default:
        break;
    }
  }
}
void estoptask(void *pvParameters){
  es_tx_frame.FIR.B.FF = CAN_frame_ext;
  es_tx_frame.FIR.B.DLC = 8;
  for(;;){
    checkCAN();
    esp_task_wdt_reset();
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
}
