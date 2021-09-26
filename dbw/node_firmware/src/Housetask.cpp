#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Housetask.h"
#include "queues.h"
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <esp_task_wdt.h>

CAN_frame_t h_rx;

/*
void checkCAN() {
  if(xQueueReceive(canToEStop,&h_rx, 0)==pdTRUE){

    printf("New %s frame", (h_rx.FIR.B.FF==CAN_frame_std ? "standard" : "extended"));
    if(h_rx.FIR.B.RTR==CAN_RTR) printf(" RTR");
    printf(" from 0x%08x, DLC %d\r\n",h_rx.MsgID,  h_rx.FIR.B.DLC);
    printf("Housetask received: ");
    for(int i = 0; i < 8; i++) printf("%c", (char)h_rx.data.u8[i]);
    printf("\n");
  }
}*/
void housetask(void *pvParameters){
  for(;;){
    esp_task_wdt_reset();
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
}
