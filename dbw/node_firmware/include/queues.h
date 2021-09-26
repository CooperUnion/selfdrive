#ifndef QUEUES_H
#define QUEUES_H
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern QueueHandle_t canToEStop;
extern QueueHandle_t healthPing;
extern QueueHandle_t modeSet;
extern QueueHandle_t canToHouse;
extern QueueHandle_t canToWatch;
extern QueueHandle_t allTasksToCAN; //All tasks send to this queue for broadcasting messages to CAN
extern QueueHandle_t UrgentMsg; //Queue for emergency messages that is always prioritized

#endif

