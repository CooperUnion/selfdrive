#ifndef CANTASK_H
#define CANTASH_H

#include <freertos/queue.h>

void cantask( void *pvParamters);
void checkMessage();
void sendMessage();
void sendToTask(QueueHandle_t TaskQueue);
void checktaskQueues();

#endif

