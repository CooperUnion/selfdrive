#include <iostream>
#include <freertos/FreeRTOS.h>
#include "esp_spi_flash.h"
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include "task_print_task_name.h"

void sampleTask(void * pvParameters){
  char *pcTaskName;
  /* The string to print out is passed in via the parameter.  Cast this to a
  character pointer. */
  pcTaskName = (char *) pvParameters;
  /* As per most tasks, this task is implemented in an infinite loop. */
  for(;;){
    /* Print out the name of this task. */
    printf("%s\n", pcTaskName);
    /* Delay for a period. */
    esp_task_wdt_reset();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

