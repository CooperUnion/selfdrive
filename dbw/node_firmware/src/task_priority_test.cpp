#include <iostream>
#include <freertos/FreeRTOS.h>
#include "esp_spi_flash.h"
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include "task_priority_test.h"

void task_priority(void * pvParameters){
  char *pcTaskName;
  /* The string to print out is passed in via the parameter.  Cast this to a
  character pointer. */
  pcTaskName = (char *) pvParameters;
  int count = 0;
  for (;;) {
    for (int i = 0; i < 1000000; i++)
        if (!(i % 100)) {
            fprintf(stderr, "got to div100\n");
            esp_task_wdt_reset();
        }
    fprintf(stderr, "Task %s ran %d times\n", pcTaskName, count++);
  }
}
