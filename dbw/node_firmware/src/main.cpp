#include <iostream>
#include <freertos/FreeRTOS.h>
#include "esp_spi_flash.h"
#include <freertos/task.h>
#include <freertos/queue.h>
#include "tasks.h"
#include "queues.h"
#include <ESP32CAN.h>
#include <CAN_config.h>

//in the header file, we had this as extern, so we needed to make the queue declaration global.
QueueHandle_t canToEStop;
QueueHandle_t canToWatch;
QueueHandle_t canToHouse;
//One single queue for all outgoing tasks, first send to CANTASK and broadcast to watchdog and ROS from that task
QueueHandle_t allTasksToCAN;
//Define queue for urgent messages, for every other task besides CANTASK, needs priority
QueueHandle_t UrgentMsg;

extern "C" void app_main() {
    //for (;;) {
    std::cout << "Howdy world from core " << xPortGetCoreID() << std::endl;
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    fflush(stdout);

    static const char *pcTextForTask1 = "Task LOW is running\r\n";
    static const char *pcTextForTask2 = "Task HIGH is running\r\n";
    TaskHandle_t taskHandle1 = NULL;
    TaskHandle_t taskHandle2 = NULL;
    canToEStop = xQueueCreate(10, sizeof(CAN_frame_t));
    allTasksToCAN = xQueueCreate(20, sizeof(CAN_frame_t));
    UrgentMsg = xQueueCreate(20, sizeof(CAN_frame_t));
    /* Create the two tasks. */
    //xTaskCreatePinnedToCore( Function to be called, name of task, stack size, parameter to pass to function, task priority, task handle, target Core);
    //xTaskCreatePinnedToCore( task_priority, "Low Priority", 1000, (void*)pcTextForTask1, 1, &taskHandle1, 0);
    //xTaskCreatePinnedToCore( task_priority, "High Priority", 1000, (void*)pcTextForTask2, configMAX_PRIORITIES, &taskHandle2, 0);
    xTaskCreatePinnedToCore( cantask, "Cantask", 2000, (void*)pcTextForTask1, 1, &taskHandle1, 0); //cantask
    xTaskCreatePinnedToCore( estoptask, "Estoptask", 2000, NULL, 1, &taskHandle2, 0); //cantask
    /* Start the scheduler so the tasks start executing. */
}
