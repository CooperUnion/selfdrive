#include <driver/timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include <stdio.h>

#define TIMER_DIVIDER 80  // Divide Base by 80


// Timer group 0, timer 0 configuration
timer_config_t timer_config = {
	.alarm_en    = TIMER_ALARM_DIS,	     // Disable alarm
	.intr_type   = TIMER_INTR_DISABLE,   // No timer interrupts
	.counter_dir = TIMER_COUNT_UP,	     // Count up
	.auto_reload = TIMER_AUTORELOAD_EN,  // Auto-reload
	.divider = TIMER_DIVIDER  // Set divider for 1ms (assumption that base
				  // runs at 80Mhz)
};

// Initialization
void init_timer0()
{
	timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
	timer_start(TIMER_GROUP_0, TIMER_0);
}

volatile uint64_t core0_idle_cycles = 0;
volatile uint64_t core1_idle_cycles = 0;

// When idle
void vAppliocationIdleHook(void)
{
	// For ISR to handle (?)
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Which core? (assumption 1 cycle == 1 tick)
	// Can ccheck by creating vApplicationTickHook and comparing # tick and
	// # idle cycles << todo

	if (xPortGetCoreID() == 0) {
		core0_idle_cycles++;
	} else if (xPortGetCoreID() == 1) {
		core1_idle_cycles++;
	}
}

void print_cpu_utilization(void *pvParameters)
{
	while (1) {
		uint64_t start_time = 0, end_time = 0;

		timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &start_time);

		// Let run for 5 sec
		// Interrupts can be called here to run other tasks
		// What if interrupt task runs for more than 5 secs?
		vTaskDelay(pdMS_TO_TICKS(5000));

		timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &end_time);

		// Calculate total ticks/cycles in 1 sec
		uint64_t ticks_elapsed = end_time - start_time;

		float core0_utilization = 100.0
			- ((float) core0_idle_cycles / ticks_elapsed) * 100.0;
		float core1_utilization = 100.0
			- ((float) core1_idle cycles / ticks_elapsed) * 100.0;

		// Print CPU utilization
		printf("Core 0 CPU Utilization: %.2f\n", core0_utilization);
		printf("Core 1 CPU Utilization: %.2f\n", core1_utilization);

		// Reset idle cycles
		core0_idle_cycles = 0;
		core1_idle_cycles = 0;

		// Might need to handle overflow if it does occur
	}
}

void app_main()
{
	init_timer0();

	xTaskCreate(print_cpu_utilization,
		"Print CPU Utilization",
		2048,
		NULL,
		1,
		NULL);

	vTaskStartScheduler();
}
