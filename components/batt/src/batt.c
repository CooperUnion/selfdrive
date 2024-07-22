#include "batt.h"

#include "firmware-base/state-machine.h"
#include <ember_taskglue.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opencan_rx.h>
#include <opencan_tx.h>

#include <math.h>
#include <string.h>

// hypothetical battery percent data
typedef struct {
	uint8_t batteryPercent;
} CAN_Message_BatteryStatus;

static u_int8_t batterypercent;

static void batt_1Hz();

// task scheduling
ember_rate_funcs_S module_rf = {
	.call_1Hz = batt_1Hz,
};

static void batt_1Hz()
{
	/*
	Do I need any authorizationsfrom SUP? like
	CANRX_is_message_battery_percent_data_Authorization_ok()
	*/
	batteryPercent = 0;
}


void CANTX_populate_BATT_BatteryPercentData(
	m->batteryPercent = batteryPercent;)
