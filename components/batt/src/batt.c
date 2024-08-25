#include "batt.h"

#include <ember_taskglue.h>
#include <opencan_rx.h>
#include <opencan_tx.h>

#include <math.h>

static uint8_t batteryPercent;

static void batt_1Hz();

ember_rate_funcs_S module_rf = {
	.call_1Hz = batt_1Hz,
};

static void batt_1Hz()
{
	batteryPercent = 0;
}

void CANTX_populate_BatteryStatus(
	struct CAN_Message_BATT_BatteryStatus * const m)
{
	m->BATT_batteryPercent = batteryPercent;
}
