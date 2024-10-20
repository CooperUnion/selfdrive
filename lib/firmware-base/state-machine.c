#include "state-machine.h"

#include <driver/gpio.h>
#include <ember_common.h>
#include <ember_taskglue.h>
#include <esp_system.h>
#include <node_pins.h>
#include <opencan_rx.h>
#include <opencan_tx.h>
#include <rom/rtc.h>

#include <sdkconfig.h>
#include <watchdog.h>

#if CONFIG_SOC_TEMP_SENSOR_SUPPORTED
#include <driver/temperature_sensor.h>
#endif

#define LED1_PIN NODE_BOARD_PIN_LED1
#define LED2_PIN NODE_BOARD_PIN_LED2

static void set_status_LEDs();
static void base_init();
static void base_10Hz();
static void base_100Hz();

static enum sys_states sys_state       = SYS_STATE_UNDEF;
static enum sys_states requested_state = SYS_STATE_UNDEF;

static bool wdt_trigger;

static RESET_REASON reset_reason;

#if CONFIG_SOC_TEMP_SENSOR_SUPPORTED
static float		    tsens_value;
temperature_sensor_handle_t temp_sensor = NULL;
#endif

ember_rate_funcs_S base_rf = {
	.call_init  = base_init,
	.call_10Hz  = base_10Hz,
	.call_100Hz = base_100Hz,
};

static void base_init()
{
	base_request_state(SYS_STATE_IDLE);

	gpio_config(&(gpio_config_t){
		.pin_bit_mask = BIT64(LED1_PIN) | BIT64(LED2_PIN),
		.mode	      = GPIO_MODE_OUTPUT,
	});

	gpio_set_level(LED1_PIN, 0);
	gpio_set_level(LED2_PIN, 0);


#if CONFIG_SOC_TEMP_SENSOR_SUPPORTED
	temperature_sensor_config_t temp_sensor_config
		= TEMPERATURE_SENSOR_CONFIG_DEFAULT(-30, 50);
	temperature_sensor_install(&temp_sensor_config, &temp_sensor);
	temperature_sensor_enable(temp_sensor);
#endif

	reset_reason = rtc_get_reset_reason(0);
}

static void base_10Hz()
{
	set_status_LEDs();

	/* critical section */
	portDISABLE_INTERRUPTS();
	{
		if (CANRX_is_node_UPD_ok()
			&& CANRX_getRaw_UPD_currentIsoTpChunk() == 0U) {
			if (sys_state != SYS_STATE_DBW_ACTIVE) {
				/* It's update time. No going back; we will
				 * reboot. */

				/*
				 * Set the RTC watchdog timeout to 1 second to
				 * give us some time since the
				 * task_wdt_servicer() is not running anymore.
				 */
				set_up_rtc_watchdog_1sec();

				/* Reboot */
				esp_restart();
			}
		}
	}
	portENABLE_INTERRUPTS();
}

static void base_100Hz()
{
	if (sys_state == SYS_STATE_ESTOP) return;

	if (sys_state == SYS_STATE_IDLE
		&& requested_state == SYS_STATE_DBW_ACTIVE) {
		sys_state = requested_state;
		return;
	}

	sys_state = requested_state;
}

static void set_status_LEDs()
{
	static bool led1_state;
	static bool led2_state;
	static uint timer;

	switch (sys_state) {
		case SYS_STATE_UNDEF:
			if (!(timer % 2)) {
				led1_state = !led1_state;
				led2_state = !led2_state;
			}
			break;

		case SYS_STATE_IDLE:
			led1_state = 1;
			if (!(timer % 1)) {
				led2_state = !led2_state;
			}
			break;

		case SYS_STATE_DBW_ACTIVE:
			led1_state = 1;
			if (!(timer % 2)) {
				led2_state = !led2_state;
			}
			break;

		case SYS_STATE_LOST_CAN:
			led2_state = 1;
			if (!(timer % 2)) {
				led1_state = !led1_state;
			}
			break;

		case SYS_STATE_ESTOP:
			led2_state = 1;
			led1_state = !led1_state;
			break;

		default:
			led1_state = !led1_state;
			led2_state = !led2_state;
			break;
	}

	if (wdt_trigger) {
		led1_state = 1;
		led2_state = 1;
	}

	gpio_set_level(LED1_PIN, led1_state);
	gpio_set_level(LED2_PIN, led2_state);

	timer++;
}

enum sys_states base_get_state(void)
{
	return sys_state;
}

void base_request_state(enum sys_states state)
{
	requested_state = state;
}

void base_set_wdt_trigger(void)
{
	wdt_trigger = true;
}

void ember_can_callback_notify_lost_can(void)
{
	sys_state = SYS_STATE_LOST_CAN;
}

void CANRX_onRxCallback_DBW_ESTOP(
	const struct CAN_MessageRaw_DBW_ESTOP * const raw,
	const struct CAN_Message_DBW_ESTOP * const    dec)
{
	(void) raw;
	(void) dec;

	sys_state = SYS_STATE_ESTOP;
}

void CANTX_populateTemplate_NodeStatus(
	struct CAN_TMessage_DBWNodeStatus * const m)
{
	switch (sys_state) {
		case SYS_STATE_UNDEF:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_UNDEF;
			break;

		case SYS_STATE_INIT:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_INIT;
			break;

		case SYS_STATE_IDLE:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_IDLE;
			break;

		case SYS_STATE_DBW_ACTIVE:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_ACTIVE;
			break;

		case SYS_STATE_LOST_CAN:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_LOST_CAN;
			break;

		case SYS_STATE_BAD:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_BAD;
			break;

		case SYS_STATE_ESTOP:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_ESTOP;
			break;

		default:
			m->sysStatus = CAN_T_DBWNODESTATUS_SYSSTATUS_UNDEF;
			break;
	}

	switch (requested_state) {
		case SYS_STATE_UNDEF:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_UNDEF;
			break;

		case SYS_STATE_INIT:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_INIT;
			break;

		case SYS_STATE_IDLE:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_IDLE;
			break;

		case SYS_STATE_DBW_ACTIVE:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_ACTIVE;
			break;

		case SYS_STATE_LOST_CAN:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_LOST_CAN;
			break;

		case SYS_STATE_BAD:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_BAD;
			break;

		case SYS_STATE_ESTOP:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_ESTOP;
			break;

		default:
			m->requestedSysStatus
				= CAN_T_DBWNODESTATUS_REQUESTEDSYSSTATUS_UNDEF;
			break;
	}

#if CONFIG_SOC_TEMP_SENSOR_SUPPORTED
	if (temperature_sensor_get_celsius(temp_sensor, &tsens_value)
		!= ESP_OK)
		tsens_value = 0.0;
	m->temperature = tsens_value;
#else
	m->temperature = -31.0;
#endif

	static typeof(m->counter) counter;
	m->counter = counter++;

	switch (reset_reason) {
		case POWERON_RESET:
			m->resetReason
				= CAN_T_DBWNODESTATUS_RESETREASON_POWERON;
			break;
		case RTC_SW_CPU_RESET:
			m->resetReason
				= CAN_T_DBWNODESTATUS_RESETREASON_SW_RESET;
			break;
		case RTCWDT_RTC_RESET:
			m->resetReason
				= CAN_T_DBWNODESTATUS_RESETREASON_WATCHDOG_RESET;
			break;
		default:
			m->resetReason
				= CAN_T_DBWNODESTATUS_RESETREASON_UNKNOWN;
			break;
	}

	m->esp32ResetReasonCode = reset_reason;
}

void CANTX_populateTemplate_NodeInfo(struct CAN_TMessage_DBWNodeInfo * const m)
{
	// TODO: reintegrate git
	m->gitHash  = 0;
	m->gitDirty = 0;

	m->eepromIdentity = 0;	// no eeprom identity at the moment
}
