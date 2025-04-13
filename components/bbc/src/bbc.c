#include "bbc.h"

#include "firmware-base/state-machine.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <dsps_biquad.h>
#include <ember_common.h>
#include <ember_taskglue.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <hal/adc_types.h>
#include <opencan_rx.h>
#include <opencan_tx.h>
#include <selfdrive/pid.h>
#include <sys/param.h>

#define DIR_PIN GPIO_NUM_5
#define PWM_PIN GPIO_NUM_4
#define SLP_PIN GPIO_NUM_3
#define FLT_PIN GPIO_NUM_2

#define LIMIT_SWITCH_DEBOUNCE 10

#define PWM_FREQUENCY	    10000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION	    10

#define CMD2DUTY(cmd) ((cmd) * ((1 << PWM_RESOLUTION) - 1))

#define PS_ADC_BITWIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define PS_ADC_CHANNEL ADC_CHANNEL_7  // GPIO_8
#define PS_ADC_UNIT    ADC_UNIT_1

#define LIM_SW_MIN GPIO_NUM_37	// too far backward
#define LIM_SW_MAX GPIO_NUM_40	// too far forward

enum adc_channel_index {
	PS_ADC_CHANNEL_INDEX,
	ADC_CHANNELS,
};

enum limit_switch_state {
	LIMIT_SWITCH_PRESSED,
	LIMIT_SWITCH_RELEASED,
};

#define SAMPLING_RATE_HZ 20000
#define SAMPLES		 20

#define FRAME_SAMPLES 10
#define FRAME_SIZE \
	(sizeof(adc_digi_output_data_t) * FRAME_SAMPLES * ADC_CHANNELS)
#define POOL_SIZE (FRAME_SIZE * 2)

#define ADC_TASK_STACK_SIZE 2048

#define PREV_SAMPLE_DELAY_S 0.010
#define PREV_SAMPLE_SIZE    ((size_t) (PREV_SAMPLE_DELAY_S * SAMPLING_RATE_HZ))

#define KP    5.00
#define KI    0.05
#define KD    0.00
#define SIGMA 1.00
#define TS    0.01

#define PID_LOWER_LIMIT -1
#define PID_UPPER_LIMIT 1

#define PID_DEADBAND_LOWER 0.05
#define PID_DEADBAND_UPPER -0.05

#define MAX_PRESSURE_READING 0.60
#define MIN_PRESSURE_READING 0.06

#define FILTER_LENGTH  2
#define FILTER_BUFFERS 2

// adjustment for SOS gain to match measured values
#define FILTER_FUDGE_FACTOR 1.47

static void  bbc_init(void);
static void  bbc_1kHz(void);
static void  adc_init(void);
static bool  adc_callback(adc_continuous_handle_t handle,
	 const adc_continuous_evt_data_t	 *cbs,
	 void					 *user_data);
static float adc_reading(enum adc_channel_index channel);
static void  adc_task(void *arg);
static void  iir_filter(float *buf);
static bool  overflow_callback(adc_continuous_handle_t handle,
	 const adc_continuous_evt_data_t	      *cbs,
	 void					      *user_data);

struct samples {
	uint16_t raw[SAMPLES];
	float	 filtered[SAMPLES];
	size_t	 index;
};

struct filter {
	float sos[FILTER_LENGTH][5];
	float w[FILTER_LENGTH][2];
	float g;
};

static ledc_channel_config_t pwm_channel = {
	.gpio_num   = PWM_PIN,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.channel    = LEDC_CHANNEL_0,
	.intr_type  = LEDC_INTR_DISABLE,
	.timer_sel  = LEDC_TIMER_0,
	.duty	    = PWM_INIT_DUTY_CYCLE,
};

static struct {
	adc_continuous_handle_t handle;
	SemaphoreHandle_t	sem;
	TaskHandle_t		task_handle;
	struct samples		samples[ADC_CHANNELS];
	struct filter		filter;
	float			filter_buffers[FILTER_BUFFERS][FRAME_SAMPLES];
	size_t			filter_buffer_index;
} adc = {
	.filter		     = {.sos = {{1.0000, 1.0000, 0, -0.9967, 0},
					{1.0000, -1.9998, 1.0000, -1.9977, 0.9978}},
				.w	     = {{0}},
				.g	     = 5.5848e-04 * FILTER_FUDGE_FACTOR},
	.filter_buffer_index = 0
};

static int    motor_direction;
volatile bool overflow;

static float desired_brake;
static float actual_brake;
static float controller_output;

static bool max_limit_switch_status;
static bool min_limit_switch_status;

static selfdrive_pid_t pid;

ember_rate_funcs_S module_rf = {
	.call_init = bbc_init,
	.call_1kHz = bbc_1kHz,
};

static void bbc_init(void)
{
	gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
	motor_direction = 1;
	gpio_set_level(DIR_PIN, !motor_direction);

	gpio_set_direction(LIM_SW_MIN, GPIO_MODE_INPUT);
	gpio_set_direction(LIM_SW_MAX, GPIO_MODE_INPUT);

	gpio_pullup_en(LIM_SW_MIN);
	gpio_pullup_en(LIM_SW_MAX);

	gpio_set_direction(SLP_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(SLP_PIN, 1);

	gpio_set_direction(FLT_PIN, GPIO_MODE_INPUT);
	gpio_pullup_en(FLT_PIN);

	static ledc_timer_config_t pwm_timer = {
		.speed_mode	 = LEDC_LOW_SPEED_MODE,
		.duty_resolution = PWM_RESOLUTION,
		.timer_num	 = LEDC_TIMER_0,
		.freq_hz	 = PWM_FREQUENCY,
	};

	ledc_timer_config(&pwm_timer);
	ledc_channel_config(&pwm_channel);

	adc_init();

	selfdrive_pid_init(
		&pid, KP, KI, KD, TS, PID_LOWER_LIMIT, PID_UPPER_LIMIT, SIGMA);
	selfdrive_pid_set_deadbands(
		&pid, PID_DEADBAND_LOWER, PID_DEADBAND_UPPER);
}

static void bbc_1kHz(void)
{
	static unsigned		       limit_switch_debounce;
	static enum limit_switch_state limit_switch_state;

	limit_switch_debounce = MIN(0, limit_switch_debounce - 1);

	bool bbc_authorized = CANRX_is_message_SUP_Authorization_ok()
		&& CANRX_get_SUP_bbcAuthorized()
		&& CANRX_is_message_CTRL_VelocityCommand_ok();

	bool motor_fault = !gpio_get_level(FLT_PIN);

	float cmd;

	if (base_get_state() != SYS_STATE_DBW_ACTIVE) {
		selfdrive_pid_setpoint_reset(&pid,
			(((float) CANRX_get_CTRL_brakePercent()) / 100.0),
			actual_brake);
	}

	if (motor_fault) {
		gpio_set_level(SLP_PIN, 0);
		cmd = 0.0;
	} else if (bbc_authorized) {
		gpio_set_level(SLP_PIN, 1);
		cmd = ((float) CANRX_get_CTRL_brakePercent()) / 100.0;
		base_request_state(SYS_STATE_DBW_ACTIVE);
	} else {
		gpio_set_level(SLP_PIN, 0);
		cmd = 0.0;
		base_request_state(SYS_STATE_IDLE);
	}

	desired_brake = cmd;

	float brake_reading = adc_reading(PS_ADC_CHANNEL_INDEX);

	brake_reading = MIN(brake_reading, MAX_PRESSURE_READING);
	brake_reading = MAX(brake_reading, MIN_PRESSURE_READING);

	actual_brake = (brake_reading - MIN_PRESSURE_READING)
		/ (MAX_PRESSURE_READING - MIN_PRESSURE_READING);

	controller_output
		= selfdrive_pid_step(&pid, desired_brake, actual_brake);

	motor_direction = (controller_output < 0) ? 0 : 1;

	min_limit_switch_status = gpio_get_level(LIM_SW_MIN);
	max_limit_switch_status = gpio_get_level(LIM_SW_MAX);

	if (!limit_switch_debounce) {
		limit_switch_state
			= (min_limit_switch_status || max_limit_switch_status)
			? LIMIT_SWITCH_PRESSED
			: LIMIT_SWITCH_RELEASED;

		limit_switch_debounce = LIMIT_SWITCH_DEBOUNCE;
	}

	if (limit_switch_state == LIMIT_SWITCH_PRESSED) {
		if (min_limit_switch_status && (controller_output >= 0.0))
			goto skip_disable;

		if (max_limit_switch_status && (controller_output <= 0.0))
			goto skip_disable;

		controller_output = 0.0;
	}

skip_disable:
	gpio_set_level(DIR_PIN, !motor_direction);

	if (controller_output < 0) {
		controller_output *= -1;
	}

	pwm_channel.duty = CMD2DUTY(controller_output);

	ledc_channel_config(&pwm_channel);
}

static void adc_init(void)
{
	overflow = false;
	adc.sem	 = xSemaphoreCreateBinary();

	xTaskCreatePinnedToCore(adc_task,
		"adc_task",
		ADC_TASK_STACK_SIZE,
		0,
		1,
		&adc.task_handle,
		1);

	adc_continuous_handle_cfg_t handle_config = {
		.max_store_buf_size = POOL_SIZE,
		.conv_frame_size    = FRAME_SIZE,
	};
	ESP_ERROR_CHECK(
		adc_continuous_new_handle(&handle_config, &adc.handle));

	adc_digi_pattern_config_t adc_pattern[ADC_CHANNELS] = {
		[PS_ADC_CHANNEL_INDEX] =
		{
		.atten = ADC_ATTEN_DB_12,
		.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
		.channel = PS_ADC_CHANNEL,
		.unit = PS_ADC_UNIT,
		},
	};
	adc_continuous_config_t config = {
		.sample_freq_hz = SAMPLING_RATE_HZ,
		.conv_mode	= ADC_CONV_SINGLE_UNIT_1,
		.format		= ADC_DIGI_OUTPUT_FORMAT_TYPE2,
		.pattern_num	= ADC_CHANNELS,
		.adc_pattern	= adc_pattern,
	};
	ESP_ERROR_CHECK(adc_continuous_config(adc.handle, &config));

	adc_continuous_evt_cbs_t evt_cbs = {
		.on_conv_done = adc_callback,
		.on_pool_ovf  = overflow_callback,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(
		adc.handle, &evt_cbs, NULL));

	ESP_ERROR_CHECK(adc_continuous_start(adc.handle));
}

static bool IRAM_ATTR adc_callback(adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t			  *cbs,
	void						  *user_data)
{
	(void) handle;
	(void) cbs;
	(void) user_data;

	return xSemaphoreGiveFromISR(adc.sem, NULL) == pdTRUE;
}

static float adc_reading(enum adc_channel_index channel)
{
	if ((channel < 0) && (channel >= ADC_CHANNELS)) return 0.0;

	const struct samples * const samples = adc.samples + channel;

	return samples->filtered[(samples->index - 1 + SAMPLES) % SAMPLES];
}

static void adc_task(void *arg)
{
	(void) arg;

loop:
	if (xSemaphoreTake(adc.sem, portMAX_DELAY) != pdTRUE) goto loop;

	if (overflow) {
		base_request_state(SYS_STATE_ESTOP);

		while (1) {
			vTaskDelay(2 / portTICK_PERIOD_MS);
		}
	}

	static uint8_t	buf[FRAME_SIZE];
	static uint32_t size;

	esp_err_t err;

	err = adc_continuous_read(adc.handle, buf, FRAME_SIZE, &size, 0);

	if (err != ESP_OK) goto loop;

	adc_digi_output_data_t *adc_samples = (adc_digi_output_data_t *) buf;
	const size_t sample_count = size / sizeof(adc_digi_output_data_t);

	float *filter_buffer = adc.filter_buffers[adc.filter_buffer_index];
	static uint16_t raw_buffer[FRAME_SAMPLES];

	struct samples *samples = NULL;

	for (size_t i = 0; i < sample_count; i++) {
		adc_digi_output_data_t * const adc_sample = adc_samples + i;

		float resolution;

		switch (adc_sample->type2.channel) {
			case PS_ADC_CHANNEL:
				samples = adc.samples + PS_ADC_CHANNEL_INDEX;
				resolution = 1
					/ ((float) ((1 << PS_ADC_BITWIDTH)
						- 1));
				break;

			default:
				samples = NULL;
				break;
		}

		const uint16_t reading = adc_sample->type2.data;
		raw_buffer[i]	       = reading;

		const float normalized_sample = reading * resolution;
		filter_buffer[i]	      = normalized_sample;
	}

	iir_filter(filter_buffer);

	for (size_t i = 0; i < sample_count; i++) {
		samples->raw[samples->index]	  = raw_buffer[i];
		samples->filtered[samples->index] = filter_buffer[i];
		samples->index = (samples->index + 1) % SAMPLES;
	}

	adc.filter_buffer_index
		= (adc.filter_buffer_index + 1) % FILTER_BUFFERS;

	goto loop;
}

static void iir_filter(float *buf)
{
	float(*sos)[5] = adc.filter.sos;
	const float g  = adc.filter.g;
	float(*w)[2]   = adc.filter.w;

	for (size_t i = 0; i < FILTER_LENGTH; i++) {
		dsps_biquad_f32(buf, buf, FRAME_SAMPLES, sos[i], w[i]);
	}

	for (size_t i = 0; i < FRAME_SAMPLES; i++) {
		buf[i] *= g;
	}
}

static bool IRAM_ATTR overflow_callback(adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t			       *cbs,
	void						       *user_data)
{
	(void) handle;
	(void) cbs;
	(void) user_data;

	overflow = true;

	return true;
}

void CANRX_onRxCallback_DBW_SetBBCGains(
	const struct CAN_TMessageRaw_PIDGains * const raw,
	const struct CAN_TMessage_PIDGains * const    dec)
{
	(void) raw;

	pid.kp = dec->gainKp;
	pid.ki = dec->gainKi;
	pid.kd = dec->gainKd;
}

void CANTX_populateTemplate_BrakeGains(struct CAN_TMessage_PIDGains * const m)
{
	m->gainKp = pid.kp;
	m->gainKi = pid.ki;
	m->gainKd = pid.kd;
}

void CANTX_populate_BBC_BrakeData(struct CAN_Message_BBC_BrakeData * const m)
{
	struct samples * const samples = adc.samples + PS_ADC_CHANNEL_INDEX;

	m->BBC_motorPercent = controller_output * 100;

	m->BBC_pressurePercent = actual_brake * 100;

	m->BBC_adcNormalizedRaw
		= (samples->raw[(samples->index - 1 + SAMPLES) % SAMPLES]
			  / (float) ((1 << PS_ADC_BITWIDTH) - 1))
		* 100;

	m->BBC_adcNormalizedFiltered
		= samples->filtered[(samples->index - 1 + SAMPLES) % SAMPLES]
		* 100;

	m->BBC_limitSwitchMax = max_limit_switch_status;

	m->BBC_limitSwitchMin = min_limit_switch_status;

	m->BBC_motorPercent = (float) pwm_channel.duty
		/ (float) (1 << PWM_RESOLUTION) * 100;
}
