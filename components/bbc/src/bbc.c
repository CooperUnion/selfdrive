#include "bbc.h"

#include "firmware-base/state-machine.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
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

#define ESP_INTR_FLAG_DEFAULT 0

#define DIR_PIN GPIO_NUM_5
#define PWM_PIN GPIO_NUM_4
#define SLP_PIN GPIO_NUM_3
#define FLT_PIN GPIO_NUM_2

#define PWM_FREQUENCY	    10000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION	    10

#define CMD2DUTY(cmd) ((cmd) * ((1 << PWM_RESOLUTION) - 1))

#define PS_ADC_BITWIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define PS_ADC_CHANNEL ADC_CHANNEL_7  // GPIO_8
#define PS_ADC_UNIT    ADC_UNIT_1

#define LIM_SW_1 GPIO_NUM_37  // too far backward
#define LIM_SW_2 GPIO_NUM_40  // too far forward

enum adc_channel_index {
	PS_ADC_CHANNEL_INDEX,
	ADC_CHANNELS,
};

#define SAMPLING_RATE_HZ      20000
#define SAMPLING_TOTAL_FRAMES 120000
#define SAMPLING_BUF_FRAMES   (SAMPLING_TOTAL_FRAMES / ADC_CHANNELS)

#define FRAME_SAMPLES 10
#define FRAME_SIZE \
	(sizeof(adc_digi_output_data_t) * FRAME_SAMPLES * ADC_CHANNELS)
#define POOL_SIZE (FRAME_SIZE * 2)

#define TASK_STACK_SIZE 2048

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

#define MAX_PRESSURE_READING 2600
#define MIN_PRESSURE_READING 155

static void	bbc_init(void);
static void	bbc_1kHz(void);
static void	adc_init(void);
static bool	adc_callback(adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t		    *cbs,
	void					    *user_data);
static uint16_t adc_reading(enum adc_channel_index channel);
static void	adc_task(void *arg);
static void	dump_samples(void);
static bool	overflow_callback(adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t			 *cbs,
	void						 *user_data);

struct dump {
	uint16_t buf[SAMPLING_BUF_FRAMES];
	size_t	 read;
	size_t	 write;
};

static ledc_channel_config_t pwm_channel = {
    .gpio_num	= PWM_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel	= LEDC_CHANNEL_0,
    .intr_type	= LEDC_INTR_DISABLE,
    .timer_sel	= LEDC_TIMER_0,
    .duty	= PWM_INIT_DUTY_CYCLE,
};

static struct {
	adc_continuous_handle_t handle;
	SemaphoreHandle_t	sem;
	TaskHandle_t		task_handle;
	struct dump		dump[ADC_CHANNELS];
} adc;

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
	gpio_set_level(DIR_PIN, motor_direction);

	gpio_set_direction(LIM_SW_1, GPIO_MODE_INPUT);
	gpio_set_direction(LIM_SW_2, GPIO_MODE_INPUT);

	gpio_pullup_en(LIM_SW_1);
	gpio_pullup_en(LIM_SW_2);

	gpio_set_direction(SLP_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(SLP_PIN, 1);

	gpio_set_direction(FLT_PIN, GPIO_MODE_INPUT);
	gpio_pullup_en(FLT_PIN);

	static ledc_timer_config_t pwm_timer = {
	    .speed_mode	     = LEDC_LOW_SPEED_MODE,
	    .duty_resolution = PWM_RESOLUTION,
	    .timer_num	     = LEDC_TIMER_0,
	    .freq_hz	     = PWM_FREQUENCY,
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
	bool bbc_authorized = CANRX_is_message_SUP_Authorization_ok()
	    && CANRX_get_SUP_bbcAuthorized()
	    && CANRX_is_message_CTRL_VelocityCommand_ok();

	bool motor_fault = !gpio_get_level(FLT_PIN);

	float cmd;

	// if not active setpoint reset
	if (base_get_state() != SYS_STATE_DBW_ACTIVE) {
		selfdrive_pid_setpoint_reset(&pid,
		    (((float32_t) CANRX_get_CTRL_brakePercent()) / 100.0),
		    actual_brake);
	}

	if (motor_fault) {
		gpio_set_level(SLP_PIN, 0);
		cmd = 0.0;
	} else if (bbc_authorized) {
		gpio_set_level(SLP_PIN, 1);
		cmd = ((float32_t) CANRX_get_CTRL_brakePercent()) / 100.0;
		base_request_state(SYS_STATE_DBW_ACTIVE);
	} else {
		gpio_set_level(SLP_PIN, 0);
		cmd = 0.0;
		base_request_state(SYS_STATE_IDLE);
	}

	desired_brake = cmd;
	actual_brake  = (((float32_t) adc_reading(PS_ADC_CHANNEL_INDEX))
			    - MIN_PRESSURE_READING)
	    / ((float32_t) (MAX_PRESSURE_READING - MIN_PRESSURE_READING));

	actual_brake = (((actual_brake) >= (1.0)) ? (1.0) : (actual_brake));

	controller_output
	    = selfdrive_pid_step(&pid, desired_brake, actual_brake);

	motor_direction = (controller_output < 0) ? 0 : 1;

	max_limit_switch_status = gpio_get_level(LIM_SW_2);
	min_limit_switch_status = gpio_get_level(LIM_SW_1);

	if (motor_direction) {
		if (gpio_get_level(LIM_SW_2)) {
			gpio_set_level(SLP_PIN, 0);
			controller_output = 0.0;
		}
	} else {
		if (gpio_get_level(LIM_SW_1)) {
			gpio_set_level(SLP_PIN, 0);
			controller_output = 0.0;
		}
	}

	gpio_set_level(DIR_PIN, motor_direction);

	if (controller_output < 0) {
		controller_output *= -1;
	}

	pwm_channel.duty = CMD2DUTY(controller_output);

	ledc_channel_config(&pwm_channel);
}

static void adc_init(void)
{
	overflow = false;
	adc.sem	 = xSemaphoreCreateCounting(ADC_CHANNELS, 0);

	xTaskCreatePinnedToCore(
	    adc_task, "adc_task", TASK_STACK_SIZE, 0, 1, &adc.task_handle, 1);

	for (size_t i = 0; i < ADC_CHANNELS; i++) {
		struct dump * const dump = adc.dump + i;

		dump->read  = SIZE_MAX;
		dump->write = 0;
	}

	adc_continuous_handle_cfg_t handle_config = {
	    .max_store_buf_size = POOL_SIZE,
	    .conv_frame_size	= FRAME_SIZE,
	};
	ESP_ERROR_CHECK(
	    adc_continuous_new_handle(&handle_config, &adc.handle));

	adc_digi_pattern_config_t adc_pattern[ADC_CHANNELS] = {
		[PS_ADC_CHANNEL_INDEX] = {
			.atten     = ADC_ATTEN_DB_12,
			.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
			.channel   = PS_ADC_CHANNEL,
			.unit      = PS_ADC_UNIT,
		},
	};
	adc_continuous_config_t config = {
	    .sample_freq_hz = SAMPLING_RATE_HZ,
	    .conv_mode	    = ADC_CONV_SINGLE_UNIT_1,
	    .format	    = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
	    .pattern_num    = ADC_CHANNELS,
	    .adc_pattern    = adc_pattern,
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

static uint16_t adc_reading(enum adc_channel_index channel)
{
	if ((channel < 0) && (channel >= ADC_CHANNELS)) return UINT16_MAX;

	const struct dump * const dump = adc.dump + channel;

	return dump->buf[(dump->write - 1 + SAMPLING_BUF_FRAMES)
	    % SAMPLING_BUF_FRAMES];
}

static void adc_task(void *arg)
{
	(void) arg;

loop:
	if (xSemaphoreTake(adc.sem, portMAX_DELAY) != pdTRUE) goto loop;

	if (overflow) {
		while (1) {
			vTaskDelay(2 / portTICK_PERIOD_MS);
		}
	}

	static uint8_t	buf[FRAME_SIZE];
	static uint32_t size;

	esp_err_t err;

	err = adc_continuous_read(adc.handle, buf, FRAME_SIZE, &size, 0);

	if (err != ESP_OK) goto loop;

	// static bool latch = false;

	// if (!latch && (base_get_state() == SYS_STATE_DBW_ACTIVE))
	// {
	// 	for (size_t i = 0; i < ADC_CHANNELS; i++) {
	// 		struct dump * const dump = adc.dump + i;

	// 		dump->read
	// 			= (dump->write - PREV_SAMPLE_SIZE)
	// 			% SAMPLING_BUF_FRAMES;
	// 	}

	// 	latch = true;
	// }

	adc_digi_output_data_t *samples = (adc_digi_output_data_t *) buf;
	const size_t sample_count = size / sizeof(adc_digi_output_data_t);

	bool start_dump = true;
	for (size_t i = 0; i < sample_count; i++) {
		adc_digi_output_data_t * const sample = samples + i;

		struct dump *dump;

		switch (sample->type2.channel) {
			case PS_ADC_CHANNEL:
				dump = adc.dump + PS_ADC_CHANNEL_INDEX;
				break;

			default:
				dump = NULL;
				break;
		}

		if (dump->read == dump->write) continue;

		start_dump = false;

		uint16_t raw_sample = sample->type2.data;

		dump->buf[dump->write] = raw_sample;
		dump->write = (dump->write + 1) % SAMPLING_BUF_FRAMES;
	}

	if (start_dump) dump_samples();

	goto loop;
}

static void dump_samples()
{
	for (size_t i = 0; i < ADC_CHANNELS; i++) {
		printf("--- unfiltered %d ---\n", i);

		struct dump * const dump = adc.dump + i;

		size_t read = dump->read;

		for (size_t j = 0; j < SAMPLING_BUF_FRAMES; j++) {
			printf("%u,%d\n", j, dump->buf[read]);
			read = (read + 1) % SAMPLING_BUF_FRAMES;

			// prevent task_wdt trips
			if (!(j % 10000)) vTaskDelay(2 / portTICK_PERIOD_MS);
		}

		dump->read = SIZE_MAX;
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
    const struct CAN_TMessage_PIDGains * const	  dec)
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
	m->BBC_motorPercent = controller_output * 100;

	m->BBC_pressurePercent = actual_brake * 100;

	m->BBC_limitSwitchMax = max_limit_switch_status;

	m->BBC_limitSwitchMin = min_limit_switch_status;
}
