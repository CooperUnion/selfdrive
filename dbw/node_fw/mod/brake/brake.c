#include "brake.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <hal/adc_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "cuber_base.h"
#include "ember_common.h"
#include "ember_taskglue.h"

#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

#define CMD_MAX 1

#define DIR_PIN GPIO_NUM_1
#define PWM_PIN GPIO_NUM_2

#define PWM_FREQUENCY       1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION      10

#define CMD2DUTY(cmd) ((cmd) * ((1 << PWM_RESOLUTION) - 1))

#define CS_ADC_CHANNEL ADC_CHANNEL_4
#define PS_ADC_CHANNEL ADC_CHANNEL_8

enum {
	CS_ADC_CHANNEL_INDEX,
	PS_ADC_CHANNEL_INDEX,
	ADC_CHANNELS,
};

#define SAMPLING_RATE_KHZ     80000
#define SAMPLING_TOTAL_FRAMES 120000
#define SAMPLING_BUF_FRAMES   (SAMPLING_TOTAL_FRAMES / ADC_CHANNELS)

#define FRAME_SAMPLES 80
#define FRAME_SIZE    (sizeof(adc_digi_output_data_t) * FRAME_SAMPLES * ADC_CHANNELS)
#define POOL_SIZE     (FRAME_SIZE * 2)

#define TASK_STACK_SIZE 8192

#define PREV_SAMPLE_DELAY_S 0.010
#define PREV_SAMPLE_SIZE    ((size_t) (PREV_SAMPLE_DELAY_S * SAMPLING_RATE_KHZ))

// ######      PROTOTYPES       ###### //

static void adc_init(void);
static bool adc_callback(
	adc_continuous_handle_t         handle,
	const adc_continuous_evt_data_t *cbs,
	void                            *user_data);
static void adc_task(void *arg);
static void dump_samples(void);

// ######     PRIVATE DATA      ###### //

static ledc_channel_config_t pwm_channel = {
	.gpio_num   = PWM_PIN,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.channel    = LEDC_CHANNEL_0,
	.intr_type  = LEDC_INTR_DISABLE,
	.timer_sel  = LEDC_TIMER_0,
	.duty       = PWM_INIT_DUTY_CYCLE,
};

struct dump {
	uint16_t buf[SAMPLING_BUF_FRAMES];
	size_t   read;
	size_t   write;
};

static struct {
	adc_continuous_handle_t handle;
	SemaphoreHandle_t       sem;
	TaskHandle_t            task_handle;
	struct dump             dump[ADC_CHANNELS];
} adc;


// ######    RATE FUNCTIONS     ###### //

static void brake_init(void);
static void brake_100Hz(void);

ember_rate_funcs_S module_rf = {
	.call_init  = brake_init,
	.call_100Hz = brake_100Hz,
};

static void brake_init(void)
{
	gpio_set_direction(DIR_PIN, GPIO_MODE_INPUT);
	gpio_set_level(DIR_PIN, 1);

	static ledc_timer_config_t pwm_timer = {
		.speed_mode      = LEDC_LOW_SPEED_MODE,
		.duty_resolution = PWM_RESOLUTION,
		.timer_num       = LEDC_TIMER_0,
		.freq_hz         = PWM_FREQUENCY,
	};

	ledc_timer_config(&pwm_timer);
	ledc_channel_config(&pwm_channel);

	adc_init();
}

static void brake_100Hz(void)
{
	static float32_t prv_cmd;

	bool brake_authorized = CANRX_is_message_SUP_Authorization_ok() &&
		CANRX_get_SUP_brakeAuthorized() &&
		CANRX_is_message_CTRL_VelocityCommand_ok();

	float cmd;

	if (brake_authorized) {
		cmd = ((float32_t) CANRX_get_CTRL_brakePercent()) / 100.0;
		base_request_state(CUBER_SYS_STATE_DBW_ACTIVE);
	} else {
		cmd = 0.0;
		base_request_state(CUBER_SYS_STATE_IDLE);
	}

	if (cmd > CMD_MAX) cmd = CMD_MAX;

	// low-pass filter
	const float32_t alpha = 0.1;
	cmd = prv_cmd + (alpha * (cmd - prv_cmd));

	prv_cmd = cmd;

	pwm_channel.duty = CMD2DUTY(cmd);
	ledc_channel_config(&pwm_channel);
}

// ######   PRIVATE FUNCTIONS   ###### //

static void adc_init(void)
{
	adc.sem = xSemaphoreCreateCounting(ADC_CHANNELS, 0);

	xTaskCreatePinnedToCore(
		adc_task,
		"adc_task",
		TASK_STACK_SIZE,
		0,
		1,
		&adc.task_handle,
		1);

	for (size_t i = 0; i < ADC_CHANNELS; i++) {
		struct dump * const dump = adc.dump + i;

		dump->read  = SIZE_MAX;
		dump->write = 0;
	}

	adc_continuous_handle_cfg_t handle_config = {
		.max_store_buf_size = POOL_SIZE,
		.conv_frame_size    = FRAME_SIZE,
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(
		&handle_config,
		&adc.handle));

	adc_digi_pattern_config_t adc_pattern[ADC_CHANNELS] = {
		[CS_ADC_CHANNEL_INDEX] = {
			.atten     = ADC_ATTEN_DB_6,
			.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
			.channel   = CS_ADC_CHANNEL,
			.unit      = ADC_UNIT_1,
		},
		[PS_ADC_CHANNEL_INDEX] = {
			.atten     = ADC_ATTEN_DB_6,
			.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
			.channel   = PS_ADC_CHANNEL,
			.unit      = ADC_UNIT_1,
		},
	};
	adc_continuous_config_t config = {
		.sample_freq_hz = SAMPLING_RATE_KHZ,
		.conv_mode      = ADC_CONV_SINGLE_UNIT_1,
		.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
		.pattern_num    = ADC_CHANNELS,
		.adc_pattern    = adc_pattern,
	};
	ESP_ERROR_CHECK(adc_continuous_config(adc.handle, &config));

	adc_continuous_evt_cbs_t evt_cbs = {
		.on_conv_done = adc_callback,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(
		adc.handle,
		&evt_cbs,
		NULL));

	ESP_ERROR_CHECK(adc_continuous_start(adc.handle));
}

static bool IRAM_ATTR adc_callback(
	adc_continuous_handle_t         handle,
	const adc_continuous_evt_data_t *cbs,
	void                            *user_data)
{
	return xSemaphoreGiveFromISR(adc.sem, NULL) == pdTRUE;
}

static void adc_task(void *arg)
{
	(void) arg;

loop:
	if (xSemaphoreTake(adc.sem, portMAX_DELAY) != pdTRUE)
	goto loop;

	static uint8_t  buf[FRAME_SIZE];
	static uint32_t size;

	esp_err_t err;

	err = adc_continuous_read(adc.handle, buf, FRAME_SIZE, &size, 0);

	if (err != ESP_OK)
		goto loop;

	static bool latch = false;

	if (!latch && (base_get_state() == CUBER_SYS_STATE_DBW_ACTIVE))
	{
		for (size_t i = 0; i < ADC_CHANNELS; i++) {
			struct dump * const dump = adc.dump + i;

			dump->read
				= (dump->write - PREV_SAMPLE_SIZE)
				% SAMPLING_BUF_FRAMES;
		}

		latch = true;
	}

	const adc_digi_output_data_t *samples = (adc_digi_output_data_t*) buf;
	const size_t sample_count = size / sizeof(adc_digi_output_data_t);

	bool start_dump = true;
	for (size_t i = 0; i < sample_count; i++)
	{
		const adc_digi_output_data_t * const sample = samples + i;

		struct dump *dump;
		switch (sample->type2.channel) {
			case CS_ADC_CHANNEL:
				dump = adc.dump + CS_ADC_CHANNEL_INDEX;
				break;

			case PS_ADC_CHANNEL:
				dump = adc.dump + PS_ADC_CHANNEL_INDEX;
				break;

			default:
				dump = NULL;
				break;
		}

		if (dump->read == dump->write)
			continue;

		start_dump = false;

		dump->buf[dump->write] = sample->type2.data;
		dump->write = (dump->write + 1) % SAMPLING_BUF_FRAMES;
	}

	if (start_dump)
		dump_samples();

	goto loop;
}

static void dump_samples()
{
	for (size_t i = 0; i < ADC_CHANNELS; i++) {
		printf("--- %d ---\n", i);

		struct dump * const dump = adc.dump + i;

		uint16_t read  = dump->read;

		for (size_t j = 0; j < SAMPLING_BUF_FRAMES; j++) {
			printf("%u,%d\n", j, dump->buf[read]);
			read = (read + 1) % SAMPLING_BUF_FRAMES;

			// prevent task_wdt trips
			if (!(j % 10000))
				vTaskDelay(2 / portTICK_PERIOD_MS);
		}

		dump->read = SIZE_MAX;
	}
}

// ######   PUBLIC FUNCTIONS    ###### //

// ######         CAN TX         ###### //

void CANTX_populate_BRAKE_BrakeData(struct CAN_Message_BRAKE_BrakeData * const m)
{
	m->BRAKE_dutyCycle = pwm_channel.duty;

	m->BRAKE_percent
		= (float32_t) pwm_channel.duty
		/ ((float32_t) (1 << PWM_RESOLUTION) * 100);
}
