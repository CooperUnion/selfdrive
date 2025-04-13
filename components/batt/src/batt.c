#include "batt.h"

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

#include <math.h>

#define ADC_TASK_STACK_SIZE 2048
#define SAMPLING_RATE_HZ    256
#define SAMPLES		    256
#define FRAME_SAMPLES	    128
#define FRAME_SIZE \
	(sizeof(adc_digi_output_data_t) * FRAME_SAMPLES * ADC_CHANNELS)

#define SAMPLES_OUT_SIZE   1024	  // how to decide the size of the output
#define PREV_SAMPLES_DELAY 0.010  // why is this the delay time
#define PREV_SAMPLES_SIZE  ((size_t) (PREV_SAMPLES_DELAY * SAMPLING_RATE_HZ))

#define POOL_SIZE (FRAME_SIZE * 2)

#define ADC_CONV	       ADC_CONV_SINGLE_UNIT_1
#define ADC_DIGI_OUTPUT_FORMAT ADC_DIGI_OUTPUT_FORMAT_TYPE2

#define BV_ADC_ATTEN	ADC_ATTEN_DB_12
#define BV_ADC_UNIT	ADC_UNIT_2
#define BV_ADC_CHANNEL	ADC_CHANNEL_7  // GPIO_8
#define BV_ADC_BITWIDTH SOC_ADC_DIGI_MAX_BITWIDTH

// #define PWM_PIN GPIO_NUM_37
// #define PWM_FREQUENCY 1000
// #define PWM_INIT_DUTY_CYCLE 0
// #define PWM_RESOLUTION 10

adc_continuous_handle_t handle = NULL;
static uint16_t		out_buf[SAMPLES_OUT_SIZE];
static size_t		readIndex;
static size_t		writeIndex;

// static ledc_timer_config_t pwm_timer = {
//     .speed_mode = LEDC_LOW_SPEED_MODE,
//     .duty_resolution = PWM_RESOLUTION,
//     .timer_num = LEDC_TIMER_0,
//     .freq_hz = PWM_FREQUENCY,
// };
//
// static ledc_channel_config_t pwm_channel = {
//     .gpio_num = PWM_PIN,
//     .speed_mode = LEDC_LOW_SPEED_MODE,
//     .channel = LEDC_CHANNEL_0,
//     .intr_type = LEDC_INTR_DISABLE,
//     .timer_sel = LEDC_TIMER_0,
//     .duty = PWM_INIT_DUTY_CYCLE,
// };

enum adc_channel_index {
	BV_ADC_CHANNEL_INDEX,
	ADC_CHANNELS,
};

static uint8_t batteryPercent;
static void    batt_init();
static void    batt_1Hz();
static void    adc_init();

static void adc_task();
static bool adc_callback(adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t		*cbs,
	void					*user_data);

static void samples_out();

struct samples {
	uint16_t raw[SAMPLES];
	// float	 filtered[SAMPLES];
	size_t index;
};

static struct {
	SemaphoreHandle_t	sem;
	adc_continuous_handle_t handle;
	TaskHandle_t		task_handle;
	struct samples		samples[ADC_CHANNELS];
} adc;

ember_rate_funcs_S module_rf = {
	.call_init = batt_init,
	.call_1Hz  = batt_1Hz,
};

static void batt_init()
{
	//    ledc_timer_config(&pwm_timer);
	//   ledc_channel_config(&pwm_channel);
	adc_init();
}

static void batt_1Hz()
{
	printf("hi\n");
	batteryPercent = 0;
}

void CANTX_populate_BATT_BatteryStatus(
	struct CAN_Message_BATT_BatteryStatus * const m)
{
	m->BATT_batteryPercent = batteryPercent;
}

// initialization
static void adc_init()
{
	// overflow = false;
	adc.sem = xSemaphoreCreateBinary();

	// create task
	xTaskCreatePinnedToCore(adc_task,
		"adc_task",
		ADC_TASK_STACK_SIZE,
		NULL,
		1,
		&adc.task_handle,
		1);


	// adc resource allocation
	adc_continuous_handle_cfg_t handle_config = {
		.max_store_buf_size = POOL_SIZE,
		.conv_frame_size    = FRAME_SIZE,
	};

	ESP_ERROR_CHECK(
		adc_continuous_new_handle(&handle_config, &adc.handle));

	// configs for each ADC channel
	adc_digi_pattern_config_t adc_pattern = {
		.atten	   = BV_ADC_ATTEN,
		.bit_width = BV_ADC_BITWIDTH,
		.channel   = BV_ADC_CHANNEL,
		.unit	   = BV_ADC_UNIT,
	};

	// adc IO configs to measure analog signal
	adc_continuous_config_t io_config = {
		.pattern_num	= ADC_CHANNELS,
		.adc_pattern	= &adc_pattern,
		.sample_freq_hz = SAMPLING_RATE_HZ,
		.conv_mode	= ADC_CONV,
		.format		= ADC_DIGI_OUTPUT_FORMAT,
	};

	// invoking callbacks
	adc_continuous_evt_cbs_t evt_cbs = {
		.on_conv_done = adc_callback,
		//.on_pool_ovf = overflow_callback,
	};

	adc_continuous_register_event_callbacks(handle, &evt_cbs, NULL);
	esp_err_t adc_continuous_start(handle);
}

static void adc_task()
{
loop:
	if (xSemaphoreTake(adc.sem, portMAX_DELAY) != pdTRUE) {
		goto loop;
	}

	static uint8_t frame_buf[FRAME_SIZE];


	static uint32_t out_length;

	if (adc_continuous_read(handle, frame_buf, FRAME_SIZE, &out_length, 0)
		!= ESP_OK) {
		goto loop;
	}


	adc_digi_output_data_t *samples;
	size_t samples_count = out_length / sizeof(adc_digi_output_data_t);

	readIndex = (writeIndex - PREV_SAMPLES_SIZE) % SAMPLES_OUT_SIZE;

	for (size_t i = 0; i < samples_count; i++) {
		// write catches up with read index
		if (writeIndex == readIndex) {
			samples_out();
		}
		// uint16_t data = samples[i].type2.data;
		// out_buf[writeIndex] = data;
		writeIndex = (writeIndex + 1) % SAMPLES_OUT_SIZE;
	}

	goto loop;
}

static void samples_out()
{
	printf("DIGI SAMPLES\n");


	static size_t total_samples = 0;
	for (size_t i = 0; i < SAMPLES_OUT_SIZE; i++) {
		printf("%u,%d\n", total_samples + i, out_buf[readIndex]);
		readIndex = (readIndex + 1) % SAMPLES_OUT_SIZE;
	}
	readIndex = SIZE_MAX;
}

// ADC callback
static bool IRAM_ATTR adc_callback(adc_continuous_handle_t handle,
	const adc_continuous_evt_data_t			  *cbs,
	void						  *user_data)
{
	// give back semaphore
	xSemaphoreGiveFromISR(adc.sem, NULL);

	return true;
}

// overflow callback
// static bool ram_attr overflow_callback(adc_continuous_handle_t handle, const
// adc_continuous_evt_data_t *cbs, void *user_data) {
//    overflow = true;
//    return true;
// }
