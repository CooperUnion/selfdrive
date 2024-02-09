#include "brake.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <hal/adc_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "cuber_base.h"
#include "ember_common.h"
#include "ember_taskglue.h"

#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //


#define CMD_MAX 0.8

#define PWM_PIN GPIO_NUM_37

#define PWM_FREQUENCY       1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION      10

#define CMD2DUTY(cmd) ((cmd) * ((1 << PWM_RESOLUTION) - 1))

#define TASK_STACK_SIZE 8192

#define SAMPLING_RATE 80000
#define SAMPLE_DUMP_TIME 1.5 // (SECONDS)
#define SAMPLE_DUMP_SIZE 120000

#define PREV_SAMPLE_DELAY 0.010 // (SECONDS)
#define PREV_SAMPLE_SIZE (size_t) (PREV_SAMPLE_DELAY * SAMPLING_RATE)
#define FRAME_SAMPLES 80
#define FRAME_SIZE (sizeof(adc_digi_output_data_t) * FRAME_SAMPLES)
#define POOL_SIZE (FRAME_SIZE * 2)

// ######      PROTOTYPES       ###### //
static void adc_init();

static bool adc_callback(adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *cbs,
                         void * user_data);
static void adc_task();
static void dump_samples();

// ######     PRIVATE DATA      ###### //

static ledc_timer_config_t pwm_timer = {
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num       = LEDC_TIMER_0,
    .freq_hz         = PWM_FREQUENCY,
};

static ledc_channel_config_t pwm_channel = {
    .gpio_num   = PWM_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = PWM_INIT_DUTY_CYCLE,
};

adc_continuous_handle_t handle = NULL;

static SemaphoreHandle_t adc_sem = NULL;

static uint16_t dump_buf[SAMPLE_DUMP_SIZE];
static size_t readIndex;
static size_t writeIndex;


// ######    RATE FUNCTIONS     ###### //

static void brake_init();
static void brake_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = brake_init,
    .call_100Hz = brake_100Hz,
};

static void brake_init()
{
    printf("Brake Init\n");
    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_channel);
    adc_init();
}

static void brake_100Hz()
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
static void adc_init() {
        adc_sem = xSemaphoreCreateCounting(32, 0);
        static TaskHandle_t adc_task_handle;
        xTaskCreatePinnedToCore(adc_task, "adc_task", TASK_STACK_SIZE, 0, 1, &adc_task_handle, 1);


        adc_continuous_handle_cfg_t adc_cont_config = {
            .max_store_buf_size = POOL_SIZE, //Pool size is 2 frames
            .conv_frame_size    = FRAME_SIZE,
        };
        ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_cont_config, &handle));

        adc_digi_pattern_config_t adc_pattern = {
            .atten     = ADC_ATTEN_DB_6,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
            //.channel   = ADC_CHANNEL_7, // GPIO 8
            //.channel   = ADC_CHANNEL_4, // GPIO 5
            .channel   = ADC_CHANNEL_9, // GPIO 10
            .unit      = ADC_UNIT_1,
        };

        adc_continuous_config_t dig_cfg = {
            .sample_freq_hz = SAMPLING_RATE,
            .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
            .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2, // Only type 2 works on ESP32S3
            .pattern_num    = 1,
            .adc_pattern    = &adc_pattern,
        };
        ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

        adc_continuous_evt_cbs_t adc_evt_cbs = {
            .on_conv_done = adc_callback,
        };
        ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &adc_evt_cbs, NULL));

        ESP_ERROR_CHECK(adc_continuous_start(handle));
}

static bool IRAM_ATTR adc_callback(adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *cbs,
                         void *user_data) {
    xSemaphoreGiveFromISR(adc_sem, NULL);
    return true;
}

static IRAM_ATTR void adc_task()
{
loop:
    if (xSemaphoreTake(adc_sem, portMAX_DELAY) != pdTRUE)
        goto loop;

    static uint8_t frame_buf[FRAME_SIZE];
    static uint32_t out_length;

    if (adc_continuous_read(handle, frame_buf, FRAME_SIZE, &out_length, 0) != ESP_OK)
        goto loop;

    static bool latch = 0;
    if (!latch && (base_get_state() == CUBER_SYS_STATE_DBW_ACTIVE))
    {
        readIndex = (writeIndex - PREV_SAMPLE_SIZE) % SAMPLE_DUMP_SIZE;
        latch = 1;
    }
    adc_digi_output_data_t *samples = (adc_digi_output_data_t*) frame_buf;
    size_t sample_count = out_length / sizeof(adc_digi_output_data_t);

    for (size_t i = 0; i < sample_count; i++)
    {
        if (writeIndex == readIndex)
        {
            dump_samples();
            readIndex = SIZE_MAX;
        }
        uint16_t data = samples[i].type2.data;
        dump_buf[writeIndex] = data;
        writeIndex = (writeIndex + 1) % SAMPLE_DUMP_SIZE;
    }

    goto loop;
}

static void dump_samples()
{
    printf("-----------\n");
    for (size_t i = 0; i < SAMPLE_DUMP_SIZE; i++)
    {
        printf("%u,%d\n",i,dump_buf[readIndex]);
        readIndex = (readIndex + 1) % SAMPLE_DUMP_SIZE;
    }

    if (!(i % 10000))
        vTaskDelay(2 / portTICK_PERIOD_MS);
}

// ######   PUBLIC FUNCTIONS    ###### //

// ######         CAN TX         ###### //

void CANTX_populate_BRAKE_BrakeData(struct CAN_Message_BRAKE_BrakeData * const m)
{
    m->BRAKE_dutyCycle = pwm_channel.duty;
    m->BRAKE_percent   = (float32_t) pwm_channel.duty / (float32_t) (1 << PWM_RESOLUTION) * 100;
}
