#include "brake.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <hal/adc_types.h>

#include "cuber_base.h"
#include "ember_common.h"
#include "ember_taskglue.h"

#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //


#define CMD_MAX 0.6

#define PWM_PIN GPIO_NUM_37

#define PWM_FREQUENCY       1000
#define PWM_INIT_DUTY_CYCLE 0
#define PWM_RESOLUTION      10

#define CMD2DUTY(cmd) ((cmd) * ((1 << PWM_RESOLUTION) - 1))

#define SAMPLING_RATE 20000

// ######      PROTOTYPES       ###### //
static void adc_calibration();

static bool adc_callback(adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *cbs,
                         void * user_data);

// ######     PRIVATE DATA      ###### //
typedef struct adc {
    adc_cali_handle_t       cali_handle;
    adc_continuous_handle_t cont_handle;
} adc_t;


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

static adc_t adc = {
    .cali_handle = NULL,
    .cont_handle = NULL,
};

// ######    RATE FUNCTIONS     ###### //

static void brake_init();
static void brake_100Hz();

ember_rate_funcs_S module_rf = {
    .call_init  = brake_init,
    .call_100Hz = brake_100Hz,
};

static void brake_init()
{
    ledc_timer_config(&pwm_timer);
    ledc_channel_config(&pwm_channel);
    adc_calibration();
}

static void brake_100Hz()
{
    static float32_t prv_cmd;

    bool brake_authorized =
        CANRX_is_message_SUP_Authorization_ok() &&
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
static void adc_calibration() {
        // Setup the calibration driver
        adc_cali_curve_fitting_config_t adc_cali_cfg = {
            .unit_id  = ADC_UNIT_1,     // Using ADC 1
            .atten    = ADC_ATTEN_DB_0, // Not sure which value to pick here
            .bitwidth = ADC_BITWIDTH_9, // Smallest option (trying to fit a lot of data)
        };
        ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&adc_cali_cfg, &adc.cali_handle));

        // Setup the continous driver
        adc_continuous_handle_cfg_t adc_cont_handle_cfg = {
            .max_store_buf_size = 1024, // size of conversion result
            .conv_frame_size    = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * 4, // Conversion frames are 16 bytes
        };
        ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_cont_handle_cfg, &adc.cont_handle));

        adc_digi_pattern_config_t adc_digi_cfg = {
            .atten     = ADC_ATTEN_DB_0,
            .bit_width = ADC_BITWIDTH_9,
            .channel   = ADC_CHANNEL_7,
            .unit      = ADC_UNIT_1,
        };

        adc_continuous_config_t adc_cont_cfg = {
            .pattern_num    = 1, // One channel
            .adc_pattern    = &adc_digi_cfg, // Pass in pre-configured digi config
            .sample_freq_hz = SAMPLING_RATE, // 20kHz
            .conv_mode      = ADC_CONV_SINGLE_UNIT_1, // Only use ADC 1
            .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2, // Only type 2 works on ESP32S3
        };
        ESP_ERROR_CHECK(adc_continuous_config(adc.cont_handle, &adc_cont_cfg));
        // From here on, the ADC should be appropriately configured
        // This is where you would setup the callback feature

        adc_continuous_evt_cbs_t adc_evt_cbs = {
            .on_conv_done = adc_callback,
        };
        ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc.cont_handle, &adc_evt_cbs, NULL));

        ESP_ERROR_CHECK(adc_continuous_start(adc.cont_handle));
}

static bool adc_callback(adc_continuous_handle_t handle,
                         const adc_continuous_evt_data_t *cbs,
                         void * user_data) {
    return false;
}


// ######   PUBLIC FUNCTIONS    ###### //

// ######         CAN TX         ###### //

void CANTX_populate_BRAKE_BrakeData(struct CAN_Message_BRAKE_BrakeData * const m)
{
    m->BRAKE_dutyCycle = pwm_channel.duty;
    m->BRAKE_percent   = (float32_t) pwm_channel.duty / (float32_t) (1 << PWM_RESOLUTION) * 100;
}
