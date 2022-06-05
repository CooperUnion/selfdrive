#include "encoder.h"

#include <driver/gpio.h>
#include <driver/timer.h>

#include "base/base.h"
#include "common.h"
#include "io/can.h"
#include "module_types.h"
#include "sys/task_glue.h"

/* Define firmware module identity for the entire build. */
const enum firmware_module_types FIRMWARE_MODULE_IDENTITY = MOD_ENCODER;

// ######        DEFINES        ###### //

#define ENCODER0_CHAN_A 25
#define ENCODER0_CHAN_B 27
#define ENCODER1_CHAN_A 34
#define ENCODER1_CHAN_B 35

#define ESP_INTR_FLAG_DEFAULT 0

#define ENCODER_MAX_TICKS  85     // slightly over 5MPH
#define ENCODER_TIMEOUT_US 20000

// ######      PROTOTYPES       ###### //

static void IRAM_ATTR encoder0_chan_a(void *arg);
static void IRAM_ATTR encoder0_chan_b(void *arg);
static void IRAM_ATTR encoder1_chan_a(void *arg);
static void IRAM_ATTR encoder1_chan_b(void *arg);

// ######     PRIVATE DATA      ###### //

static volatile int64_t pulse_cnt[2];
static int64_t prv_pulse_cnt[2];

// ######          CAN          ###### //

static struct CAN_dbwNode_Encoder_Data_t CAN_Encoder;

static const can_outgoing_t can_Encoder_Data_cfg = {
    .id = CAN_DBWNODE_ENCODER_DATA_FRAME_ID,
    .extd = CAN_DBWNODE_ENCODER_DATA_IS_EXTENDED,
    .dlc = CAN_DBWNODE_ENCODER_DATA_LENGTH,
    .pack = CAN_dbwNode_Encoder_Data_pack,
};

// ######    RATE FUNCTIONS     ###### //

static void encoder_init();
static void encoder_100Hz();

const struct rate_funcs module_rf = {
    .call_init  = encoder_init,
    .call_100Hz = encoder_100Hz,
};

static void encoder_init()
{
    const timer_config_t timer_config = {
        .alarm_en    = TIMER_ALARM_DIS,
        .counter_en  = TIMER_PAUSE,
        .intr_type   = TIMER_INTR_NONE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider     = 80,  // 1E-6 * 80MHz (microseconds)
    };
    timer_init(TIMER_GROUP_1, TIMER_0, &timer_config);

    gpio_set_direction(ENCODER0_CHAN_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER0_CHAN_B, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER1_CHAN_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER1_CHAN_B, GPIO_MODE_INPUT);

    gpio_set_intr_type(ENCODER0_CHAN_A, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER0_CHAN_B, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER1_CHAN_A, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(ENCODER1_CHAN_B, GPIO_INTR_ANYEDGE);

    gpio_pullup_en(ENCODER0_CHAN_A);
    gpio_pullup_en(ENCODER0_CHAN_B);
    gpio_pullup_en(ENCODER1_CHAN_A);
    gpio_pullup_en(ENCODER1_CHAN_B);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    gpio_isr_handler_add(ENCODER0_CHAN_A, encoder0_chan_a, NULL);
    gpio_isr_handler_add(ENCODER0_CHAN_B, encoder0_chan_b, NULL);
    gpio_isr_handler_add(ENCODER1_CHAN_A, encoder1_chan_a, NULL);
    gpio_isr_handler_add(ENCODER1_CHAN_B, encoder1_chan_b, NULL);

    timer_start(TIMER_GROUP_1, TIMER_0);
}

static void encoder_100Hz()
{
    static uint64_t timer_val;
    static uint64_t prv_timer_val;

    timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &timer_val);

    CAN_Encoder.Encoder0 = pulse_cnt[0] - prv_pulse_cnt[0];
    CAN_Encoder.Encoder1 = pulse_cnt[1] - prv_pulse_cnt[1];
    CAN_Encoder.Time     = timer_val - prv_timer_val;

    can_send_iface(&can_Encoder_Data_cfg, &CAN_Encoder);

    if (
        (ABS(CAN_Encoder.Encoder0) >= ENCODER_MAX_TICKS) ||
        (ABS(CAN_Encoder.Encoder1) >= ENCODER_MAX_TICKS)
    )
        base_set_state_estop(CAN_dbwESTOP_Reason_LIMIT_EXCEEDED_CHOICE);

    if (CAN_Encoder.Time >= ENCODER_TIMEOUT_US)
        base_set_state_estop(CAN_dbwESTOP_Reason_TIMEOUT_CHOICE);

    prv_pulse_cnt[0] += CAN_Encoder.Encoder0;
    prv_pulse_cnt[1] += CAN_Encoder.Encoder1;
    prv_timer_val    += CAN_Encoder.Time;
}

// ######   PRIVATE FUNCTIONS   ###### //

static void IRAM_ATTR encoder0_chan_a(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER0_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER0_CHAN_B);

    if (chan_a) {
        if (chan_b) {
            --pulse_cnt[0];
        } else {
            ++pulse_cnt[0];
        }
    } else {
        if (chan_b) {
            ++pulse_cnt[0];
        } else {
            --pulse_cnt[0];
        }
    }
}

static void IRAM_ATTR encoder0_chan_b(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER0_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER0_CHAN_B);

    if (chan_b) {
        if (chan_a) {
            ++pulse_cnt[0];
        } else {
            --pulse_cnt[0];
        }
    } else {
        if (chan_a) {
            --pulse_cnt[0];
        } else {
            ++pulse_cnt[0];
        }
    }
}

static void IRAM_ATTR encoder1_chan_a(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER1_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER1_CHAN_B);

    if (chan_a) {
        if (chan_b) {
            --pulse_cnt[1];
        } else {
            ++pulse_cnt[1];
        }
    } else {
        if (chan_b) {
            ++pulse_cnt[1];
        } else {
            --pulse_cnt[1];
        }
    }
}

static void IRAM_ATTR encoder1_chan_b(void *arg)
{
    const uint32_t chan_a = gpio_get_level(ENCODER1_CHAN_A);
    const uint32_t chan_b = gpio_get_level(ENCODER1_CHAN_B);

    if (chan_b) {
        if (chan_a) {
            ++pulse_cnt[1];
        } else {
            --pulse_cnt[1];
        }
    } else {
        if (chan_a) {
            --pulse_cnt[1];
        } else {
            ++pulse_cnt[1];
        }
    }
}

// ######   PUBLIC FUNCTIONS    ###### //
