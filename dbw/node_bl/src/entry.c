#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/timer.h>
#include <driver/twai.h>
#include <string.h>

#include "esp_ota_ops.h"

#include "can.h"
#include "can_gen.h"

// ######        DEFINES        ###### //

#define TIMER_GROUP 0, 0

#define MAX_TRIGGER_WAIT_TIME_MS 1000

#define GPIO_LED1 GPIO_NUM_33
#define GPIO_LED2 GPIO_NUM_32

static uint8_t fw_image[1024 * 160];

// ######      PROTOTYPES       ###### //

// ######     PRIVATE DATA      ###### //

static struct {
    const esp_partition_t *running, *boot, *update;
} part_info;

static enum bl_state_E {
    BL_STATE_AWAIT_TRIGGER,
    BL_STATE_BOOT_INTO_FW,
    BL_STATE_RECIEVING_IMG,
    BL_STATE_WRITING_IMG,
    BL_STATE_FAULT,
} bl_state;

static uint64_t state_start_time;

// ######          CAN          ###### //

void can_send(int id, int val)
{
    twai_message_t message;
    message.extd = 0;
    message.rtr = 0;

    message.identifier = id;

    message.data_length_code = 1;

    message.data[0] = val;
    message.data[1] = 0;
    message.data[2] = 0;
    message.data[3] = 0;

    esp_err_t r = twai_transmit(&message, 0);
}

// ######   PRIVATE FUNCTIONS   ###### //

static void init_leds(void)
{
    gpio_pad_select_gpio(GPIO_LED1);
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED1, 0);

    gpio_pad_select_gpio(GPIO_LED2);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LED2, 0);
}

static void set_led1(bool state)
{
    gpio_set_level(GPIO_LED1, state);
}

static void set_led2(bool state)
{
    gpio_set_level(GPIO_LED2, state);
}

static void populate_part_info(void)
{
    part_info.running = esp_ota_get_running_partition();
    part_info.boot = esp_ota_get_boot_partition();
    part_info.update = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
}

static void set_up_timer(void)
{
    const timer_config_t time_cfg = {
        .alarm_en = TIMER_ALARM_DIS,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 8000, // 80MHz / 8000 = 10kHz timer rate
    };

    timer_init(TIMER_GROUP, &time_cfg);
    timer_set_counter_value(TIMER_GROUP, 0);
    timer_start(TIMER_GROUP);
}

static uint64_t get_millis_since_boot(void)
{
    uint64_t t = 0;
    timer_get_counter_value(TIMER_GROUP, &t);

    return t / 10; // 0.1ms -> ms
}

static void set_bl_state(enum bl_state_E s)
{
    bl_state = s;
    state_start_time = get_millis_since_boot();
}

static uint64_t get_time_in_current_state(void)
{
    uint64_t now = get_millis_since_boot();
    return now - state_start_time;
}

// ######   PUBLIC FUNCTIONS    ###### //

void app_main()
{
    set_up_timer();
    can_init();

    init_leds();
    set_led1(1);
    set_led2(0);

    populate_part_info();

    set_bl_state(BL_STATE_AWAIT_TRIGGER);

    twai_message_t msg;
    uint32_t total_size = 0;

    for (;;) {
        switch (bl_state) {
            case BL_STATE_AWAIT_TRIGGER:
            {
                if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
                    if (msg.identifier == CAN_DBW_UPDATERUPDATETRIGGER_FRAME_ID) {
                        can_send(CAN_DBW_NODEUPDATERESPONSE_FRAME_ID, 0);
                        set_bl_state(BL_STATE_RECIEVING_IMG);
                        break;
                    }
                }

                if (get_time_in_current_state() > 3000) {
                    set_bl_state(BL_STATE_BOOT_INTO_FW);
                    break;
                }

                break;
            }

            case BL_STATE_BOOT_INTO_FW:
            {
                esp_err_t err = esp_ota_set_boot_partition(part_info.update);
                can_send(0x17, err);

                if (err != ESP_OK) {
                    set_bl_state(BL_STATE_FAULT);
                    break;
                }

                can_send(0x18, err);

                esp_restart();
            }

            case BL_STATE_RECIEVING_IMG:
            {
                set_led1(1);
                set_led2(1);
                if (twai_receive(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
                    switch (msg.identifier) {
                        case CAN_DBW_UPDATERUPDATEDATA_FRAME_ID:
                        {
                            uint32_t position = 0;

                            position |= msg.data[0];
                            position |= msg.data[1] << 8;

                            position *= 5;

                            memcpy(fw_image + position, msg.data + 3, 5);
                            break;
                        }

                        case CAN_DBW_UPDATERUPDATEDONE_FRAME_ID:
                        {
                            total_size |= msg.data[0];
                            total_size |= msg.data[1] << 8;
                            total_size |= msg.data[2] << 16;
                            total_size |= msg.data[3] << 24;

                            set_bl_state(BL_STATE_WRITING_IMG);
                            break;
                        }

                        default:
                            break;
                    }
                }

                break;
            }

            case BL_STATE_WRITING_IMG:
            {
                set_led1(0);
                set_led2(1);
                esp_ota_handle_t update_handle = 0;
                esp_err_t err = esp_ota_begin(part_info.update, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);

                can_send(0x14, err);
                if (err != ESP_OK) {
                    esp_ota_abort(update_handle);

                    set_bl_state(BL_STATE_FAULT);
                    break;
                }

                err = esp_ota_write(update_handle, fw_image, total_size);
                can_send(0x15, err);

                if (err != ESP_OK) {
                    esp_ota_abort(update_handle);
                    set_bl_state(BL_STATE_FAULT);
                    break;
                }

                err = esp_ota_end(update_handle);
                can_send(0x16, err);
                if (err != ESP_OK) {
                    set_bl_state(BL_STATE_FAULT);
                    break;
                }

                set_bl_state(BL_STATE_BOOT_INTO_FW);
            }

            break;

            case BL_STATE_FAULT:
            {
                bool blink = (get_time_in_current_state() % 200) > 100;
                set_led1(blink);
                set_led2(!blink);

                if (get_time_in_current_state() > 2000) {
                    esp_restart();
                }

                break;
            }

            default:
                set_bl_state(BL_STATE_BOOT_INTO_FW);
        }
    }
}
