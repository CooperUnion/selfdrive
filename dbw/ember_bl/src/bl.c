#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include <esp_app_format.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ember_app_desc.h"
#include "ember_taskglue.h"
#include "isotp.h"
#include "isotp_user.h"
#include "opencan_callbacks.h" // for enqueue_tx_message
#include "opencan_rx.h"
#include "opencan_tx.h"

// ######        DEFINES        ###### //

#define log(...) fprintf(stderr, __VA_ARGS__)

#define ISOTP_CHUNK_SIZE 4095

enum bl_state {
    BL_STATE_INIT,
    BL_STATE_AWAIT_TRIGGER,
    BL_STATE_RECV_CHUNK,
    BL_STATE_CHECK_DESC,
    BL_STATE_COMMIT_CHUNK,
    BL_STATE_FINALIZE,
    BL_STATE_REBOOT_FW,
    BL_STATE_FAULT,
    BL_STATE_RESET,
};

// ######      PROTOTYPES       ###### //

static void set_state(enum bl_state next_state);
static uint32_t time_in_state(void);
static bool app_desc_ok(const uint8_t * chunk_data);

// ######     PRIVATE DATA      ###### //

static IsoTpLink isotp_link;
static uint8_t isotp_tx_buf[ISOTP_CHUNK_SIZE + 1];
static uint8_t isotp_rx_buf[ISOTP_CHUNK_SIZE + 1];
static uint8_t isotp_chunk_data[ISOTP_CHUNK_SIZE + 1];

static enum bl_state bl_state = BL_STATE_INIT;
static uint32_t state_start_time;

const esp_partition_t *app_partition;

// ######          CAN          ###### //

#define CANRX_ISOTP_CALLBACK_NAME_(IDENTITY) CANRX_onRxCallback_UPD_IsoTpTx_ ## IDENTITY
#define CANRX_ISOTP_CALLBACK_NAME(IDENTITY) CANRX_ISOTP_CALLBACK_NAME_(IDENTITY)

void CANRX_ISOTP_CALLBACK_NAME(EMBER_NODE_IDENTITY)(
    const uint8_t * const data,
    const uint8_t len)
{
    isotp_on_can_message(&isotp_link, data, len);
}

void CANTX_populateTemplate_Status(struct CAN_TMessage_BlStatus * const m)
{
    typeof(m->state) s;

    switch (bl_state) {
        case BL_STATE_INIT:             s = CAN_T_BLSTATUS_STATE_AWAIT_TRIGGER;     break;
        case BL_STATE_AWAIT_TRIGGER:    s = CAN_T_BLSTATUS_STATE_AWAIT_TRIGGER;     break;
        case BL_STATE_RECV_CHUNK:       s = CAN_T_BLSTATUS_STATE_RECV_CHUNK;        break;
        case BL_STATE_CHECK_DESC:       s = CAN_T_BLSTATUS_STATE_CHECK_DESC;        break;
        case BL_STATE_COMMIT_CHUNK:     s = CAN_T_BLSTATUS_STATE_COMMIT_CHUNK;      break;
        case BL_STATE_FINALIZE:         s = CAN_T_BLSTATUS_STATE_FINALIZE;          break;
        case BL_STATE_REBOOT_FW:        s = CAN_T_BLSTATUS_STATE_REBOOT_FW;         break;
        case BL_STATE_FAULT:            s = CAN_T_BLSTATUS_STATE_FAULT;             break;
        case BL_STATE_RESET:            s = CAN_T_BLSTATUS_STATE_RESET;             break;
        default:                        s = CAN_T_BLSTATUS_STATE_RESET;             break;
    }

    m->state = s;
}

// ######    RATE FUNCTIONS     ###### //

static void bl_init(void);
static void bl_step(void);
static void bl_loop(void *unused);

const ember_rate_funcs_S bl_rf = {
    .call_init = bl_init,
    // .call_1kHz = bl_1kHz,
};

static void bl_init(void) {
    set_state(BL_STATE_INIT);

    log("^^^ EMBER BOOTLOADER v0.1.0 ^^^\n");
    log("--> This bootloader image's node identity is: %s\n", ember_app_description.node_identity);
    log("*** Looking for app partition...\n");

    // Find the app partition.
    app_partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!app_partition) {
        log("!!! Failed to find app partition.\n");
        set_state(BL_STATE_RESET);
        return;
    } else {
        log("--> Found app partition.\n");
        // esp_ota_get_state_partition not working...
        log("*** Getting app information...\n");

        esp_ota_img_states_t app_state;

        const esp_err_t ret =
            esp_ota_get_state_partition(app_partition, &app_state);

        if (ret == ESP_OK) {
            log("*** App is in state %d\n.", app_state);
        } else {
            log("!!! Couldn't get app partition state: %s\n", esp_err_to_name(ret));
        }
    }

    #define CANRX_ISOTP_TX_ID_(IDENTITY) CAN_MSG_ ## IDENTITY ## BL_IsoTpTx_ID
    #define CANRX_ISOTP_TX_ID(IDENTITY) CANRX_ISOTP_TX_ID_(IDENTITY)

    isotp_init_link(
        &isotp_link,
        CANRX_ISOTP_TX_ID(EMBER_NODE_IDENTITY),
        isotp_tx_buf,
        sizeof(isotp_tx_buf),
        isotp_rx_buf,
        sizeof(isotp_rx_buf)
    );

    static TaskHandle_t bl_loop_handle;
    xTaskCreatePinnedToCore(bl_loop, "BL_LOOP", 8192, NULL, 3, &bl_loop_handle, 1);
}

#define ESP_CHECKED(call)                                               \
{                                                                       \
    const esp_err_t err = (call);                                       \
    if (err != ESP_OK) {                                                \
        log(                                                            \
            "!!! Got error in checked op `%s` (err = %d, %s)\n",        \
            #call, err, esp_err_to_name(err));                          \
        next_state = BL_STATE_FAULT;                                    \
        break;                                                          \
    }                                                                   \
}                                                                       \

static void bl_step(void) {
    static esp_ota_handle_t ota_handle = 0;
    static uint32_t update_size = 0;
    static uint32_t bytes_so_far = 0;

    enum bl_state next_state = bl_state;
    switch (bl_state) {
        case BL_STATE_INIT:
            next_state = BL_STATE_AWAIT_TRIGGER;
            break;

        case BL_STATE_AWAIT_TRIGGER:
            if (CANRX_is_node_UPD_ok() && CANRX_get_UPD_currentIsoTpChunk() == 0U) {
                log("--> UPD is present; update triggered.\n");

                // Note the total number of bytes
                update_size = CANRX_get_UPD_updateSizeBytes();

                // Begin OTA update
                ESP_CHECKED(esp_ota_begin(app_partition, OTA_SIZE_UNKNOWN, &ota_handle));

                next_state = BL_STATE_RECV_CHUNK;
            } else if (time_in_state() > 1500U) {
                log("--> UPD not detected, skipping update.\n");
                next_state = BL_STATE_REBOOT_FW;
            }
            break;

        case BL_STATE_RECV_CHUNK:
            /* Take care of ISOTP */
            isotp_poll(&isotp_link);

            static uint16_t this_chunk_size;
            const int isotp_ret = isotp_receive(&isotp_link, isotp_chunk_data, sizeof(isotp_chunk_data), &this_chunk_size);

            static uint16_t current_chunk;
            if (isotp_ret == ISOTP_RET_OK) {
                log("--> Got new ISOTP full message with size %"PRIu16": %"PRIu16"\n", this_chunk_size, current_chunk);
            }
            /**********************/

            if (isotp_ret == ISOTP_RET_OK) {  // we got a chunk
                // Does our chunk count match the UPD chunk count?
                uint16_t upd_chunk = CANRX_get_UPD_currentIsoTpChunk();
                if (upd_chunk != current_chunk) {
                    log("!!! Chunk count mismatch: UPD is on chunk %"PRIu16" and we're on %"PRIu16"\n",
                        upd_chunk, current_chunk);
                    next_state = BL_STATE_FAULT;
                }

                bytes_so_far += this_chunk_size;

                if (current_chunk == 0) {
                    next_state = BL_STATE_CHECK_DESC;
                } else {
                    next_state = BL_STATE_COMMIT_CHUNK;
                }
            } else if (time_in_state() > 1500U) {  // expected chunk but didn't get one
                log("!!! Expected isotp chunk but didn't get one.\n");
                next_state = BL_STATE_FAULT;
            }
            break;

        case BL_STATE_CHECK_DESC:;
            if (app_desc_ok(isotp_chunk_data)) {
                next_state = BL_STATE_COMMIT_CHUNK;
            } else {
                log("!!! ember_app_desc check failed. Aborting.\n");
                next_state = BL_STATE_FAULT;
            }

            break;

        case BL_STATE_COMMIT_CHUNK:
            // Let's write it in.
            log("--> Committing chunk %d...\n", current_chunk);
            ESP_CHECKED(esp_ota_write(ota_handle, isotp_chunk_data, this_chunk_size));

            if (bytes_so_far < update_size) {
                current_chunk++;
                next_state = BL_STATE_RECV_CHUNK;
            } else if (bytes_so_far == update_size) {
                next_state = BL_STATE_FINALIZE;
            } else {
                log("Got more update data than expected: we got %"PRIu32" total bytes so far,"
                       " but expected %"PRIu32" total.\n", bytes_so_far, update_size);
                next_state = BL_STATE_FAULT;
            }
            break;

        case BL_STATE_FINALIZE:
            // Finalize the update
            log("--> Finalizing update...\n");
            ESP_CHECKED(esp_ota_end(ota_handle));
            next_state = BL_STATE_REBOOT_FW;
            break;

        case BL_STATE_REBOOT_FW:
            log("--> Setting boot partition to app...\n");
            ESP_CHECKED(esp_ota_set_boot_partition(app_partition));

            next_state = BL_STATE_RESET;
            break;

        case BL_STATE_FAULT:
            // just linger for a little bit and then reset.
            if (time_in_state() > 1000U) {
                next_state = BL_STATE_RESET;
            }

            break;

        case BL_STATE_RESET:
            log("*** Resetting NOW ***\n");
            esp_restart();

            break; // should be unreachable

        default:
            log("!!! Invalid state %d\n", bl_state);
            next_state = BL_STATE_RESET;
            break;
    }

    if (next_state != bl_state)
        set_state(next_state);
}

static void bl_loop(void *unused) {
    (void)unused;

    for (;;) {
        bl_step();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ######   PRIVATE FUNCTIONS   ###### //

static void set_state(const enum bl_state next_state) {
    bl_state = next_state;
    state_start_time = esp_timer_get_time() / (int64_t)1000;

    const char *s = "INVALID";

    switch (next_state) {
        case BL_STATE_INIT:             s = "INIT";             break;
        case BL_STATE_AWAIT_TRIGGER:    s = "AWAIT_TRIGGER";    break;
        case BL_STATE_RECV_CHUNK:       s = "RECV_CHUNK";       break;
        case BL_STATE_CHECK_DESC:       s = "CHECK_DESC";       break;
        case BL_STATE_COMMIT_CHUNK:     s = "COMMIT_CHUNK";     break;
        case BL_STATE_FINALIZE:         s = "FINALIZE";         break;
        case BL_STATE_REBOOT_FW:        s = "REBOOT_FW";        break;
        case BL_STATE_RESET:            s = "RESET";            break;
        case BL_STATE_FAULT:            s = "FAULT";            break;
    }

    log("--> At %"PRIu32": next state is %s\n", state_start_time, s);
}

static uint32_t time_in_state(void) {
    return (esp_timer_get_time() / (int64_t)1000) - state_start_time;
}

static bool app_desc_ok(const uint8_t * const chunk_data) {
    // get the app description out of the first chunk
    const void * const new_app_desc_addr =
        // see ember_app_desc.h
        chunk_data +
        sizeof(esp_image_header_t) +
        sizeof(esp_image_segment_header_t) +
        sizeof(esp_app_desc_t);

    ember_app_desc_v1_t new_app_desc;
    memcpy(&new_app_desc, new_app_desc_addr, sizeof(new_app_desc));

    // check ember_magic
    const bool ember_magic_matches = !strncmp(
        EMBER_MAGIC,
        new_app_desc.ember_magic,
        sizeof(EMBER_MAGIC)
    );

    if (!ember_magic_matches) {
        _Static_assert(sizeof(EMBER_MAGIC) == 8, "Need EMBER_MAGIC to be 8 chars for log below");
        log("!!! Invalid ember_magic for new app's ember_app_desc: \"%.8s\".\n", new_app_desc.ember_magic);
        return false;
    }

    // check app_desc_version
    if (new_app_desc.app_desc_version != EMBER_APP_DESC_VERSION) {
        log("!!! Unexpected version %"PRIu16" (expected %"PRIu16") for new app's app_desc_version.\n",
            new_app_desc.app_desc_version,
            EMBER_APP_DESC_VERSION
        );
        return false;
    }

    // check that node_identity matches ours
    const bool identities_match = !strncmp(
        ember_app_description.node_identity,
        new_app_desc.node_identity,
        sizeof(ember_app_description.node_identity));

    if (identities_match) {
        log("--> Identity of new app matches bootloader. Proceeding.\n");
    } else {
        log("--> Identity of new app (\"%.16s\") does not match bootloader!\n", new_app_desc.node_identity);
        return false;
    }

    return true;
}

// ######   PUBLIC FUNCTIONS    ###### //

void isotp_user_debug(const char* message, ...) {
    va_list args;
    va_start(args, message);
    fprintf(stderr, "*** ISOTP Says: ");
    vfprintf(stderr, message, args);
    fprintf(stderr, "\n");
    va_end(args);
}

int isotp_user_send_can(
    const uint32_t arbitration_id,
    const uint8_t * const data,
    const uint8_t size)
{
    CAN_callback_enqueue_tx_message(data, size, arbitration_id);

    return ISOTP_RET_OK;
}

uint32_t isotp_user_get_ms(void) {
    return esp_timer_get_time() / (int64_t)1000;
}
