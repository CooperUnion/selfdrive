#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>

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
    BL_STATE_COMMIT_CHUNK,
    BL_STATE_FINALIZE,
    BL_STATE_REBOOT_FW,
    BL_STATE_FAULT,
    BL_STATE_RESET,
};

// ######      PROTOTYPES       ###### //

static void set_state(enum bl_state next_state);
static uint32_t time_in_state(void);

// ######     PRIVATE DATA      ###### //

static IsoTpLink isotp_link;
static uint8_t isotp_tx_buf[ISOTP_CHUNK_SIZE + 1];
static uint8_t isotp_rx_buf[ISOTP_CHUNK_SIZE + 1];
static uint8_t isotp_chunk_data[ISOTP_CHUNK_SIZE + 1];

static enum bl_state bl_state = BL_STATE_INIT;
static uint32_t state_start_time;

const esp_partition_t *app_partition;

// ######          CAN          ###### //

void CANRX_onRxCallback_UPD_IsoTpTx(
    const uint8_t * const data,
    const uint8_t len)
{
    isotp_on_can_message(&isotp_link, data, len);
}

void CANTX_populate_TESTBL_Status(struct CAN_Message_TESTBL_Status * const m)
{
    typeof(m->TESTBL_state) s;

    switch (bl_state) {
        case BL_STATE_INIT:             s = CAN_TESTBL_STATE_AWAIT_TRIGGER;     break;
        case BL_STATE_AWAIT_TRIGGER:    s = CAN_TESTBL_STATE_AWAIT_TRIGGER;     break;
        case BL_STATE_RECV_CHUNK:       s = CAN_TESTBL_STATE_RECV_CHUNK;        break;
        case BL_STATE_COMMIT_CHUNK:     s = CAN_TESTBL_STATE_COMMIT_CHUNK;      break;
        case BL_STATE_FINALIZE:         s = CAN_TESTBL_STATE_FINALIZE;          break;
        case BL_STATE_REBOOT_FW:        s = CAN_TESTBL_STATE_REBOOT_FW;         break;
        case BL_STATE_FAULT:            s = CAN_TESTBL_STATE_FAULT;             break;
        case BL_STATE_RESET:            s = CAN_TESTBL_STATE_RESET;             break;
        default:                        s = CAN_TESTBL_STATE_RESET;             break;
    }

    m->TESTBL_state = s;
}

// ######    RATE FUNCTIONS     ###### //

static void bl_init(void);
static void bl_1kHz(void);

const ember_rate_funcs_S bl_rf = {
    .call_init = bl_init,
    .call_1kHz = bl_1kHz,
};

static void bl_init(void) {
    set_state(BL_STATE_INIT);

    // Find the app partition.
    app_partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (!app_partition) {
        log("Failed to find app partition.\n");
        set_state(BL_STATE_RESET);
        return;
    }

    isotp_init_link(
        &isotp_link,
        0x301,
        isotp_tx_buf,
        sizeof(isotp_tx_buf),
        isotp_rx_buf,
        sizeof(isotp_rx_buf)
    );
}

#define ESP_CHECKED(call)                                               \
{                                                                       \
    const esp_err_t err = (call);                                       \
    if (err != ESP_OK) {                                                \
        log(                                                            \
            "Got error in checked op `%s` (err = %d)\n", #call, err);   \
        next_state = BL_STATE_FAULT;                                    \
        break;                                                          \
    }                                                                   \
}                                                                       \

static void bl_1kHz(void) {

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
                log("UPD is present; update triggered.\n");

                // Note the total number of bytes
                update_size = CANRX_get_UPD_updateSizeBytes();

                // Begin OTA update
                ESP_CHECKED(esp_ota_begin(app_partition, OTA_SIZE_UNKNOWN, &ota_handle));

                next_state = BL_STATE_RECV_CHUNK;
            } else if (time_in_state() > 1500U) {
                log("UPD not detected, skipping update.\n");
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
                log("Got new ISOTP full message with size %"PRIu16": %"PRIu16"\n", this_chunk_size, current_chunk);
                log("starting data: %lx\n", *(uint32_t *)isotp_chunk_data);
            }
            /**********************/

            if (isotp_ret == ISOTP_RET_OK) {  // we got a chunk
                // Does our chunk count match the UPD chunk count?
                uint16_t upd_chunk = CANRX_get_UPD_currentIsoTpChunk();
                if (upd_chunk != current_chunk) {
                    log("Chunk count mismatch: UPD is on chunk %"PRIu16" and we're on %"PRIu16"\n",
                        upd_chunk, current_chunk);
                    next_state = BL_STATE_FAULT;
                }

                bytes_so_far += this_chunk_size;
                next_state = BL_STATE_COMMIT_CHUNK;
            } else if (time_in_state() > 1500U) {  // expected chunk but didn't get one
                log("Expected isotp chunk but didn't get one.\n");
                next_state = BL_STATE_FAULT;
            }
            break;

        case BL_STATE_COMMIT_CHUNK:
            // Let's write it in.
            log("Committing chunk %d with size %"PRIu16"...\n", current_chunk, this_chunk_size);
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
            log("Finalizing update...\n");
            ESP_CHECKED(esp_ota_end(ota_handle));
            next_state = BL_STATE_REBOOT_FW;
            break;

        case BL_STATE_REBOOT_FW:
            log("Setting boot partition to app...\n");
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
            log("!! Invalid state %d\n", bl_state);
            next_state = BL_STATE_RESET;
            break;
    }

    if (next_state != bl_state)
        set_state(next_state);
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
        case BL_STATE_COMMIT_CHUNK:     s = "COMMIT_CHUNK";     break;
        case BL_STATE_FINALIZE:         s = "FINALIZE";         break;
        case BL_STATE_REBOOT_FW:        s = "REBOOT_FW";        break;
        case BL_STATE_RESET:            s = "RESET";            break;
        case BL_STATE_FAULT:            s = "FAULT";            break;
    }

    log("At %"PRIu32": next state is %s\n\n", state_start_time, s);
}

static uint32_t time_in_state(void) {
    return (esp_timer_get_time() / (int64_t)1000) - state_start_time;
}

// ######   PUBLIC FUNCTIONS    ###### //

void isotp_user_debug(const char* message, ...) {
    va_list args;
    va_start(args, message);
    fprintf(stderr, "ISOTP Says: ");
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
