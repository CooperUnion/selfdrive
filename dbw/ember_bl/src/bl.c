#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include <esp_timer.h>

#include "ember_taskglue.h"
#include "isotp.h"
#include "isotp_user.h"
#include "opencan_callbacks.h" // for enqueue_tx_message
#include "opencan_rx.h"

// ######        DEFINES        ###### //

#define ISOTP_BUFFER_SIZE 4096

enum bl_state {
    BL_STATE_INIT,
    BL_STATE_AWAIT_TRIGGER,
    BL_STATE_AWAIT_START,
    BL_STATE_AWAIT_ISOTP,
    BL_STATE_RECV_ISOTP,
    BL_STATE_FINALIZE,
    BL_STATE_REBOOT_FW,
    BL_STATE_RESET,
};

// ######      PROTOTYPES       ###### //

static void set_state(enum bl_state next_state);
static uint32_t time_in_state(void);

// ######     PRIVATE DATA      ###### //

static IsoTpLink isotp_link;
static uint8_t isotp_tx_buf[ISOTP_BUFFER_SIZE];
static uint8_t isotp_rx_buf[ISOTP_BUFFER_SIZE];
static uint8_t isotp_message[ISOTP_BUFFER_SIZE];

static enum bl_state bl_state = BL_STATE_INIT;
static uint32_t state_start_time;

// ######          CAN          ###### //

void CANRX_onRxCallback_UPD_IsoTpTx(
    const uint8_t * const data,
    const uint8_t len)
{
    // printf("isotp data:");
    // for (int i = 0; i < len; i++) {
    //     printf(" 0x%" PRIX8, data[i]);
    // }
    // printf("\n");

    isotp_on_can_message(&isotp_link, data, len);
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

    isotp_init_link(
        &isotp_link,
        0x301,
        isotp_tx_buf,
        sizeof(isotp_tx_buf),
        isotp_rx_buf,
        sizeof(isotp_rx_buf)
    );
}

static void bl_1kHz(void) {
    isotp_poll(&isotp_link);

    uint16_t rec_size;
    const int rec = isotp_receive(&isotp_link, isotp_message, sizeof(isotp_message), &rec_size);

    if (rec == ISOTP_RET_OK) {
        printf("Got new ISOTP full message with size %" PRIu16 "\n", rec_size);
    }

    enum bl_state next_state = bl_state;

    switch (bl_state) {
        case BL_STATE_INIT: break;
        case BL_STATE_AWAIT_TRIGGER:
        case BL_STATE_AWAIT_START:
        case BL_STATE_AWAIT_ISOTP:
        case BL_STATE_RECV_ISOTP:
        case BL_STATE_FINALIZE:
        case BL_STATE_REBOOT_FW:
        default:
            // printf("!! Invalid state %d\n", bl_state);
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
}

static uint32_t time_in_state(void) {
    return (esp_timer_get_time() / (int64_t)1000) - state_start_time;
}

// ######   PUBLIC FUNCTIONS    ###### //

void isotp_user_debug(const char* message, ...) {
    va_list args;
    va_start(args, message);
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
