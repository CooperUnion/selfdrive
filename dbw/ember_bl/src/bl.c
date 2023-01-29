#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>

#include <esp_timer.h>

#include "isotp.h"
#include "isotp_user.h"

#include "opencan_callbacks.h" // for enqueue_tx_message
#include "opencan_rx.h"

/* ISOTP */
#define ISOTP_BUFFER_SIZE 4096

static IsoTpLink isotp_link;
static uint8_t isotp_tx_buf[ISOTP_BUFFER_SIZE];
static uint8_t isotp_rx_buf[ISOTP_BUFFER_SIZE];

static uint8_t isotp_message[ISOTP_BUFFER_SIZE];

/* CAN */
void CANRX_onRxCallback_UPD_IsoTpTx(
    const uint8_t * const data,
    const uint8_t len)
{
    printf("isotp data:");
    for (int i = 0; i < len; i++) {
        printf(" 0x%" PRIX8, data[i]);
    }
    printf("\n");

    isotp_on_can_message(&isotp_link, data, len);
}

/* Rate Functions */
void bl_init(void) {
    isotp_init_link(
        &isotp_link,
        0x301,
        isotp_tx_buf,
        sizeof(isotp_tx_buf),
        isotp_rx_buf,
        sizeof(isotp_rx_buf)
    );
}

void bl_1kHz(void) {
    isotp_poll(&isotp_link);

    uint16_t rec_size;
    int rec = isotp_receive(&isotp_link, isotp_message, sizeof(isotp_message), &rec_size);

    if (rec == ISOTP_RET_OK) {
        printf("Got new ISOTP full message with size %" PRIu16 "\n", rec_size);
    }
}

/* ISOTP Callbacks */
void isotp_user_debug(const char* message, ...) {
    va_list args;
    va_start(args, message);
    vfprintf(stderr, message, args);
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
    return esp_timer_get_time() * 1000U;
}
