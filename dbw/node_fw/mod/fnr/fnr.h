#ifndef FNR_H
#define FNR_H

#include <stdbool.h>
#include <stdint.h>

enum drive_states {
    NEUTRAL,
    FORWARD,
    REVERSE,
};

enum relay_states {
    MANUAL,
    DBW,
};

static void control_relay(bool auth);
static void set_drive_state(enum drive_states state);

#endif
