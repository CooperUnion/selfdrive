#ifndef TASK_GLUE_H
#define TASK_GLUE_H

/*
 * Struct to be defined by each module with rate functions.
 * See tasking.c and module_list.h.
 */
struct rate_funcs_ {
    void (*call_init)();
    void (*call_1Hz)();
    void (*call_10Hz)();
    void (*call_100Hz)();
    void (*call_1kHz)();
};

typedef const struct rate_funcs_ ember_rate_funcs_S;

#endif
