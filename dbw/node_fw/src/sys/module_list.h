/* 
 * this header is a logical part of tasking.c. It is not meant to be included
 * anywhere else and does not have include guards.
 */ 

#include "sys/task_glue.h"

/*
 * List an extern definition for each module's struct rate_tasks.
 */
extern const struct rate_funcs blink_hello_rf;
extern const struct rate_funcs can_rf;

/*
 * List of references to the task structs. Order matters - the modules will be
 * initialized in the order they appear here.
 *
 * The functions within each rate (1Hz, 10Hz, etc) will also run in the order
 * they appear here, but it's best not to rely on that fact for program logic.
 */
static const struct rate_funcs* task_list[] = {
    &blink_hello_rf,
    &can_rf,
};
