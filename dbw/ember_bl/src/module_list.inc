/*
 * this is a logical part of entry.c. It is not meant to be included anywhere
 * else and does not have include guards.
 */

#include "ember_common.h"
#include "ember_taskglue.h"

/*
 * List an extern definition for each module's struct rate_tasks.
 */
extern ember_rate_funcs_S leds_rf;
extern ember_rate_funcs_S can_rf;
extern ember_rate_funcs_S bl_rf;

/*
 * Single extern definition for the one-per-build mod/ module.
 */
extern ember_rate_funcs_S module_rf;

/*
 * List of references to the task structs. Order matters - the modules will be
 * initialized in the order they appear here.
 *
 * The functions within each rate (1Hz, 10Hz, etc) will also run in the order
 * they appear here, but it's best not to rely on that fact for program logic.
 */
ember_rate_funcs_S* ember_task_list[] = {
    &leds_rf,
    &can_rf,
    &bl_rf,
};

const size_t ember_task_count = ARRAY_SIZE(ember_task_list);
