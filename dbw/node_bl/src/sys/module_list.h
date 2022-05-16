#ifndef MODULE_LIST_H
#define MODULE_LIST_H

#include "sys/task_glue.h"

/*
 * List an extern definition for each src/ module's struct rate_tasks.
 */
extern const struct rate_funcs base_rf;
extern const struct rate_funcs bl_rf;
extern const struct rate_funcs can_rf;
extern const struct rate_funcs eeprom_rf;

/*
 * List of references to the task structs. Order matters - the modules will be
 * initialized in the order they appear here.
 *
 * The functions within each rate (1Hz, 10Hz, etc) will also run in the order
 * they appear here, but it's best not to rely on that fact for program logic.
 */
static const struct rate_funcs* task_list[] = {
    &base_rf,
    &bl_rf,
    &can_rf,
    &eeprom_rf,
};

#endif
