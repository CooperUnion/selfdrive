#ifndef MODULE_LIST_H
#define MODULE_LIST_H

#include "sys/task_glue.h"

/*
 * List an extern definition for each src/ module's struct rate_tasks.
 */
extern const struct rate_funcs can_rf;
extern const struct rate_funcs eeprom_rf;

#endif
