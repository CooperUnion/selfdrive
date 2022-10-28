#ifdef _EMBER

#include "eeprom.h"

#include "task_glue.h"

struct rate_funcs eeprom_rf = {
  .call_init = eeprom_init,
};

#endif
