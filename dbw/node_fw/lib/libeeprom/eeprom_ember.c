#ifdef _EMBER

#include "eeprom.h"

#include "ember_taskglue.h"

struct rate_funcs eeprom_rf = {
  .call_init = eeprom_init,
};

#endif
