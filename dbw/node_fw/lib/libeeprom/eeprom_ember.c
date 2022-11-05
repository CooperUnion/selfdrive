#ifdef _EMBER

#include "eeprom.h"

#include "ember_taskglue.h"

ember_rate_funcs_S eeprom_rf = {
  .call_init = eeprom_init,
};

#endif
