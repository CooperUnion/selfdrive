#ifndef EEPROM_H
#define EEPROM_H

#include <esp_err.h>
#include <stddef.h>
#include <stdint.h>

void eeprom_init(void);

esp_err_t eeprom_read(uint16_t addr, uint8_t *data, size_t len);
esp_err_t eeprom_write(uint16_t addr, const uint8_t *data, size_t len);
esp_err_t eeprom_write_byte(uint16_t addr, uint8_t data);

#endif
