#ifndef __EEPROM_STORAGE_H
#define __EEPROM_STORAGE_H

#include "main.h"

void eeprom_storage_init(void);

uint8_t eeprom_storage_is_ready(void);

HAL_StatusTypeDef eeprom_storage_read(uint32_t address,
                                      uint8_t *data,
                                      uint16_t len);

HAL_StatusTypeDef eeprom_storage_write(uint32_t address,
                                       const uint8_t *data,
                                       uint16_t len);

#endif /* __EEPROM_STORAGE_H */

