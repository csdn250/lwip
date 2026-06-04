#ifndef __DAC_TPC112S4_H
#define __DAC_TPC112S4_H

#include "main.h"

#define DAC_TPC112S4_CHANNEL_COUNT 4U

void dac_tpc112s4_write_channel(uint8_t channel,
                                uint16_t code);

void dac_tpc112s4_write_all(uint16_t ch1,
                            uint16_t ch2,
                            uint16_t ch3,
                            uint16_t ch4);

void dac_tpc112s4_test_pattern(void);

#endif /* __DAC_TPC112S4_H */
