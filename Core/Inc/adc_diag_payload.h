#ifndef __ADC_DIAG_PAYLOAD_H
#define __ADC_DIAG_PAYLOAD_H

#include "main.h"

uint16_t adc_diag_payload_build_log_snapshot(uint8_t *payload,
                                             uint16_t max_len);

uint16_t adc_diag_payload_build_dac_status(uint8_t *payload,
                                           uint16_t max_len);

#endif /* __ADC_DIAG_PAYLOAD_H */
