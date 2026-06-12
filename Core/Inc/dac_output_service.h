#ifndef __DAC_OUTPUT_SERVICE_H
#define __DAC_OUTPUT_SERVICE_H

#include "main.h"
#include "adc_acq_service.h"

void dac_output_service_init(void);
void dac_output_service_process(void);
void dac_output_service_process_adc_cascade(void);
uint8_t dac_output_service_is_adc_cascade_enabled(void);
uint8_t dac_output_service_is_manual_enabled(void);
void dac_output_service_apply_adc_sample(const adc_acq_sample_t *sample);
void dac_output_service_get_current_codes(uint16_t *codes,
                                          uint8_t max_count);

#endif /*__DAC_OUTPUT_SERVICE_H*/

