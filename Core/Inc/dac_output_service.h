#ifndef __DAC_OUTPUT_SERVICE_H
#define __DAC_OUTPUT_SERVICE_H

#include "main.h"
#include "adc_acq_service.h"

void dac_output_service_init(void);
void dac_output_service_process(void);
void dac_output_service_process_adc_cascade(void);
void dac_output_service_apply_adc_sample(const adc_acq_sample_t *sample);

#endif /*__DAC_OUTPUT_SERVICE_H*/

