#ifndef __ADC_ACQ_SERVICE_H
#define __ADC_ACQ_SERVICE_H

#include "main.h"

#define ADC_ACQ_CHANNEL_COUNT 12U
/*代表 DMA 半缓冲区里放 64 组采样点。每一组采样点包含 12 路数据*/
#define ADC_ACQ_GROUPS_PER_HALF 64U

typedef struct
{
    uint32_t timestamp_us;
    uint16_t raw[ADC_ACQ_CHANNEL_COUNT];
} adc_acq_sample_t;

void adc_acq_service_init(void);
void adc_acq_service_start(void);
uint8_t adc_acq_service_get_sample(adc_acq_sample_t *sample);

#endif //__ADC_ACQ_SERVICE_H

