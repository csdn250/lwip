#ifndef __ADC_STATUS_SERVICE_H
#define __ADC_STATUS_SERVICE_H

#include "main.h"
#include "adc_acq_service.h"

typedef struct
{
    uint32_t alarm_flags;
    uint32_t state_flags;
    float adc_avg[ADC_ACQ_CHANNEL_COUNT];
    uint32_t avg_sample_count;
} adc_status_snapshot_t;

#define ADC_STATUS_STATE_TCP_CONNECTED   (1UL << 0)
#define ADC_STATUS_STATE_ADC_STREAMING   (1UL << 1)
#define ADC_STATUS_STATE_DAC_CASCADE     (1UL << 2)
#define ADC_STATUS_STATE_EEPROM_READY    (1UL << 3)
#define ADC_STATUS_STATE_ADC_DMA_ACTIVE  (1UL << 4)
#define ADC_STATUS_STATE_DAC_MANUAL      (1UL << 5)

#define ADC_STATUS_ALARM_EEPROM_NOT_READY (1UL << 0)
#define ADC_STATUS_ALARM_ADC_DMA_STOPPED (1UL << 1)
#define ADC_STATUS_ALARM_TCP_SEND_FAIL (1UL << 2)

void adc_status_service_init(void);
void adc_status_service_process(void);
void adc_status_service_update_sample(const adc_acq_sample_t *sample);
void adc_status_service_set_runtime_flags(uint32_t state_flags,
                                          uint32_t alarm_flags);
void adc_status_service_get_snapshot(adc_status_snapshot_t *snapshot);

#endif /*__ADC_STATUS_SERVICE_H*/
