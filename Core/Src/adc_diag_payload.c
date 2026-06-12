#include "adc_diag_payload.h"

#include "adc_proto.h"
#include "app_log.h"
#include "app_log_persist.h"
#include "dac_output_service.h"
#include "device_config.h"

#define ADC_LOG_SNAPSHOT_VERSION 1U
#define ADC_LOG_SNAPSHOT_MAX_RECORDS 8U
#define ADC_LOG_SNAPSHOT_HEADER_BYTES 4U
#define ADC_LOG_RECORD_WIRE_BYTES 16U

#define DAC_STATUS_BYTES (DEVICE_CONFIG_DAC_CHANNEL_COUNT * sizeof(uint16_t))

static app_log_record_t s_log_snapshot_records[ADC_LOG_SNAPSHOT_MAX_RECORDS];

uint16_t adc_diag_payload_build_log_snapshot(uint8_t *payload,
                                             uint16_t max_len)
{
    uint16_t index;
    uint16_t count;
    uint16_t i;

    if ((NULL == payload) || (max_len < ADC_LOG_SNAPSHOT_HEADER_BYTES))
    {
        return 0U;
    }

    count = app_log_snapshot(s_log_snapshot_records,
                             ADC_LOG_SNAPSHOT_MAX_RECORDS);

    index = 0U;
    payload[index++] = ADC_LOG_SNAPSHOT_VERSION;
    payload[index++] = (uint8_t)count;
    payload[index++] = ADC_LOG_RECORD_WIRE_BYTES;
    payload[index++] = 0U;

    for (i = 0U; i < count; i++)
    {
        if ((uint16_t)(index + ADC_LOG_RECORD_WIRE_BYTES) > max_len)
        {
            break;
        }

        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].tick_ms);
        adc_proto_put_u16_be(payload, &index, s_log_snapshot_records[i].event);
        adc_proto_put_u16_be(payload, &index, s_log_snapshot_records[i].arg0);
        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].arg1);
        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].arg2);
    }

    return index;
}

uint16_t adc_diag_payload_build_persist_log_snapshot(uint8_t *payload,
                                                     uint16_t max_len)
{
    uint16_t index;
    uint16_t count;
    uint16_t i;

    if ((NULL == payload) || (max_len < ADC_LOG_SNAPSHOT_HEADER_BYTES))
    {
        return 0U;
    }

    count = app_log_persist_load(s_log_snapshot_records,
                                 ADC_LOG_SNAPSHOT_MAX_RECORDS);

    index = 0U;
    payload[index++] = ADC_LOG_SNAPSHOT_VERSION;
    payload[index++] = (uint8_t)count;
    payload[index++] = ADC_LOG_RECORD_WIRE_BYTES;
    payload[index++] = 1U; /* 1 = EEPROM persistent snapshot */

    for (i = 0U; i < count; i++)
    {
        if ((uint16_t)(index + ADC_LOG_RECORD_WIRE_BYTES) > max_len)
        {
            break;
        }

        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].tick_ms);
        adc_proto_put_u16_be(payload, &index, s_log_snapshot_records[i].event);
        adc_proto_put_u16_be(payload, &index, s_log_snapshot_records[i].arg0);
        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].arg1);
        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].arg2);
    }

    return index;
}

uint16_t adc_diag_payload_build_dac_status(uint8_t *payload,
                                           uint16_t max_len)
{
    uint16_t codes[DEVICE_CONFIG_DAC_CHANNEL_COUNT];
    uint16_t index;
    uint8_t ch;

    if ((NULL == payload) || (max_len < DAC_STATUS_BYTES))
    {
        return 0U;
    }

    dac_output_service_get_current_codes(codes,
                                         DEVICE_CONFIG_DAC_CHANNEL_COUNT);

    index = 0U;
    for (ch = 0U; ch < DEVICE_CONFIG_DAC_CHANNEL_COUNT; ch++)
    {
        adc_proto_put_u16_be(payload, &index, codes[ch]);
    }

    return index;
}
