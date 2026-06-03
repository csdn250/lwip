#include "adc_frame_builder.h"

#include <string.h>

static uint8_t adc_frame_builder_count_channels(uint16_t channel_mask)
{
    uint8_t count = 0U;
    uint8_t ch;

    for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ch++)
    {
        if (0U != (channel_mask & (1U << ch)))
        {
            count++;
        }
    }

    return count;
}

uint16_t adc_frame_builder_build_raw_u16_batch(const adc_acq_sample_t *samples,
                                               uint16_t sample_count,
                                               uint32_t first_seq,
                                               uint16_t channel_mask,
                                               uint8_t *out_buf,
                                               uint16_t out_buf_size)
{
    adc_frame_header_t header;
    uint16_t sample_index;
    uint8_t ch;
    uint8_t channel_count;
    uint16_t offset;
    uint16_t value;
    uint16_t payload_bytes;

    if ((NULL == samples) || (NULL == out_buf) || (0U == sample_count))
    {
        return 0U;
    }

    channel_mask &= (uint16_t)((1U << ADC_FRAME_MAX_CHANNEL_COUNT) - 1U);
    channel_count = adc_frame_builder_count_channels(channel_mask);

    if (0U == channel_count)
    {
        return 0U;
    }

    payload_bytes = (uint16_t)(sample_count * channel_count * sizeof(uint16_t));

    if (out_buf_size < (ADC_FRAME_HEADER_BYTES + payload_bytes))
    {
        return 0U;
    }

    header.magic = ADC_FRAME_MAGIC;
    header.seq = first_seq;
    header.timestamp_us = samples[0].timestamp_us;
    header.channel_mask = channel_mask;
    header.channel_count = channel_count;
    header.sample_format = ADC_FRAME_FORMAT_RAW_U16;
    header.payload_bytes = payload_bytes;

    memcpy(out_buf, &header, sizeof(header));

    offset = ADC_FRAME_HEADER_BYTES;

    /* Payload order: sample0 CH1..CH12, sample1 CH1..CH12, ... */
    for (sample_index = 0U; sample_index < sample_count; sample_index++)
    {
        for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ch++)
        {
            if (0U != (channel_mask & (1U << ch)))
            {
                value = samples[sample_index].raw[ch];

                out_buf[offset] = (uint8_t)(value >> 8);
                out_buf[offset + 1U] = (uint8_t)(value & 0xFFU);
                offset = (uint16_t)(offset + sizeof(uint16_t));
            }
        }
    }

    return offset;
}

uint16_t adc_frame_builder_build_cal_float_batch(const adc_acq_sample_t *samples,
                                                 uint16_t sample_count,
                                                 uint32_t first_seq,
                                                 uint16_t channel_mask,
                                                 const device_adc_cal_config_t *cal_config,
                                                 uint8_t *out_buf,
                                                 uint16_t out_buf_size)
{
    adc_frame_header_t header;
    uint16_t sample_index;
    uint8_t ch;
    uint8_t channel_count;
    uint16_t offset;
    uint16_t payload_bytes;
    float value;
    float k;

    if ((NULL == samples) ||
        (NULL == cal_config) ||
        (NULL == out_buf) ||
        (0U == sample_count))
    {
        return 0U;
    }

    channel_mask &= (uint16_t)((1U << ADC_FRAME_MAX_CHANNEL_COUNT) - 1U);
    channel_count = adc_frame_builder_count_channels(channel_mask);

    if (0U == channel_count)
    {
        return 0U;
    }

    payload_bytes = (uint16_t)(sample_count * channel_count * sizeof(float));

    if (out_buf_size < (ADC_FRAME_HEADER_BYTES + payload_bytes))
    {
        return 0U;
    }

    header.magic = ADC_FRAME_MAGIC;
    header.seq = first_seq;
    header.timestamp_us = samples[0].timestamp_us;
    header.channel_mask = channel_mask;
    header.channel_count = channel_count;
    header.sample_format = ADC_FRAME_FORMAT_CAL_FLOAT;
    header.payload_bytes = payload_bytes;

    memcpy(out_buf, &header, sizeof(header));

    offset = ADC_FRAME_HEADER_BYTES;

    /*
     * Calibration format:
     *   k_raw is stored as k / 0.00000001
     *   converted = raw * k + b_raw
     */
    for (sample_index = 0U; sample_index < sample_count; sample_index++)
    {
        for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ch++)
        {
            if (0U != (channel_mask & (1U << ch)))
            {
                k = (float)cal_config->ch[ch].k_raw * DEVICE_CONFIG_ADC_CAL_K_SCALE;
                value = ((float)samples[sample_index].raw[ch] * k) +
                        (float)cal_config->ch[ch].b_raw;

                memcpy(&out_buf[offset], &value, sizeof(value));
                offset = (uint16_t)(offset + sizeof(value));
            }
        }
    }

    return offset;
}
