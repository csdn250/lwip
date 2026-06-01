#include "adc_frame_builder.h"
#include <string.h>

static uint8_t adc_frame_builder_count_channels(uint16_t channel_mask)
{
    uint8_t count;
    uint8_t ch;

    count = 0U;

    for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ++ch)
    {
        if (0U != (channel_mask & (1U << ch)))
        {
            ++count;
        }
    }

    return count;
}

uint16_t adc_frame_builder_build_raw_float(const adc_acq_sample_t *sample,
                                           uint16_t channel_mask,
                                           uint8_t *out_buf,
                                           uint16_t out_buf_size)
{
    adc_frame_header_t header;
    float value;
    uint8_t ch;
    uint8_t channel_count;
    uint16_t offset;

    if ((NULL == sample) || (NULL == out_buf))
    {
        return 0U;
    }

    channel_mask &= (uint16_t)((1U << ADC_FRAME_MAX_CHANNEL_COUNT) - 1U);
    channel_count = adc_frame_builder_count_channels(channel_mask);

    if (0U == channel_count)
    {
        return 0U;
    }

    if (out_buf_size < (ADC_FRAME_HEADER_BYTES + (channel_count * sizeof(float))))
    {
        return 0U;
    }

    header.magic = ADC_FRAME_MAGIC;
    header.seq = sample->timestamp_us;
    header.timestamp_us = sample->timestamp_us;
    header.channel_mask = channel_mask;
    header.channel_count = channel_count;
    header.sample_format = ADC_FRAME_FORMAT_RAW_FLOAT;
    header.payload_bytes = (uint16_t)(channel_count * sizeof(float));

    memcpy(out_buf, &header, sizeof(header));

    offset = ADC_FRAME_HEADER_BYTES;

    for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ++ch)
    {
        if (0U != (channel_mask & (1U << ch)))
        {
            value = (float)((int32_t)sample->raw[ch] - 32768);

            memcpy(&out_buf[offset], &value, sizeof(float));
            offset = (uint16_t)(offset + sizeof(float));
        }
    }

    return offset;
}
