#include "adc_frame_builder.h"

#include <string.h>

static void adc_frame_builder_put_u16_be(uint8_t *buf,
                                         uint16_t *offset,
                                         uint16_t value)
{
    buf[(*offset)++] = (uint8_t)(value >> 8);
    buf[(*offset)++] = (uint8_t)(value & 0xFFU);
}

static void adc_frame_builder_put_u32_be(uint8_t *buf,
                                         uint16_t *offset,
                                         uint32_t value)
{
    buf[(*offset)++] = (uint8_t)(value >> 24);
    buf[(*offset)++] = (uint8_t)(value >> 16);
    buf[(*offset)++] = (uint8_t)(value >> 8);
    buf[(*offset)++] = (uint8_t)(value & 0xFFU);
}

static void adc_frame_builder_put_float_be(uint8_t *buf,
                                           uint16_t *offset,
                                           float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    adc_frame_builder_put_u32_be(buf, offset, bits);
}

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

    if ((payload_bytes > ADC_FRAME_MAX_RAW_PAYLOAD_BYTES) ||
        (out_buf_size < ADC_FRAME_MAX_BYTES))
    {
        return 0U;
    }

    /*
     * The ADC stream payload sent to the PC is fixed-size. payload_bytes in
     * the inner ADC header still tells the PC how many bytes are real samples;
     * the remaining bytes are zero padding.
     */
    memset(out_buf, 0, ADC_FRAME_MAX_BYTES);

    offset = 0U;
    adc_frame_builder_put_u32_be(out_buf, &offset, ADC_FRAME_MAGIC);
    adc_frame_builder_put_u32_be(out_buf, &offset, first_seq);
    adc_frame_builder_put_u32_be(out_buf, &offset, samples[0].timestamp_us);
    adc_frame_builder_put_u16_be(out_buf, &offset, channel_mask);
    adc_frame_builder_put_u16_be(out_buf, &offset, channel_count);
    adc_frame_builder_put_u16_be(out_buf, &offset, ADC_FRAME_FORMAT_RAW_U16);
    adc_frame_builder_put_u16_be(out_buf, &offset, payload_bytes);

    /* Payload order: sample0 CH1..CH12, sample1 CH1..CH12, ... */
    for (sample_index = 0U; sample_index < sample_count; sample_index++)
    {
        for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ch++)
        {
            if (0U != (channel_mask & (1U << ch)))
            {
                value = samples[sample_index].raw[ch];
                adc_frame_builder_put_u16_be(out_buf, &offset, value);
            }
        }
    }

    (void)offset;
    return ADC_FRAME_MAX_BYTES;
}

uint16_t adc_frame_builder_build_cal_float_batch(const adc_acq_sample_t *samples,
                                                 uint16_t sample_count,
                                                 uint32_t first_seq,
                                                 uint16_t channel_mask,
                                                 const device_adc_cal_config_t *cal_config,
                                                 uint8_t *out_buf,
                                                 uint16_t out_buf_size)
{
    uint16_t sample_index;
    uint8_t ch;
    uint8_t channel_count;
    uint16_t offset;
    uint16_t payload_bytes;
    float value;

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

    if ((payload_bytes > ADC_FRAME_MAX_CONVERTED_PAYLOAD_BYTES) ||
        (out_buf_size < ADC_FRAME_MAX_BYTES))
    {
        return 0U;
    }

    /*
     * Keep stream payload length fixed while preserving the real valid data
     * length in the inner ADC header's payload_bytes field.
     */
    memset(out_buf, 0, ADC_FRAME_MAX_BYTES);

    offset = 0U;
    adc_frame_builder_put_u32_be(out_buf, &offset, ADC_FRAME_MAGIC);
    adc_frame_builder_put_u32_be(out_buf, &offset, first_seq);
    adc_frame_builder_put_u32_be(out_buf, &offset, samples[0].timestamp_us);
    adc_frame_builder_put_u16_be(out_buf, &offset, channel_mask);
    adc_frame_builder_put_u16_be(out_buf, &offset, channel_count);
    adc_frame_builder_put_u16_be(out_buf, &offset, ADC_FRAME_FORMAT_CAL_FLOAT);
    adc_frame_builder_put_u16_be(out_buf, &offset, payload_bytes);

    /*
     * Calibration format:
     *   converted = raw * k + b
     *   k/b are stored as float values in device_config.
     */
    for (sample_index = 0U; sample_index < sample_count; sample_index++)
    {
        for (ch = 0U; ch < ADC_FRAME_MAX_CHANNEL_COUNT; ch++)
        {
            if (0U != (channel_mask & (1U << ch)))
            {
                value = ((float)samples[sample_index].raw[ch] *
                         cal_config->ch[ch].k) +
                        cal_config->ch[ch].b;

                adc_frame_builder_put_float_be(out_buf, &offset, value);
            }
        }
    }

    (void)offset;
    return ADC_FRAME_MAX_BYTES;
}
