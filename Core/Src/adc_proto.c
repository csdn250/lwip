#include "adc_proto.h"
#include <string.h>

uint32_t adc_proto_crc32(const uint8_t *data,
                         uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;
    uint8_t bit;

    if (NULL == data)
    {
        return 0UL;
    }

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

uint16_t adc_proto_payload_len(const uint8_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    return (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
}

uint8_t adc_proto_is_frame_valid(const uint8_t *frame,
                                 uint16_t frame_len,
                                 uint16_t max_payload_len)
{
    uint16_t payload_len;
    uint16_t crc_len;
    uint32_t crc_calc;
    uint32_t crc_recv;

    if ((NULL == frame) || (frame_len < ADC_PROTO_MIN_FRAME_SIZE))
    {
        return 0U;
    }

    if ((frame[0] != ADC_PROTO_SOF0) ||
        (frame[1] != ADC_PROTO_SOF1))
    {
        return 0U;
    }

    if ((frame[frame_len - 2U] != ADC_PROTO_EOF0) ||
        (frame[frame_len - 1U] != ADC_PROTO_EOF1))
    {
        return 0U;
    }

    payload_len = adc_proto_payload_len(frame);
    if (payload_len > max_payload_len)
    {
        return 0U;
    }

    if (frame_len != (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD))
    {
        return 0U;
    }

    crc_len = (uint16_t)(ADC_PROTO_HEADER_SIZE + payload_len);
    crc_calc = adc_proto_crc32(frame, crc_len);
    crc_recv = ((uint32_t)frame[ADC_PROTO_HEADER_SIZE + payload_len] << 24) |
               ((uint32_t)frame[ADC_PROTO_HEADER_SIZE + payload_len + 1U] << 16) |
               ((uint32_t)frame[ADC_PROTO_HEADER_SIZE + payload_len + 2U] << 8) |
               frame[ADC_PROTO_HEADER_SIZE + payload_len + 3U];

    return (crc_calc == crc_recv) ? 1U : 0U;
}

uint16_t adc_proto_build_frame(uint8_t *frame,
                               uint16_t frame_buf_size,
                               uint8_t cmd,
                               const uint8_t *payload,
                               uint16_t payload_len)
{
    uint16_t frame_len;
    uint16_t crc_len;
    uint32_t crc;

    if (NULL == frame)
    {
        return 0U;
    }

    if ((payload_len > 0U) && (NULL == payload))
    {
        return 0U;
    }

    frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);
    if (frame_buf_size < frame_len)
    {
        return 0U;
    }

    frame[0] = ADC_PROTO_SOF0;
    frame[1] = ADC_PROTO_SOF1;
    frame[2] = cmd;
    frame[3] = (uint8_t)(payload_len >> 8);
    frame[4] = (uint8_t)(payload_len & 0xFFU);

    if (payload_len > 0U)
    {
        memcpy(&frame[ADC_PROTO_HEADER_SIZE], payload, payload_len);
    }

    crc_len = (uint16_t)(ADC_PROTO_HEADER_SIZE + payload_len);
    crc = adc_proto_crc32(frame, crc_len);

    frame[ADC_PROTO_HEADER_SIZE + payload_len] = (uint8_t)(crc >> 24);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 1U] = (uint8_t)(crc >> 16);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 2U] = (uint8_t)(crc >> 8);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 3U] = (uint8_t)(crc & 0xFFU);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 4U] = ADC_PROTO_EOF0;
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 5U] = ADC_PROTO_EOF1;

    return frame_len;
}

int32_t adc_proto_get_i32_be(const uint8_t *buf,
                             uint16_t *index)
{
    int32_t value;

    if ((NULL == buf) || (NULL == index))
    {
        return 0L;
    }

    value = (int32_t)(((uint32_t)buf[*index] << 24) |
                      ((uint32_t)buf[*index + 1U] << 16) |
                      ((uint32_t)buf[*index + 2U] << 8) |
                      ((uint32_t)buf[*index + 3U]));

    *index = (uint16_t)(*index + 4U);

    return value;
}

void adc_proto_put_u32_be(uint8_t *buf,
                          uint16_t *index,
                          uint32_t value)
{
    if ((NULL == buf) || (NULL == index))
    {
        return;
    }

    buf[(*index)++] = (uint8_t)(value >> 24);
    buf[(*index)++] = (uint8_t)(value >> 16);
    buf[(*index)++] = (uint8_t)(value >> 8);
    buf[(*index)++] = (uint8_t)(value & 0xFFU);
}

void adc_proto_put_u16_be(uint8_t *buf,
                          uint16_t *index,
                          uint16_t value)
{
    if ((NULL == buf) || (NULL == index))
    {
        return;
    }

    buf[(*index)++] = (uint8_t)(value >> 8);
    buf[(*index)++] = (uint8_t)(value & 0xFFU);
}

uint8_t adc_proto_is_fixed_frame_valid(const uint8_t *frame)
{
    uint16_t data_len;
    uint32_t crc_calc;
    uint32_t crc_recv;

    if (NULL == frame)
    {
        return 0U;
    }

    if ((frame[0] != ADC_PROTO_SOF0) ||
        (frame[1] != ADC_PROTO_SOF1))
    {
        return 0U;
    }

    if ((frame[ADC_PROTO_FIXED_EOF_OFFSET] != ADC_PROTO_EOF0) ||
        (frame[ADC_PROTO_FIXED_EOF_OFFSET + 1U] != ADC_PROTO_EOF1))
    {
        return 0U;
    }

    data_len = adc_proto_fixed_data_len(frame);
    if (data_len > ADC_PROTO_FIXED_DATA_CAPACITY)
    {
        return 0U;
    }

    crc_calc = adc_proto_crc32(frame, ADC_PROTO_FIXED_CRC_OFFSET);

    crc_recv = ((uint32_t)frame[ADC_PROTO_FIXED_CRC_OFFSET] << 24) |
               ((uint32_t)frame[ADC_PROTO_FIXED_CRC_OFFSET + 1U] << 16) |
               ((uint32_t)frame[ADC_PROTO_FIXED_CRC_OFFSET + 2U] << 8) |
               frame[ADC_PROTO_FIXED_CRC_OFFSET + 3U];

    return (crc_recv == crc_calc) ? 1U : 0U;
}

uint16_t adc_proto_fixed_data_len(const uint8_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    return adc_proto_payload_len(frame);
}

uint16_t adc_proto_fixed_block_id(const uint8_t *frame)
{
    if (NULL == frame)
    {
        return 0U;
    }

    return (uint16_t)(((uint16_t)frame[ADC_PROTO_FIXED_DATA_OFFSET] << 8) |
                      frame[ADC_PROTO_FIXED_DATA_OFFSET + 1U]);
}

uint16_t adc_proto_build_fixed_frame(uint8_t *frame,
                                     uint16_t frame_buf_size,
                                     uint8_t cmd,
                                     uint16_t block_id,
                                     const uint8_t *data,
                                     uint16_t data_len)
{
    uint16_t index;
    uint32_t crc;

    if (NULL == frame)
    {
        return 0U;
    }

    if (frame_buf_size < ADC_PROTO_FIXED_FRAME_SIZE)
    {
        return 0U;
    }

    if (data_len > ADC_PROTO_FIXED_DATA_CAPACITY)
    {
        return 0U;
    }

    if ((data_len > 0U) && (NULL == data))
    {
        return 0U;
    }

    memset(frame, 0, ADC_PROTO_FIXED_FRAME_SIZE);

    frame[0] = ADC_PROTO_SOF0;
    frame[1] = ADC_PROTO_SOF1;
    frame[2] = cmd;
    frame[3] = (uint8_t)(data_len >> 8);
    frame[4] = (uint8_t)(data_len & 0xFFU);

    index = ADC_PROTO_FIXED_DATA_OFFSET;
    frame[index++] = (uint8_t)(block_id >> 8);
    frame[index++] = (uint8_t)(block_id & 0xFFU);

    if (data_len > 0U)
    {
        memcpy(&frame[index], data, data_len);
    }

    crc = adc_proto_crc32(frame, ADC_PROTO_FIXED_CRC_OFFSET);

    frame[ADC_PROTO_FIXED_CRC_OFFSET] = (uint8_t)(crc >> 24);
    frame[ADC_PROTO_FIXED_CRC_OFFSET + 1U] = (uint8_t)(crc >> 16);
    frame[ADC_PROTO_FIXED_CRC_OFFSET + 2U] = (uint8_t)(crc >> 8);
    frame[ADC_PROTO_FIXED_CRC_OFFSET + 3U] = (uint8_t)(crc & 0xFFU);

    frame[ADC_PROTO_FIXED_EOF_OFFSET] = ADC_PROTO_EOF0;
    frame[ADC_PROTO_FIXED_EOF_OFFSET + 1U] = ADC_PROTO_EOF1;

    return ADC_PROTO_FIXED_FRAME_SIZE;
}
