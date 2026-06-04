#ifndef __ADC_FRAME_BUILDER_H
#define __ADC_FRAME_BUILDER_H

#include "main.h"
#include "adc_acq_service.h"
#include "device_config.h"

#define ADC_FRAME_MAGIC 0x41444331U
#define ADC_FRAME_MAX_CHANNEL_COUNT ADC_ACQ_CHANNEL_COUNT

/* One TCP data payload can contain several 12-channel sample groups. */
#define ADC_FRAME_BATCH_GROUP_COUNT 32U
#define ADC_FRAME_CONVERTED_BATCH_GROUP_COUNT 16U

#define ADC_FRAME_MAX_RAW_PAYLOAD_BYTES \
    (ADC_FRAME_BATCH_GROUP_COUNT * ADC_FRAME_MAX_CHANNEL_COUNT * sizeof(uint16_t))

#define ADC_FRAME_MAX_CONVERTED_PAYLOAD_BYTES \
    (ADC_FRAME_CONVERTED_BATCH_GROUP_COUNT * ADC_FRAME_MAX_CHANNEL_COUNT * sizeof(float))

#define ADC_FRAME_MAX_PAYLOAD_BYTES ADC_FRAME_MAX_RAW_PAYLOAD_BYTES

#define ADC_FRAME_HEADER_BYTES ((uint16_t)sizeof(adc_frame_header_t))
#define ADC_FRAME_MAX_BYTES (ADC_FRAME_HEADER_BYTES + ADC_FRAME_MAX_PAYLOAD_BYTES)

typedef enum
{
    ADC_FRAME_FORMAT_RAW_U16 = 0x81U,
    ADC_FRAME_FORMAT_CAL_FLOAT = 0x82U
} adc_frame_format_t;

/*
 * ADC stream payload header.
 *
 * This header lives inside the outer TCP protocol payload:
 *   12 34 CMD LEN PAYLOAD CRC32 56 78
 *
 * All multi-byte fields in this payload are big-endian.
 *
 * magic         Fixed ADC payload marker.
 * seq           First sample sequence number in this payload.
 * timestamp_us  Currently mirrors the sample sequence for debug.
 * channel_mask  Bit mask of enabled channels. 0x0FFF means CH1..CH12.
 * channel_count Number of channels in each sample group.
 * sample_format 0x81 raw uint16, 0x82 calibrated float.
 * payload_bytes Bytes after this header.
 */

#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;
    uint32_t seq;
    uint32_t timestamp_us;
    uint16_t channel_mask;
    uint16_t channel_count;
    uint16_t sample_format;
    uint16_t payload_bytes;
} adc_frame_header_t;
#pragma pack(pop)

uint16_t adc_frame_builder_build_raw_u16_batch(const adc_acq_sample_t *samples,
                                               uint16_t sample_count,
                                               uint32_t first_seq,
                                               uint16_t channel_mask,
                                               uint8_t *out_buf,
                                               uint16_t out_buf_size);

uint16_t adc_frame_builder_build_cal_float_batch(const adc_acq_sample_t *samples,
                                                 uint16_t sample_count,
                                                 uint32_t first_seq,
                                                 uint16_t channel_mask,
                                                 const device_adc_cal_config_t *cal_config,
                                                 uint8_t *out_buf,
                                                 uint16_t out_buf_size);

#endif /* __ADC_FRAME_BUILDER_H */
