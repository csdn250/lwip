#ifndef __ADC_FRAME_BUILDER_H
#define __ADC_FRAME_BUILDER_H

#include "main.h"
#include "adc_acq_service.h"

#define ADC_FRAME_MAGIC 0x31434441U
#define ADC_FRAME_MAX_CHANNEL_COUNT ADC_ACQ_CHANNEL_COUNT
#define ADC_FRAME_MAX_PAYLOAD_BYTES (ADC_FRAME_MAX_CHANNEL_COUNT * sizeof(float))
#define ADC_FRAME_HEADER_BYTES ((uint16_t)sizeof(adc_frame_header_t))
#define ADC_FRAME_MAX_BYTES (ADC_FRAME_HEADER_BYTES + ADC_FRAME_MAX_PAYLOAD_BYTES)

typedef enum
{
    ADC_FRAME_FORMAT_RAW_FLOAT = 0x81U,
    ADC_FRAME_FORMAT_CAL_FLOAT = 0x82U
}adc_frame_format_t;

/*
magic          找帧头
seq            判断丢帧/乱序
timestamp_us   采样时间
channel_mask   哪些通道开启
channel_count  payload 里有几个通道数据
sample_format  数据格式，raw float 或 calibrated float
payload_bytes  payload 字节数
*/
#pragma pack(push,1)
typedef struct 
{
    uint32_t magic;
    uint32_t seq;
    uint32_t timestamp_us;
    uint16_t channel_mask;
    uint16_t channel_count;
    uint16_t sample_format;
    uint16_t payload_bytes;
}adc_frame_header_t;
#pragma pack(pop)

uint16_t adc_frame_builder_build_raw_float(const adc_acq_sample_t *sample,
                                            uint16_t channel_mask,
                                            uint8_t *out_buf,
                                            uint16_t out_buf_size);


#endif //end of __ADC_FRAME_BUILDER_H

