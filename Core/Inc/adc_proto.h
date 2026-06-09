#ifndef __ADC_PROTO_H
#define __ADC_PROTO_H

#include "main.h"

#define ADC_PROTO_SOF0 0x12U
#define ADC_PROTO_SOF1 0x34U
#define ADC_PROTO_EOF0 0x56U
#define ADC_PROTO_EOF1 0x78U

/* Frame: SOF(2) + CMD(1) + LEN(2) + PAYLOAD(n) + CRC32(4) + EOF(2) */
#define ADC_PROTO_HEADER_SIZE 5U
#define ADC_PROTO_CRC_SIZE 4U
#define ADC_PROTO_EOF_SIZE 2U
#define ADC_PROTO_TAIL_SIZE (ADC_PROTO_CRC_SIZE + ADC_PROTO_EOF_SIZE)
#define ADC_PROTO_FRAME_OVERHEAD (ADC_PROTO_HEADER_SIZE + ADC_PROTO_TAIL_SIZE)
#define ADC_PROTO_MIN_FRAME_SIZE ADC_PROTO_FRAME_OVERHEAD

#define ADC_PROTO_FIXED_FRAME_SIZE 150U
#define ADC_PROTO_FIXED_CRC_OFFSET 144U
#define ADC_PROTO_FIXED_EOF_OFFSET 148U
#define ADC_PROTO_FIXED_DATA_OFFSET ADC_PROTO_HEADER_SIZE
#define ADC_PROTO_FIXED_BLOCK_ID_SIZE 2U
#define ADC_PROTO_FIXED_DATA_CAPACITY \
    (ADC_PROTO_FIXED_CRC_OFFSET - ADC_PROTO_FIXED_DATA_OFFSET - ADC_PROTO_FIXED_BLOCK_ID_SIZE)

uint32_t adc_proto_crc32(const uint8_t *data,
                         uint16_t len);

uint16_t adc_proto_payload_len(const uint8_t *frame);

uint8_t adc_proto_is_frame_valid(const uint8_t *frame,
                                 uint16_t frame_len,
                                 uint16_t max_payload_len);

uint16_t adc_proto_build_frame(uint8_t *frame,
                               uint16_t frame_buf_size,
                               uint8_t cmd,
                               const uint8_t *payload,
                               uint16_t payload_len);

int32_t adc_proto_get_i32_be(const uint8_t *buf,
                             uint16_t *index);

float adc_proto_get_float_be(const uint8_t *buf,
                             uint16_t *index);

void adc_proto_put_float_be(uint8_t *buf,
                            uint16_t *index,
                            float value);

void adc_proto_put_u32_be(uint8_t *buf,
                          uint16_t *index,
                          uint32_t value);

void adc_proto_put_u16_be(uint8_t *buf,
                          uint16_t *index,
                          uint16_t value);

uint8_t adc_proto_is_fixed_frame_valid(const uint8_t *frame);

uint16_t adc_proto_fixed_data_len(const uint8_t *frame);

uint16_t adc_proto_fixed_block_id(const uint8_t *frame);

uint16_t adc_proto_build_fixed_frame(uint8_t *frame,
                                     uint16_t frame_buf_size,
                                     uint8_t cmd,
                                     uint16_t block_id,
                                     const uint8_t *data,
                                     uint16_t data_len);

#endif /* __ADC_PROTO_H */
