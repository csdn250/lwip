#ifndef __ADC_PROTO_H
#define __ADC_PROTO_H

#include "main.h"

#define ADC_PROTO_SOF0 0x12U
#define ADC_PROTO_SOF1 0x34U
#define ADC_PROTO_EOF0 0x56U
#define ADC_PROTO_EOF1 0x78U

/** @brief PC 发送给 MCU 的协议命令 */
#define ADC_PROTO_CMD_WRITE_PARAM 0x01U // 写入参数
#define ADC_PROTO_CMD_READ_PARAM 0x02U  // 读取参数
#define ADC_PROTO_CMD_HEARTBEAT 0x07U   // 心跳包

/** @brief MCU 发送给 PC 的响应帧命令（由协议文档预留） */
#define ADC_PROTO_RSP_RAW_DATA 0x81U       // 原始 ADC 数据流
#define ADC_PROTO_RSP_CONVERTED_DATA 0x82U // 转换后的 ADC 数据流
/** @brief 调试状态帧命令 */
#define ADC_PROTO_RSP_DEBUG_STATUS 0x83U

/** @brief 日志诊断命令 */
#define ADC_LOG_DIAG_ACTION_READ_RAM 0x00U           // 读取 RAM 日志快照
#define ADC_LOG_DIAG_ACTION_CLEAR 0x01U              // 清空 RAM 日志
#define ADC_LOG_DIAG_ACTION_READ_PERSIST 0x02U       // 读取 EEPROM 持久日志快照
#define ADC_LOG_DIAG_ACTION_STOP_WATCHDOG_FEED 0xA5U // 停止看门狗喂狗（调试用）

/** @brief ADC 数据流类型定义 */
#define ADC_STREAM_TYPE_RAW ADC_PROTO_RSP_RAW_DATA             // 原始数据
#define ADC_STREAM_TYPE_CONVERTED ADC_PROTO_RSP_CONVERTED_DATA // 转换数据

/* Frame: SOF(2) + CMD(1) + LEN(2) + PAYLOAD(n) + CRC32(4) + EOF(2) */
#define ADC_PROTO_HEADER_SIZE 5U
#define ADC_PROTO_CRC_SIZE 4U
#define ADC_PROTO_EOF_SIZE 2U
#define ADC_PROTO_TAIL_SIZE (ADC_PROTO_CRC_SIZE + ADC_PROTO_EOF_SIZE)
#define ADC_PROTO_FRAME_OVERHEAD (ADC_PROTO_HEADER_SIZE + ADC_PROTO_TAIL_SIZE)
#define ADC_PROTO_MIN_FRAME_SIZE ADC_PROTO_FRAME_OVERHEAD

#define ADC_PROTO_COMMAND_FRAME_SIZE 150U
#define ADC_PROTO_COMMAND_CRC_OFFSET 144U
#define ADC_PROTO_COMMAND_EOF_OFFSET 148U
#define ADC_PROTO_COMMAND_DATA_OFFSET ADC_PROTO_HEADER_SIZE
#define ADC_PROTO_COMMAND_BLOCK_ID_SIZE 2U
#define ADC_PROTO_COMMAND_DATA_CAPACITY \
    (ADC_PROTO_COMMAND_CRC_OFFSET - ADC_PROTO_COMMAND_DATA_OFFSET - ADC_PROTO_COMMAND_BLOCK_ID_SIZE)

uint32_t adc_proto_crc32(const uint8_t *data,
                         uint16_t len);

uint16_t adc_proto_payload_len(const uint8_t *frame);

uint16_t adc_proto_build_stream_frame(uint8_t *frame,
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

uint8_t adc_proto_is_host_command(uint8_t cmd);

uint8_t adc_proto_is_command_frame_valid(const uint8_t *frame);

uint16_t adc_proto_command_data_len(const uint8_t *frame);

uint16_t adc_proto_command_block_id(const uint8_t *frame);

uint16_t adc_proto_build_command_frame(uint8_t *frame,
                                       uint16_t frame_buf_size,
                                       uint8_t cmd,
                                       uint16_t block_id,
                                       const uint8_t *data,
                                       uint16_t data_len);

#endif /* __ADC_PROTO_H */
