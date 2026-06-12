#ifndef __ADC_PARAM_STORE_H
#define __ADC_PARAM_STORE_H

#include "main.h"

#include <stdint.h>

#define ADC_PROTO_WRITE_STATUS_OK 0x00U
#define ADC_PROTO_WRITE_STATUS_SAVE_PENDING 0x01U
#define ADC_PROTO_WRITE_STATUS_BAD_LEN 0x02U
#define ADC_PROTO_WRITE_STATUS_NOT_FOUND 0x03U
#define ADC_PROTO_WRITE_STATUS_TOO_LONG 0x04U

#define ADC_PARAM_BLOCK_CAL_DATA 0x0001U
#define ADC_PARAM_BLOCK_CONTROL 0x0002U
#define ADC_PARAM_BLOCK_CONFIG 0x0003U
#define ADC_PARAM_BLOCK_IP_ADDR 0x0004U
#define ADC_PARAM_BLOCK_MAC_ADDR 0x0005U
#define ADC_PARAM_BLOCK_PORT 0x0006U
#define ADC_PARAM_BLOCK_NETMASK 0x0007U
#define ADC_PARAM_BLOCK_GATEWAY 0x0008U
#define ADC_PARAM_BLOCK_DA_CH1 0x0009U
#define ADC_PARAM_BLOCK_DA_CH2 0x000AU
#define ADC_PARAM_BLOCK_DA_CH3 0x000BU
#define ADC_PARAM_BLOCK_DA_CH4 0x000CU

#define ADC_PARAM_BLOCK_LOG_SNAPSHOT 0x000DU
#define ADC_PARAM_BLOCK_DAC_STATUS 0x000EU
#define ADC_PARAM_BLOCK_DEVICE_NAME 0x000FU
#define ADC_PARAM_BLOCK_DAC_CAL 0x0010U

typedef struct
{
    uint16_t block_id;
    uint8_t *data;
    uint16_t len;
    uint16_t max_len;
} adc_param_block_t;

void adc_param_store_init(void);

adc_param_block_t *adc_param_store_find_block(uint16_t block_id);

void adc_param_store_sync_from_config(void);
uint8_t adc_param_store_check_write_len(uint16_t block_id,
                                        uint16_t len);

/*
 * Apply IP/MAC/PORT/NETMASK/GATEWAY blocks to device_config.
 * Mark network config dirty so UDP discovery can broadcast new values.
 */
uint8_t adc_param_store_apply_cal_param(const uint8_t *data,
                                        uint16_t len);

uint8_t adc_param_store_apply_dac_param(uint16_t block_id,
                                        const uint8_t *data,
                                        uint16_t len);

uint8_t adc_param_store_apply_dac_cal_param(const uint8_t *data,
                                            uint16_t len);

uint8_t adc_param_store_apply_network_param(uint16_t block_id,
                                            const uint8_t *data,
                                            uint16_t len);

/*
 * Apply DEVICE_NAME block: copy raw bytes into device_config network name
 * (truncated to DEVICE_CONFIG_NAME_MAX_LEN-1 chars, NUL-terminated) and
 * request a deferred EEPROM save.
 */
uint8_t adc_param_store_apply_name_param(const uint8_t *data,
                                         uint16_t len);

uint8_t adc_param_store_is_network_dirty(void);

void adc_param_store_clear_network_dirty(void);

#endif /*__ADC_PARAM_STORE_H*/
