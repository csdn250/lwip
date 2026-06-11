#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

#include "main.h"

#define DEVICE_CONFIG_NAME_MAX_LEN 32U

#define DEVICE_CONFIG_ADC_CHANNEL_COUNT 12U

#define DEVICE_CONFIG_DAC_CHANNEL_COUNT 4U
#define DEVICE_CONFIG_DAC_MODE_MANUAL 0U
#define DEVICE_CONFIG_DAC_MODE_ADC_CASCADE 1U
#define DEVICE_CONFIG_DAC_ADC_CH_INVALID 0xFFU

typedef struct
{
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint16_t tcp_port;
    char name[DEVICE_CONFIG_NAME_MAX_LEN];
} device_network_config_t;

typedef struct
{
    float k;
    float b;
} device_adc_cal_channel_t;

typedef struct
{
    device_adc_cal_channel_t ch[DEVICE_CONFIG_ADC_CHANNEL_COUNT];
} device_adc_cal_config_t;

typedef struct
{
    uint8_t mode;
    float manual_voltage;
    uint8_t adc_channel;
    float k;
    float b;
} device_dac_channel_config_t;

typedef struct
{
    device_dac_channel_config_t ch[DEVICE_CONFIG_DAC_CHANNEL_COUNT];
} device_dac_config_t;

void device_config_init_defaults(void);

const device_network_config_t *device_config_get_network(void);

void device_config_set_network(const device_network_config_t *network_config);

HAL_StatusTypeDef device_config_save_all(void);
HAL_StatusTypeDef device_config_load_all(void);

/*
 * 延迟保存机制
 * ------------
 * device_config_save_all() 内部是阻塞式 I2C 写（每页都要等 EEPROM ready，
 * 最坏跨多页时阻塞数百毫秒）。若在 lwIP recv 回调里直接调用，会阻塞
 * MX_LWIP_Process() 并影响看门狗喂狗。
 *
 * 因此参数应用路径只调用 device_config_request_save() 置脏标记，真正的
 * EEPROM 写由主循环空闲点调用 device_config_process_save() 完成（安全点，
 * 不在任何 lwIP 回调上下文中）。
 */
void device_config_request_save(void);
uint8_t device_config_is_save_pending(void);
void device_config_process_save(void);

const device_adc_cal_config_t *device_config_get_adc_calibration(void);

void device_config_set_adc_calibration(const device_adc_cal_config_t *adc_cal_config);

const device_dac_config_t *device_config_get_dac_config(void);

void device_config_set_dac_config(const device_dac_config_t *dac_config);

#endif /* __DEVICE_CONFIG_H */
