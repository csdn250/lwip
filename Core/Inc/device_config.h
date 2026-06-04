#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

#include "main.h"

#define DEVICE_CONFIG_NAME_MAX_LEN 32U

#define DEVICE_CONFIG_ADC_CHANNEL_COUNT 12U
#define DEVICE_CONFIG_ADC_CAL_DEFAULT_K_RAW 100000000L
#define DEVICE_CONFIG_ADC_CAL_DEFAULT_B_RAW 0L
#define DEVICE_CONFIG_ADC_CAL_K_SCALE 0.00000001f

#define DEVICE_CONFIG_DAC_CHANNEL_COUNT 4U
#define DEVICE_CONFIG_DAC_MODE_MANUAL 0U
#define DEVICE_CONFIG_DAC_MODE_ADC_CASCADE 1U
#define DEVICE_CONFIG_DAC_ADC_CH_INVALID 0xFFU
#define DEVICE_CONFIG_DAC_CAL_DEFAULT_K_RAW 100000000L
#define DEVICE_CONFIG_DAC_CAL_DEFAULT_B_RAW 0L
#define DEVICE_CONFIG_DAC_CAL_K_SCALE 0.00000001f

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
    int32_t k_raw;
    int32_t b_raw;
} device_adc_cal_channel_t;

typedef struct
{
    device_adc_cal_channel_t ch[DEVICE_CONFIG_ADC_CHANNEL_COUNT];
} device_adc_cal_config_t;

typedef struct
{
    uint8_t mode;
    int32_t manual_raw;
    uint8_t adc_channel;
    int32_t k_raw;
    int32_t b_raw;
} device_dac_channel_config_t;

typedef struct
{
    device_dac_channel_config_t ch[DEVICE_CONFIG_DAC_CHANNEL_COUNT];
} device_dac_config_t;

void device_config_init_defaults(void);

const device_network_config_t *device_config_get_network(void);

void device_config_set_network(const device_network_config_t *config);

HAL_StatusTypeDef device_config_save_network(void);

HAL_StatusTypeDef device_config_load_network(void);

const device_adc_cal_config_t *device_config_get_adc_calibration(void);

void device_config_set_adc_calibration(const device_adc_cal_config_t *config);

const device_dac_config_t *device_config_get_dac_config(void);

void device_config_set_dac_config(const device_dac_config_t *config);

#endif /* __DEVICE_CONFIG_H */
