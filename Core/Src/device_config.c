#include "device_config.h"
#include "eeprom_storage.h"

#include <string.h>

#define DEVICE_CONFIG_EEPROM_ADDR 0x0000U
#define DEVICE_CONFIG_MAGIC 0x41444346UL
#define DEVICE_CONFIG_VERSION 2U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    device_network_config_t network;
    device_adc_cal_config_t adc_cal;
    device_dac_config_t dac;
    uint32_t crc32;
} device_config_record_t;

static device_network_config_t s_network_config;
static device_adc_cal_config_t s_adc_cal_config;
static device_dac_config_t s_dac_config;

static uint32_t device_config_crc32(const uint8_t *data, uint16_t len);

void device_config_init_defaults(void)
{
    uint8_t ch;

    memset(&s_network_config, 0, sizeof(s_network_config));
    memset(&s_adc_cal_config, 0, sizeof(s_adc_cal_config));
    memset(&s_dac_config, 0, sizeof(s_dac_config));

    s_network_config.mac[0] = 0x02U;
    s_network_config.mac[1] = 0x00U;
    s_network_config.mac[2] = 0x00U;
    s_network_config.mac[3] = 0x00U;
    s_network_config.mac[4] = 0x00U;
    s_network_config.mac[5] = 0x21U;

    s_network_config.ip[0] = 192U;
    s_network_config.ip[1] = 168U;
    s_network_config.ip[2] = 1U;
    s_network_config.ip[3] = 21U;

    s_network_config.netmask[0] = 255U;
    s_network_config.netmask[1] = 255U;
    s_network_config.netmask[2] = 255U;
    s_network_config.netmask[3] = 0U;

    s_network_config.gateway[0] = 192U;
    s_network_config.gateway[1] = 168U;
    s_network_config.gateway[2] = 1U;
    s_network_config.gateway[3] = 1U;

    s_network_config.tcp_port = 8080U;

    memcpy(s_network_config.name, "ADDA_COLLECT_1", 14U);

    for (ch = 0U; ch < DEVICE_CONFIG_ADC_CHANNEL_COUNT; ch++)
    {
        s_adc_cal_config.ch[ch].k_raw = DEVICE_CONFIG_ADC_CAL_DEFAULT_K_RAW;
        s_adc_cal_config.ch[ch].b_raw = DEVICE_CONFIG_ADC_CAL_DEFAULT_B_RAW;
    }

    for (ch = 0U; ch < DEVICE_CONFIG_DAC_CHANNEL_COUNT; ch++)
    {
        s_dac_config.ch[ch].mode = DEVICE_CONFIG_DAC_MODE_MANUAL;
        s_dac_config.ch[ch].manual_raw = 0L;
        s_dac_config.ch[ch].adc_channel = DEVICE_CONFIG_DAC_ADC_CH_INVALID;
        s_dac_config.ch[ch].k_raw = DEVICE_CONFIG_DAC_CAL_DEFAULT_K_RAW;
        s_dac_config.ch[ch].b_raw = DEVICE_CONFIG_DAC_CAL_DEFAULT_B_RAW;
    }
}

const device_network_config_t *device_config_get_network(void)
{
    return &s_network_config;
}

void device_config_set_network(const device_network_config_t *config)
{
    if (NULL == config)
    {
        return;
    }

    memcpy(&s_network_config, config, sizeof(s_network_config));
}

const device_adc_cal_config_t *device_config_get_adc_calibration(void)
{
    return &s_adc_cal_config;
}

void device_config_set_adc_calibration(const device_adc_cal_config_t *config)
{
    if (NULL == config)
    {
        return;
    }

    memcpy(&s_adc_cal_config, config, sizeof(s_adc_cal_config));
}

const device_dac_config_t *device_config_get_dac_config(void)
{
    return &s_dac_config;
}

void device_config_set_dac_config(const device_dac_config_t *config)
{
    if (NULL == config)
    {
        return;
    }

    memcpy(&s_dac_config, config, sizeof(s_dac_config));
}

HAL_StatusTypeDef device_config_save_all(void)
{
    device_config_record_t record;
    uint16_t crc_len;

    if (0U == eeprom_storage_is_ready())
    {
        return HAL_ERROR;
    }

    memset(&record, 0, sizeof(record));

    record.magic = DEVICE_CONFIG_MAGIC;
    record.version = DEVICE_CONFIG_VERSION;
    record.length = (uint16_t)(sizeof(record.network) +
                               sizeof(record.adc_cal) +
                               sizeof(record.dac));

    memcpy(&record.network, &s_network_config, sizeof(record.network));
    memcpy(&record.adc_cal, &s_adc_cal_config, sizeof(record.adc_cal));
    memcpy(&record.dac, &s_dac_config, sizeof(record.dac));

    crc_len = (uint16_t)(sizeof(record) - sizeof(record.crc32));
    record.crc32 = device_config_crc32((const uint8_t *)&record, crc_len);

    return eeprom_storage_write(DEVICE_CONFIG_EEPROM_ADDR,
                                (const uint8_t *)&record,
                                sizeof(record));
}

HAL_StatusTypeDef device_config_save_network(void)
{
    return device_config_save_all();
}

static uint32_t device_config_crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if (0UL != (crc & 1UL))
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

HAL_StatusTypeDef device_config_load_all(void)
{
    device_config_record_t record;
    uint16_t crc_len;
    uint32_t crc_calc;

    if (0U == eeprom_storage_is_ready())
    {
        return HAL_ERROR;
    }

    if (HAL_OK != eeprom_storage_read(DEVICE_CONFIG_EEPROM_ADDR,
                                      (uint8_t *)&record,
                                      sizeof(record)))
    {
        return HAL_ERROR;
    }

    if (DEVICE_CONFIG_MAGIC != record.magic)
    {
        return HAL_ERROR;
    }

    if (DEVICE_CONFIG_VERSION != record.version)
    {
        return HAL_ERROR;
    }

    if ((uint16_t)(sizeof(record.network) +
                   sizeof(record.adc_cal) +
                   sizeof(record.dac)) != record.length)
    {
        return HAL_ERROR;
    }

    crc_len = (uint16_t)(sizeof(record) - sizeof(record.crc32));
    crc_calc = device_config_crc32((const uint8_t *)&record, crc_len);

    if (crc_calc != record.crc32)
    {
        return HAL_ERROR;
    }

    memcpy(&s_network_config, &record.network, sizeof(s_network_config));
    memcpy(&s_adc_cal_config, &record.adc_cal, sizeof(s_adc_cal_config));
    memcpy(&s_dac_config, &record.dac, sizeof(s_dac_config));

    return HAL_OK;
}

HAL_StatusTypeDef device_config_load_network(void)
{
    return device_config_load_all();
}
