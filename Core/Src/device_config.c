#include "device_config.h"
#include "eeprom_storage.h"
#include "app_log.h"

#include <string.h>

#define DEVICE_CONFIG_EEPROM_ADDR 0x0000U
#define DEVICE_CONFIG_MAGIC 0x41444346UL
#define DEVICE_CONFIG_VERSION 3U

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

/*
 * 延迟保存脏标记：1 = 有配置改动待写入 EEPROM。
 * 由 device_config_request_save() 置位（参数应用路径调用，非阻塞），
 * 由 device_config_process_save() 在主循环空闲点清位并完成实际写入。
 */
static uint8_t s_config_save_pending;

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
        s_dac_config.ch[ch].manual_voltage = 0.0f;
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

/**
 * @brief 请求延迟保存配置
 * @details 仅置脏标记，立即返回，不触发任何阻塞式 EEPROM 写。
 *          真正的写由主循环 device_config_process_save() 完成。
 * @retval None
 *
 * @note 参数应用路径（cal/dac/network/name）应调用本函数，避免在
 *       lwIP recv 回调里同步写 EEPROM 而阻塞协议栈与看门狗喂狗。
 */
void device_config_request_save(void)
{
    s_config_save_pending = 1U;
}

/**
 * @brief 查询是否有待保存的配置改动
 * @retval uint8_t 1 = 有待保存改动，0 = 无
 */
uint8_t device_config_is_save_pending(void)
{
    return s_config_save_pending;
}

/**
 * @brief 处理延迟保存（在主循环空闲点调用）
 * @details 若存在待保存改动，则执行一次合并后的完整记录写入。
 *          无论成功失败都会清除脏标记，避免反复重试持续阻塞主循环。
 * @retval None
 *
 * @note 此处仍是阻塞式 I2C 写，但已从 lwIP 回调上下文挪到主循环安全点，
 *       且多次连续改动会合并为一次写入。
 */
void device_config_process_save(void)
{
    HAL_StatusTypeDef status;

    if (0U == s_config_save_pending)
    {
        return;
    }

    /* 先清标记：即使本次写失败也不在主循环里反复阻塞重试 */
    s_config_save_pending = 0U;

    status = device_config_save_all();

    if (HAL_OK == status)
    {
        app_log_record(APP_LOG_EVENT_CONFIG_SAVED, 0U, 0U, 0U);
    }
    else
    {
        app_log_record(APP_LOG_EVENT_CONFIG_SAVE_FAILED,
                       (uint16_t)status,
                       0U,
                       0U);
    }
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
