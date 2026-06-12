#include "app_log_persist.h"
#include "eeprom_storage.h"

#include <string.h>

#define APP_LOG_PERSIST_EEPROM_ADDR 0x0400U
#define APP_LOG_PERSIST_MAGIC 0x4C4F4731UL /* "LOG1" */
#define APP_LOG_PERSIST_VERSION 1U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint16_t count;
    uint16_t reserved;
    app_log_record_t records[APP_LOG_PERSIST_MAX_RECORDS];
    uint32_t crc32;
} app_log_persist_record_t;

static uint8_t s_persist_save_pending;

static uint32_t app_log_persist_crc32(const uint8_t *data,
                                      uint16_t len);

void app_log_persist_init(void)
{
    s_persist_save_pending = 0U;
}

void app_log_persist_request_save(void)
{
    s_persist_save_pending = 1U;
}

void app_log_persist_process(void)
{
    app_log_persist_record_t record;
    uint16_t crc_len;

    if (0U == s_persist_save_pending)
    {
        return;
    }

    s_persist_save_pending = 0U;

    if (0U == eeprom_storage_is_ready())
    {
        return;
    }

    memset(&record, 0, sizeof(record));

    record.magic = APP_LOG_PERSIST_MAGIC;
    record.version = APP_LOG_PERSIST_VERSION;
    record.record_size = (uint16_t)sizeof(app_log_record_t);
    record.count = app_log_snapshot(record.records,
                                    APP_LOG_PERSIST_MAX_RECORDS);

    crc_len = (uint16_t)(sizeof(record) - sizeof(record.crc32));
    record.crc32 = app_log_persist_crc32((const uint8_t *)&record,
                                         crc_len);

    (void)eeprom_storage_write(APP_LOG_PERSIST_EEPROM_ADDR,
                               (const uint8_t *)&record,
                               sizeof(record));
}

uint16_t app_log_persist_load(app_log_record_t *out,
                              uint16_t max_count)
{
    app_log_persist_record_t record;
    uint16_t crc_len;
    uint32_t crc;
    uint16_t copy_count;

    if ((NULL == out) || (0U == max_count))
    {
        return 0U;
    }

    if (HAL_OK != eeprom_storage_read(APP_LOG_PERSIST_EEPROM_ADDR,
                                      (uint8_t *)&record,
                                      sizeof(record)))
    {
        return 0U;
    }

    crc_len = (uint16_t)(sizeof(record) - sizeof(record.crc32));
    crc = app_log_persist_crc32((const uint8_t *)&record,
                                crc_len);

    if ((APP_LOG_PERSIST_MAGIC != record.magic) ||
        (APP_LOG_PERSIST_VERSION != record.version) ||
        (sizeof(app_log_record_t) != record.record_size) ||
        (record.crc32 != crc))
    {
        return 0U;
    }

    copy_count = record.count;
    if (copy_count > APP_LOG_PERSIST_MAX_RECORDS)
    {
        copy_count = APP_LOG_PERSIST_MAX_RECORDS;
    }

    if (copy_count > max_count)
    {
        copy_count = max_count;
    }

    memcpy(out, record.records, copy_count * sizeof(app_log_record_t));

    return copy_count;
}

void app_log_persist_clear(void)
{
    app_log_persist_record_t record;
    uint16_t crc_len;

    memset(&record, 0, sizeof(record));

    record.magic = APP_LOG_PERSIST_MAGIC;
    record.version = APP_LOG_PERSIST_VERSION;
    record.record_size = (uint16_t)sizeof(app_log_record_t);
    record.count = 0U;

    crc_len = (uint16_t)(sizeof(record) - sizeof(record.crc32));
    record.crc32 = app_log_persist_crc32((const uint8_t *)&record,
                                         crc_len);

    (void)eeprom_storage_write(APP_LOG_PERSIST_EEPROM_ADDR,
                               (const uint8_t *)&record,
                               sizeof(record));
}

static uint32_t app_log_persist_crc32(const uint8_t *data,
                                      uint16_t len)
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

