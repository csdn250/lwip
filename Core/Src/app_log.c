#define LOG_TAG "BOOT"
#define LOG_LVL ELOG_LVL_INFO

#include "app_log.h"
#include "SEGGER_RTT.h"
#include "elog.h"
#include <string.h>

static app_log_record_t s_log_records[APP_LOG_RECORD_CAPACITY];
static uint16_t s_log_write_index;
static uint16_t s_log_count;

void app_log_init(void)
{
    SEGGER_RTT_Init();
    app_log_clear();

    elog_init();
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_start();

    log_i("logger started");
}

void app_log_key_event(app_log_event_t event,
                       const char *message)
{
    if (NULL == message)
    {
        message = "";
    }

    app_log_record(event, 0U, 0U, 0U);
    log_i("event=%lu %s", (unsigned long)event, message);
}

void app_log_record(app_log_event_t event,
                    uint16_t arg0,
                    uint32_t arg1,
                    uint32_t arg2)
{
    app_log_record_t *record;

    record = &s_log_records[s_log_write_index];

    record->tick_ms = HAL_GetTick();
    record->event = (uint16_t)event;
    record->arg0 = arg0;
    record->arg1 = arg1;
    record->arg2 = arg2;

    s_log_write_index++;
    if (s_log_write_index >= APP_LOG_RECORD_CAPACITY)
    {
        s_log_write_index = 0U;
    }

    if (s_log_count < APP_LOG_RECORD_CAPACITY)
    {
        s_log_count++;
    }
}

uint16_t app_log_snapshot(app_log_record_t *out,
                          uint16_t max_count)
{
    uint16_t copy_count;
    uint16_t start;
    uint16_t i;
    uint16_t index;

    if ((NULL == out) || (0U == max_count))
    {
        return 0U;
    }

    copy_count = s_log_count;
    if (copy_count > max_count)
    {
        copy_count = max_count;
    }

    start = (s_log_write_index + APP_LOG_RECORD_CAPACITY - copy_count) %
            APP_LOG_RECORD_CAPACITY;

    for (i = 0U; i < copy_count; i++)
    {
        index = (start + i) % APP_LOG_RECORD_CAPACITY;
        out[i] = s_log_records[index];
    }

    return copy_count;
}

void app_log_clear(void)
{
    memset(s_log_records, 0, sizeof(s_log_records));
    s_log_write_index = 0U;
    s_log_count = 0U;
}