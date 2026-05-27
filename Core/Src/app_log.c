#define LOG_TAG "BOOT"
#define LOG_LVL ELOG_LVL_INFO

#include "app_log.h"
#include "SEGGER_RTT.h"
#include "elog.h"

void app_log_init(void)
{
    SEGGER_RTT_Init();

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

    log_i("event=%lu %s", (unsigned long)event, message);
}
