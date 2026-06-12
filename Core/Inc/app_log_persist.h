#ifndef __APP_LOG_PERSIST_H
#define __APP_LOG_PERSIST_H

#include "main.h"
#include "app_log.h"

#define APP_LOG_PERSIST_MAX_RECORDS 16U

void app_log_persist_init(void);

void app_log_persist_request_save(void);

void app_log_persist_process(void);

uint16_t app_log_persist_load(app_log_record_t *out,
                              uint16_t max_cout);

void app_log_persist_clear(void);

#endif /*__APP_LOG_PERSIST_H*/
