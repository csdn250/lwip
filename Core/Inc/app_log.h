#ifndef __APP_LOG_H
#define __APP_LOG_H

#include "main.h"

typedef enum
{
    APP_LOG_EVENT_LOGGER_STARTED = 1,
    APP_LOG_EVENT_PERIPHERALS_READY,
    APP_LOG_EVENT_LWIP_READY,
    APP_LOG_EVENT_ETH_LINK_UP,
    APP_LOG_EVENT_ETH_LINK_DOWN,
    APP_LOG_EVENT_ADC_DMA_STARTED,
    APP_LOG_EVENT_ADC_DMA_ERROR,
    APP_LOG_EVENT_SPI_DAC_ERROR,
    APP_LOG_EVENT_RS485_ERROR,
    APP_LOG_EVENT_I2C_EEPROM_ERROR,

    APP_LOG_EVENT_TCP_ACCEPT,
    APP_LOG_EVENT_TCP_CLOSE,
    APP_LOG_EVENT_TCP_ERROR,
    APP_LOG_EVENT_TCP_RX_OVERFLOW,
    APP_LOG_EVENT_TCP_BAD_FRAME,

    APP_LOG_EVENT_PROTO_HEARTBEAT,
    APP_LOG_EVENT_PROTO_READ_PARAM,
    APP_LOG_EVENT_PROTO_WRITE_PARAM,

    APP_LOG_EVENT_PARAM_WRITE_RESULT,
    APP_LOG_EVENT_NETIF_APPLIED,

    APP_LOG_EVENT_ADC_STREAM_START,
    APP_LOG_EVENT_ADC_STREAM_STOP,
    APP_LOG_EVENT_ADC_STREAM_SEND_FAIL,

    APP_LOG_EVENT_DAC_PARAM_APPLIED,
    APP_LOG_EVENT_DAC_OUTPUT_LIMIT,
    APP_LOG_EVENT_LOG_CLEARED,
} app_log_event_t;

#define APP_LOG_RECORD_CAPACITY 64U

typedef struct
{
    uint32_t tick_ms;
    uint16_t event;
    uint16_t arg0;
    uint32_t arg1;
    uint32_t arg2;
} app_log_record_t;



void app_log_init(void);
void app_log_key_event(app_log_event_t event,
                       const char *message);
void app_log_record(app_log_event_t event,
                    uint16_t arg0,
                    uint32_t arg1,
                    uint32_t arg2);

uint16_t app_log_snapshot(app_log_record_t *out,
                          uint16_t max_count);

void app_log_clear(void);

#endif // end of __APP_LOG_H
