#ifndef __APP_LOG_H
#define __APP_LOG_H

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
} app_log_event_t;

void app_log_init(void);
void app_log_key_event(app_log_event_t event,
                       const char *message);

#endif // end of __APP_LOG_H
