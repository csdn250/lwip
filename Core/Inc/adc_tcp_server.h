#ifndef __ADC_TCP_SERVER_H
#define __ADC_TCP_SERVER_H

#include "main.h"

/* TCP server public API */
void adc_tcp_server_init(void);
void adc_tcp_server_process(void);

uint8_t adc_tcp_server_has_client(void);
uint8_t adc_tcp_server_is_streaming(void);
uint8_t adc_tcp_server_is_network_config_dirty(void);
uint8_t adc_tcp_server_is_watchdog_feed_enabled(void);

#endif /* __ADC_TCP_SERVER_H */
