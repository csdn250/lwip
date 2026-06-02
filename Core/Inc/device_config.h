#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

#include "main.h"

#define DEVICE_CONFIG_NAME_MAX_LEN 32U

typedef struct
{
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint16_t tcp_port;
    char name[DEVICE_CONFIG_NAME_MAX_LEN];
} device_network_config_t;

void device_config_init_defaults(void);

const device_network_config_t *device_config_get_network(void);

void device_config_set_network(const device_network_config_t *config);

#endif /* __DEVICE_CONFIG_H */

