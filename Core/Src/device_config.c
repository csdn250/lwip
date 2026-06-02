#include "device_config.h"

#include <string.h>

static device_network_config_t s_network_config;

void device_config_init_defaults(void)
{
    memset(&s_network_config, 0, sizeof(s_network_config));

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

