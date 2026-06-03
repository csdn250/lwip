#include "udp_discovery.h"

#include "adc_tcp_server.h"
#include "device_config.h"

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"

#include "SEGGER_RTT.h"

#include <string.h>

#define UDP_DISCOVERY_PORT 8081U
#define UDP_DISCOVERY_INTERVAL_MS 5000U

#define UDP_DISCOVERY_CMD_STATUS 0x71U
#define UDP_DISCOVERY_FRAME_MAX 96U
#define UDP_DISCOVERY_PAYLOAD_MAX 64U

#define UDP_DISCOVERY_SOF0 0x12U
#define UDP_DISCOVERY_SOF1 0x34U
#define UDP_DISCOVERY_EOF0 0x56U
#define UDP_DISCOVERY_EOF1 0x78U

#define UDP_DISCOVERY_STATUS_TCP_CONNECTED 0x01U
#define UDP_DISCOVERY_STATUS_NET_DIRTY 0x02U

static struct udp_pcb *s_udp_pcb;
static uint32_t s_last_broadcast_tick;

static uint32_t udp_discovery_crc32(const uint8_t *data,
                                    uint16_t len);
static uint16_t udp_discovery_build_payload(uint8_t *payload,
                                            uint16_t payload_size);
static uint16_t udp_discovery_build_frame(uint8_t *frame,
                                          uint16_t frame_size);
static void udp_discovery_send_broadcast(void);

void udp_discovery_init(void)
{
    s_udp_pcb = udp_new();

    if (NULL == s_udp_pcb)
    {
        SEGGER_RTT_WriteString(0, "udp discovery new failed\r\n");
        return;
    }

    ip_set_option(s_udp_pcb, SOF_BROADCAST);

    if (ERR_OK != udp_bind(s_udp_pcb, IP_ADDR_ANY, UDP_DISCOVERY_PORT))
    {
        SEGGER_RTT_WriteString(0, "udp discovery bind failed\r\n");
        udp_remove(s_udp_pcb);
        return;
    }

    s_last_broadcast_tick = 0U;
    SEGGER_RTT_WriteString(0, "udp discovery init\r\n");
}

void udp_discovery_process(void)
{
    uint32_t now_tick;

    if (NULL == s_udp_pcb)
    {
        return;
    }

    if (0U != adc_tcp_server_has_client())
    {
        return;
    }

    now_tick = HAL_GetTick();

    if ((now_tick - s_last_broadcast_tick) < UDP_DISCOVERY_INTERVAL_MS)
    {
        return;
    }

    s_last_broadcast_tick = HAL_GetTick();

    udp_discovery_send_broadcast();
}

static uint32_t udp_discovery_crc32(const uint8_t *data, uint16_t len)
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

static uint16_t udp_discovery_build_payload(uint8_t *payload, uint16_t payload_size)
{
    const device_network_config_t *net_cfg;
    uint16_t index = 0U;

    if (58U > payload_size)
    {
        return 0U;
    }

    net_cfg = device_config_get_network();

    payload[index++] = 'A';
    payload[index++] = 'D';
    payload[index++] = 'D';
    payload[index++] = 'A';
    payload[index++] = 1U;

    payload[index] = 0U;

    if (0U != adc_tcp_server_has_client())
    {
        payload[index] |= UDP_DISCOVERY_STATUS_TCP_CONNECTED;
    }

    if (0U != adc_tcp_server_is_network_config_dirty())
    {
        payload[index] |= UDP_DISCOVERY_STATUS_NET_DIRTY;
    }

    index++;

    memcpy(&payload[index], net_cfg->mac, sizeof(net_cfg->mac));
    index += sizeof(net_cfg->mac);

    memcpy(&payload[index], net_cfg->ip, sizeof(net_cfg->ip));
    index += sizeof(net_cfg->ip);

    memcpy(&payload[index], net_cfg->netmask, sizeof(net_cfg->netmask));
    index += sizeof(net_cfg->netmask);

    memcpy(&payload[index], net_cfg->gateway, sizeof(net_cfg->gateway));
    index += sizeof(net_cfg->gateway);

    payload[index++] = (uint8_t)(net_cfg->tcp_port >> 8);
    payload[index++] = (uint8_t)(net_cfg->tcp_port & 0xFFU);

    memcpy(&payload[index], net_cfg->name, DEVICE_CONFIG_NAME_MAX_LEN);
    index += DEVICE_CONFIG_NAME_MAX_LEN;

    return index;
}

static uint16_t udp_discovery_build_frame(uint8_t *frame, uint16_t frame_size)
{
    uint8_t payload[UDP_DISCOVERY_PAYLOAD_MAX];
    uint16_t payload_len;
    uint16_t frame_len;
    uint16_t crc_pos;
    uint32_t crc;

    payload_len = udp_discovery_build_payload(payload, sizeof(payload));
    if (payload_len == 0U)
    {
        return 0U;
    }

    frame_len = (uint16_t)(5U + payload_len + 4U + 2U);
    if (frame_size < frame_len)
    {
        return 0U;
    }

    frame[0] = UDP_DISCOVERY_SOF0;
    frame[1] = UDP_DISCOVERY_SOF1;
    frame[2] = UDP_DISCOVERY_CMD_STATUS;
    frame[3] = (uint8_t)(payload_len >> 8);
    frame[4] = (uint8_t)(payload_len & 0xFFU);

    memcpy(&frame[5], payload, payload_len);

    crc = udp_discovery_crc32(frame, (uint16_t)(5U + payload_len));
    crc_pos = (uint16_t)(5U + payload_len);

    frame[crc_pos] = (uint8_t)(crc >> 24);
    frame[crc_pos + 1U] = (uint8_t)(crc >> 16);
    frame[crc_pos + 2U] = (uint8_t)(crc >> 8);
    frame[crc_pos + 3U] = (uint8_t)(crc & 0xFFU);

    frame[crc_pos + 4U] = UDP_DISCOVERY_EOF0;
    frame[crc_pos + 5U] = UDP_DISCOVERY_EOF1;

    return frame_len;
}

static void udp_discovery_send_broadcast(void)
{
    uint8_t frame[UDP_DISCOVERY_FRAME_MAX];
    uint16_t frame_len;
    struct pbuf *p;
    ip_addr_t broadcast_ip;

    frame_len = udp_discovery_build_frame(frame, sizeof(frame));
    if (0U == frame_len)
    {
        SEGGER_RTT_WriteString(0, "udp discovery build failed\r\n");
        return;
    }

    p = pbuf_alloc(PBUF_TRANSPORT, frame_len, PBUF_RAM);
    if (NULL == p)
    {
        SEGGER_RTT_WriteString(0, "udp discovery pbuf failed\r\n");
        return;
    }

    memcpy(p->payload, frame, frame_len);

    IP4_ADDR(&broadcast_ip, 255, 255, 255, 255);

    if (ERR_OK == udp_sendto(s_udp_pcb,
                             p,
                             &broadcast_ip,
                             UDP_DISCOVERY_PORT))
    {
        SEGGER_RTT_WriteString(0, "udp discovery broadcast\r\n");
    }
    else
    {
        SEGGER_RTT_WriteString(0, "udp discovery send failed\r\n");
    }

    pbuf_free(p);
}
