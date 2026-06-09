#include "adc_param_store.h"
#include "device_config.h"
#include "adc_proto.h"
#include "SEGGER_RTT.h"

#include <string.h>

#define ADC_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define ADC_CAL_PARAM_BYTES (DEVICE_CONFIG_ADC_CHANNEL_COUNT * 2U * sizeof(int32_t))
#define DAC_PARAM_BYTES 14U

static uint8_t s_param_cal_data[128];
static uint8_t s_param_control[8];
static uint8_t s_param_config[4] = {1U, 0U, 0x05U, 0xB4U};
static uint8_t s_param_ip_addr[8] = {192U, 168U, 1U, 21U, 192U, 168U, 1U, 20U};
static uint8_t s_param_mac_addr[6] = {0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x21U};
static uint8_t s_param_port[4] = {0x1FU, 0x90U, 0x00U, 0x00U};
static uint8_t s_param_netmask[8] = {255U, 255U, 255U, 0U, 255U, 255U, 255U, 0U};
static uint8_t s_param_gateway[8] = {192U, 168U, 1U, 1U, 192U, 168U, 1U, 1U};
static uint8_t s_param_da_ch1[DAC_PARAM_BYTES];
static uint8_t s_param_da_ch2[DAC_PARAM_BYTES];
static uint8_t s_param_da_ch3[DAC_PARAM_BYTES];
static uint8_t s_param_da_ch4[DAC_PARAM_BYTES];

//{参数ID, 参数数据存放地址, 当前有效长度, 最大允许长度}
static adc_param_block_t s_param_table[] =
    {
        {ADC_PARAM_BLOCK_CAL_DATA, s_param_cal_data,
         ADC_CAL_PARAM_BYTES, sizeof(s_param_cal_data)},
        {ADC_PARAM_BLOCK_CONTROL, s_param_control,
         sizeof(s_param_control), sizeof(s_param_control)},
        {ADC_PARAM_BLOCK_CONFIG, s_param_config,
         sizeof(s_param_config), sizeof(s_param_config)},
        {ADC_PARAM_BLOCK_IP_ADDR, s_param_ip_addr,
         sizeof(s_param_ip_addr), sizeof(s_param_ip_addr)},
        {ADC_PARAM_BLOCK_MAC_ADDR, s_param_mac_addr,
         sizeof(s_param_mac_addr), sizeof(s_param_mac_addr)},
        {ADC_PARAM_BLOCK_PORT, s_param_port,
         sizeof(s_param_port), sizeof(s_param_port)},
        {ADC_PARAM_BLOCK_NETMASK, s_param_netmask,
         sizeof(s_param_netmask), sizeof(s_param_netmask)},
        {ADC_PARAM_BLOCK_GATEWAY, s_param_gateway,
         sizeof(s_param_gateway), sizeof(s_param_gateway)},
        {ADC_PARAM_BLOCK_DA_CH1, s_param_da_ch1,
         sizeof(s_param_da_ch1), sizeof(s_param_da_ch1)},
        {ADC_PARAM_BLOCK_DA_CH2, s_param_da_ch2,
         sizeof(s_param_da_ch2), sizeof(s_param_da_ch2)},
        {ADC_PARAM_BLOCK_DA_CH3, s_param_da_ch3,
         sizeof(s_param_da_ch3), sizeof(s_param_da_ch3)},
        {ADC_PARAM_BLOCK_DA_CH4, s_param_da_ch4,
         sizeof(s_param_da_ch4), sizeof(s_param_da_ch4)},
};

static uint8_t s_network_config_dirty;

void adc_param_store_init(void)
{
    /*todo:等待device_config同步到镜像表*/
    adc_param_store_sync_from_config();
}

adc_param_block_t *adc_param_store_find_block(uint16_t block_id)
{
    uint16_t i;

    for (i = 0U; i < ADC_ARRAY_SIZE(s_param_table); i++)
    {
        if (s_param_table[i].block_id == block_id)
        {
            return &s_param_table[i];
        }
    }

    return NULL;
}

static void adc_param_store_sync_dac_param_block(uint8_t *buf,
                                                 const device_dac_channel_config_t
                                                     *config)
{
    uint16_t index;

    if ((NULL == buf) || (NULL == config))
    {
        return;
    }

    index = 0U;

    buf[index++] = config->mode;

    adc_proto_put_float_be(buf,
                           &index,
                           config->manual_voltage);

    buf[index++] = config->adc_channel;

    adc_proto_put_u32_be(buf,
                         &index,
                         (uint32_t)config->k_raw);
    adc_proto_put_u32_be(buf,
                         &index,
                         (uint32_t)config->b_raw);
}

void adc_param_store_sync_from_config(void)
{
    const device_network_config_t *net_cfg;
    const device_adc_cal_config_t *cal_cfg;
    const device_dac_config_t *dac_cfg;

    uint16_t index;
    uint8_t ch;

    net_cfg = device_config_get_network();
    cal_cfg = device_config_get_adc_calibration();
    dac_cfg = device_config_get_dac_config();

    memcpy(s_param_ip_addr, net_cfg->ip, sizeof(net_cfg->ip));
    memcpy(s_param_mac_addr, net_cfg->mac, sizeof(net_cfg->mac));

    s_param_port[0] = (uint8_t)(net_cfg->tcp_port >> 8);
    s_param_port[1] = (uint8_t)(net_cfg->tcp_port & 0xFFU);

    memcpy(s_param_netmask, net_cfg->netmask, sizeof(net_cfg->netmask));
    memcpy(s_param_gateway, net_cfg->gateway, sizeof(net_cfg->gateway));

    index = 0U;
    for (ch = 0U; ch < DEVICE_CONFIG_ADC_CHANNEL_COUNT; ch++)
    {
        adc_proto_put_u32_be(s_param_cal_data,
                             &index,
                             (uint32_t)cal_cfg->ch[ch].k_raw);
        adc_proto_put_u32_be(s_param_cal_data,
                             &index,
                             (uint32_t)cal_cfg->ch[ch].b_raw);
    }

    adc_param_store_sync_dac_param_block(s_param_da_ch1,
                                         &dac_cfg->ch[0]);
    adc_param_store_sync_dac_param_block(s_param_da_ch2,
                                         &dac_cfg->ch[1]);
    adc_param_store_sync_dac_param_block(s_param_da_ch3,
                                         &dac_cfg->ch[2]);
    adc_param_store_sync_dac_param_block(s_param_da_ch4,
                                         &dac_cfg->ch[3]);
}

uint8_t adc_param_store_check_write_len(uint16_t block_id,
                                        uint16_t len)
{
    if (ADC_PARAM_BLOCK_CAL_DATA == block_id)
    {
        return (ADC_CAL_PARAM_BYTES == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    if ((ADC_PARAM_BLOCK_DA_CH1 <= block_id) &&
        (ADC_PARAM_BLOCK_DA_CH4 >= block_id))
    {
        return (DAC_PARAM_BYTES == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    if (ADC_PARAM_BLOCK_IP_ADDR == block_id)
    {
        return (sizeof(s_param_ip_addr) == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }
    else if (ADC_PARAM_BLOCK_MAC_ADDR == block_id)
    {
        return (sizeof(s_param_mac_addr) == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }
    else if (ADC_PARAM_BLOCK_PORT == block_id)
    {
        return (sizeof(s_param_port) == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }
    else if (ADC_PARAM_BLOCK_NETMASK == block_id)
    {
        return (sizeof(s_param_netmask) == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }
    else if (ADC_PARAM_BLOCK_GATEWAY == block_id)
    {
        return (sizeof(s_param_gateway) == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    return ADC_PROTO_WRITE_STATUS_OK;
}

uint8_t adc_param_store_apply_network_param(uint16_t block_id,
                                            const uint8_t *data,
                                            uint16_t len)
{
    device_network_config_t net_cfg;

    if (NULL == data)
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    memcpy(&net_cfg,
           device_config_get_network(),
           sizeof(net_cfg));

    if (ADC_PARAM_BLOCK_IP_ADDR == block_id)
    {
        if (len < sizeof(net_cfg.ip))
        {
            SEGGER_RTT_WriteString(0, "net param ip bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }

        memcpy(net_cfg.ip, data, sizeof(net_cfg.ip));
    }
    else if (ADC_PARAM_BLOCK_MAC_ADDR == block_id)
    {
        if (len < sizeof(net_cfg.mac))
        {
            SEGGER_RTT_WriteString(0, "net param mac bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }

        memcpy(net_cfg.mac, data, sizeof(net_cfg.mac));
    }
    else if (ADC_PARAM_BLOCK_PORT == block_id)
    {
        if (len < 2U)
        {
            SEGGER_RTT_WriteString(0, "net param port bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }

        net_cfg.tcp_port = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    }
    else if (ADC_PARAM_BLOCK_NETMASK == block_id)
    {
        if (len < sizeof(net_cfg.netmask))
        {
            SEGGER_RTT_WriteString(0, "net param mask bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }

        memcpy(net_cfg.netmask, data, sizeof(net_cfg.netmask));
    }
    else if (ADC_PARAM_BLOCK_GATEWAY == block_id)
    {
        if (len < sizeof(net_cfg.gateway))
        {
            SEGGER_RTT_WriteString(0, "net param gateway bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }

        memcpy(net_cfg.gateway, data, sizeof(net_cfg.gateway));
    }
    else
    {
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    device_config_set_network(&net_cfg);
    s_network_config_dirty = 1U;

    if (HAL_OK == device_config_save_network())
    {
        SEGGER_RTT_WriteString(0, "net param saved\r\n");
        SEGGER_RTT_WriteString(0, "net param applied\r\n");
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    SEGGER_RTT_WriteString(0, "net param save pending\r\n");
    SEGGER_RTT_WriteString(0, "net param applied\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

uint8_t adc_param_store_is_network_dirty(void)
{
    return s_network_config_dirty;
}

void adc_param_store_clear_network_dirty(void)
{
    s_network_config_dirty = 0U;
}

uint8_t adc_param_store_apply_cal_param(const uint8_t *data,
                                        uint16_t len)
{
    device_adc_cal_config_t cal;
    uint16_t index;
    uint8_t ch;

    if ((NULL == data) || (ADC_CAL_PARAM_BYTES != len))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    index = 0U;

    for (ch = 0U; ch < DEVICE_CONFIG_ADC_CHANNEL_COUNT; ch++)
    {
        cal.ch[ch].k_raw = adc_proto_get_i32_be(data, &index);
        cal.ch[ch].b_raw = adc_proto_get_i32_be(data, &index);
    }

    device_config_set_adc_calibration(&cal);
    adc_param_store_sync_from_config();

    if (HAL_OK == device_config_save_all())
    {
        SEGGER_RTT_WriteString(0, "cal param saved\r\n");
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    SEGGER_RTT_WriteString(0, "cal param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

uint8_t adc_param_store_apply_dac_param(uint16_t block_id,
                                        const uint8_t *data,
                                        uint16_t len)
{
    device_dac_config_t dac_config;
    uint16_t index;
    uint8_t dac_ch;

    if ((NULL == data) || (DAC_PARAM_BYTES != len))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    if ((block_id < ADC_PARAM_BLOCK_DA_CH1) ||
        (block_id > ADC_PARAM_BLOCK_DA_CH4))
    {
        return ADC_PROTO_WRITE_STATUS_NOT_FOUND;
    }

    dac_ch = (uint8_t)(block_id - ADC_PARAM_BLOCK_DA_CH1);

    memcpy(&dac_config,
           device_config_get_dac_config(),
           sizeof(dac_config));

    index = 0U;

    dac_config.ch[dac_ch].mode = data[index++];

    dac_config.ch[dac_ch].manual_voltage = adc_proto_get_float_be(data, &index);

    dac_config.ch[dac_ch].adc_channel = data[index++];

    dac_config.ch[dac_ch].k_raw = adc_proto_get_i32_be(data, &index);

    dac_config.ch[dac_ch].b_raw = adc_proto_get_i32_be(data, &index);

    if (dac_config.ch[dac_ch].mode > DEVICE_CONFIG_DAC_MODE_ADC_CASCADE)
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    if ((DEVICE_CONFIG_DAC_MODE_ADC_CASCADE == dac_config.ch[dac_ch].mode) &&
        (dac_config.ch[dac_ch].adc_channel >= DEVICE_CONFIG_ADC_CHANNEL_COUNT))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    device_config_set_dac_config(&dac_config);
    adc_param_store_sync_from_config();

    SEGGER_RTT_WriteString(0, "dac param applied\r\n");

    if (HAL_OK == device_config_save_all())
    {
        SEGGER_RTT_WriteString(0, "dac param saved\r\n");
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    SEGGER_RTT_WriteString(0, "dac param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}
