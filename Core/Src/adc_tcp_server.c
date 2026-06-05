#include "adc_tcp_server.h"

#include "SEGGER_RTT.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "device_config.h"
#include "adc_frame_builder.h"
#include "dac_output_service.h"

#include <string.h>

/*
 * TCP server run flow:
 * 1. adc_tcp_server_init() creates a listening PCB on port 8080.
 * 2. adc_tcp_server_accept() records the connected client PCB.
 * 3. adc_tcp_server_recv() copies TCP bytes into s_rx_buf, then frees pbuf.
 * 4. adc_tcp_server_parse_rx() splits continuous bytes into protocol frames.
 * 5. adc_tcp_server_handle_frame() dispatches the command.
 *
 * The listen PCB only accepts connections.
 * The client PCB is the real data path.
 * TCP is a byte stream, so recv data must be buffered and framed.
 */

/* ========================= Protocol Definition ========================= */

#define ADC_TCP_SERVER_PORT 8080U

#define ADC_PROTO_SOF0 0x12U
#define ADC_PROTO_SOF1 0x34U
#define ADC_PROTO_EOF0 0x56U
#define ADC_PROTO_EOF1 0x78U

// cmd pc send to mcu
#define ADC_PROTO_CMD_WRITE_PARAM 0x01U
#define ADC_PROTO_CMD_READ_PARAM 0x02U
#define ADC_PROTO_CMD_HEARTBEAT 0x07U

#define ADC_PROTO_WRITE_STATUS_OK 0x00U
#define ADC_PROTO_WRITE_STATUS_SAVE_PENDING 0x01U
#define ADC_PROTO_WRITE_STATUS_BAD_LEN 0x02U
#define ADC_PROTO_WRITE_STATUS_NOT_FOUND 0x03U
#define ADC_PROTO_WRITE_STATUS_TOO_LONG 0x04U

/* 0x81/0x82 are reserved by the protocol document for ADC data stream frames. */
#define ADC_PROTO_RSP_RAW_DATA 0x81U
#define ADC_PROTO_RSP_CONVERTED_DATA 0x82U

/* Frame: SOF(2) + CMD(1) + LEN(2) + PAYLOAD(n) + CRC32(4) + EOF(2) */
#define ADC_PROTO_HEADER_SIZE 5U
#define ADC_PROTO_CRC_SIZE 4U
#define ADC_PROTO_EOF_SIZE 2U
#define ADC_PROTO_TAIL_SIZE (ADC_PROTO_CRC_SIZE + ADC_PROTO_EOF_SIZE)
#define ADC_PROTO_FRAME_OVERHEAD (ADC_PROTO_HEADER_SIZE + ADC_PROTO_TAIL_SIZE)
#define ADC_PROTO_MIN_FRAME_SIZE ADC_PROTO_FRAME_OVERHEAD

#define ADC_TCP_RX_BUF_SIZE 1500U
#define ADC_PROTO_MAX_PAYLOAD_SIZE (ADC_TCP_RX_BUF_SIZE - ADC_PROTO_FRAME_OVERHEAD)

#define ADC_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define ADC_PARAM_BLOCK_CAL_DATA 0x0001U
#define ADC_PARAM_BLOCK_CONTROL 0x0002U
#define ADC_PARAM_BLOCK_CONFIG 0x0003U
#define ADC_PARAM_BLOCK_IP_ADDR 0x0004U
#define ADC_PARAM_BLOCK_MAC_ADDR 0x0005U
#define ADC_PARAM_BLOCK_PORT 0x0006U
#define ADC_PARAM_BLOCK_NETMASK 0x0007U
#define ADC_PARAM_BLOCK_GATEWAY 0x0008U
#define ADC_PARAM_BLOCK_DA_CH1 0x0009U
#define ADC_PARAM_BLOCK_DA_CH2 0x000AU
#define ADC_PARAM_BLOCK_DA_CH3 0x000BU
#define ADC_PARAM_BLOCK_DA_CH4 0x000CU

#define ADC_CAL_PARAM_BYTES (DEVICE_CONFIG_ADC_CHANNEL_COUNT * 2U * sizeof(int32_t))

#define ADC_STREAM_TYPE_RAW ADC_PROTO_RSP_RAW_DATA
#define ADC_STREAM_TYPE_CONVERTED ADC_PROTO_RSP_CONVERTED_DATA

#define ADC_STREAM_MAX_FRAMES_PER_PUMP 32U
#define ADC_STREAM_TCP_SEND_MARGIN 32U

#define ADC_PROTO_RSP_DEBUG_STATUS 0x83U

#define DAC_PARAM_BYTES 14U

#define ADC_TCP_DEBUG_STATUS_ENABLE 0U

typedef struct
{
    uint16_t block_id;
    uint8_t *data;
    uint16_t len;
    uint16_t max_len;
} adc_param_block_t;

/* ============================= Module State ============================ */

static struct tcp_pcb *s_listen_pcb;
static struct tcp_pcb *s_client_pcb;

static uint8_t s_param_cal_data[128];
static uint8_t s_param_control[8];
static uint8_t s_param_config[4] = {1U, 0U, 0x05U, 0xB4U};
static uint8_t s_param_ip_addr[8] = {192U, 168U, 1U, 21U, 192U, 168U, 1U, 20U};
static uint8_t s_param_mac_addr[6] = {0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x21U};
static uint8_t s_param_port[4] = {0x1FU, 0x90U, 0x00U, 0x00U};
static uint8_t s_param_netmask[8] = {255U, 255U, 255U, 0U, 255U, 255U, 255U, 0U};
static uint8_t s_param_gateway[8] = {192U, 168U, 1U, 1U, 192U, 168U, 1U, 1U};
static uint8_t s_param_da_ch1[14];
static uint8_t s_param_da_ch2[14];
static uint8_t s_param_da_ch3[14];
static uint8_t s_param_da_ch4[14];

static adc_param_block_t s_param_table[] =
    {
        {ADC_PARAM_BLOCK_CAL_DATA,
         s_param_cal_data,
         ADC_CAL_PARAM_BYTES,
         sizeof(s_param_cal_data)},
        {ADC_PARAM_BLOCK_CONTROL,
         s_param_control,
         sizeof(s_param_control),
         sizeof(s_param_control)},
        {ADC_PARAM_BLOCK_CONFIG,
         s_param_config,
         sizeof(s_param_config),
         sizeof(s_param_config)},
        {ADC_PARAM_BLOCK_IP_ADDR,
         s_param_ip_addr,
         sizeof(s_param_ip_addr),
         sizeof(s_param_ip_addr)},
        {ADC_PARAM_BLOCK_MAC_ADDR,
         s_param_mac_addr,
         sizeof(s_param_mac_addr),
         sizeof(s_param_mac_addr)},
        {ADC_PARAM_BLOCK_PORT,
         s_param_port,
         sizeof(s_param_port),
         sizeof(s_param_port)},
        {ADC_PARAM_BLOCK_NETMASK,
         s_param_netmask,
         sizeof(s_param_netmask),
         sizeof(s_param_netmask)},
        {ADC_PARAM_BLOCK_GATEWAY,
         s_param_gateway,
         sizeof(s_param_gateway),
         sizeof(s_param_gateway)},
        {ADC_PARAM_BLOCK_DA_CH1,
         s_param_da_ch1,
         sizeof(s_param_da_ch1),
         sizeof(s_param_da_ch1)},
        {ADC_PARAM_BLOCK_DA_CH2,
         s_param_da_ch2,
         sizeof(s_param_da_ch2),
         sizeof(s_param_da_ch2)},
        {ADC_PARAM_BLOCK_DA_CH3,
         s_param_da_ch3,
         sizeof(s_param_da_ch3),
         sizeof(s_param_da_ch3)},
        {ADC_PARAM_BLOCK_DA_CH4,
         s_param_da_ch4,
         sizeof(s_param_da_ch4),
         sizeof(s_param_da_ch4)},
};

/* 后续做连续采集上传时使用：1=允许周期发送 ADC 数据，0=停止。 */
static uint8_t s_adc_stream_enabled;

static uint8_t s_adc_stream_type = ADC_STREAM_TYPE_RAW;
static uint32_t s_adc_stream_seq;

static uint32_t s_debug_status_last_tick;
static uint32_t s_tcp_send_fail_count;
static uint16_t s_tcp_sndbuf_min;

/* TCP 接收缓存：先把 pbuf 里的字节拷贝到这里，再释放 pbuf。 */
static uint8_t s_rx_buf[ADC_TCP_RX_BUF_SIZE];
static uint16_t s_rx_len;

/* 这些缓冲区比较大，放到静态区，避免函数调用时占用过多栈空间。 */
static uint8_t s_tx_frame_buf[ADC_TCP_RX_BUF_SIZE];
static uint8_t s_adc_stream_payload[ADC_FRAME_MAX_BYTES];
static adc_acq_sample_t s_adc_stream_samples[ADC_FRAME_BATCH_GROUP_COUNT];

static uint8_t s_network_config_dirty;

/* ========================== Function Prototypes ======================== */

/* Public entry points */
void adc_tcp_server_init(void);

void adc_tcp_server_process(void);

uint8_t adc_tcp_server_has_client(void);

/* lwIP TCP callbacks */
static err_t adc_tcp_server_accept(void *arg,
                                   struct tcp_pcb *newpcb,
                                   err_t err);

static err_t adc_tcp_server_recv(void *arg,
                                 struct tcp_pcb *tpcb,
                                 struct pbuf *p,
                                 err_t err);

static void adc_tcp_server_error(void *arg, err_t err);

/* Connection helpers */
static void adc_tcp_server_close_client(struct tcp_pcb *tpcb);

/* Receive-buffer helpers */
static void adc_tcp_server_store_pbuf(struct tcp_pcb *tpcb, struct pbuf *p);

static void adc_tcp_server_drop_one_rx_byte(void);

static void adc_tcp_server_remove_frame(uint16_t frame_len);

/* Protocol helpers */
static uint32_t adc_proto_crc32(const uint8_t *data, uint16_t len);

static uint8_t adc_proto_is_frame_valid(const uint8_t *frame,
                                        uint16_t frame_len);

static uint16_t adc_proto_payload_len(const uint8_t *frame);

static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len);

static adc_param_block_t *adc_tcp_server_find_param_block(uint16_t block_id);

static uint8_t adc_tcp_server_apply_network_param(uint16_t block_id,
                                                  const uint8_t *data,
                                                  uint16_t len);

static uint8_t adc_tcp_server_apply_cal_param(const uint8_t *data,
                                              uint16_t len);

static uint8_t adc_tcp_server_apply_dac_param(uint16_t block_id,
                                              const uint8_t *data,
                                              uint16_t len);

static int32_t adc_tcp_server_get_i32_be(const uint8_t *buf,
                                         uint16_t *index);

static void adc_tcp_server_apply_control_param(const uint8_t *data,
                                               uint16_t len);

static void adc_tcp_server_pump_adc_stream(void);

static void adc_tcp_server_sync_param_blocks_from_config(void);

static void adc_tcp_server_parse_rx(struct tcp_pcb *tpcb);

static void adc_tcp_server_handle_frame(struct tcp_pcb *tpcb,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t payload_len);

static err_t adc_tcp_server_send_write_status(struct tcp_pcb *tpcb,
                                              uint16_t block_id,
                                              uint8_t status);

static uint8_t adc_tcp_server_check_write_param_len(uint16_t block_id,
                                                    uint16_t len);

static void adc_tcp_server_send_debug_status(void);

static void adc_tcp_server_put_u32(uint8_t *buf,
                                   uint16_t *index,
                                   uint32_t value);

static void adc_tcp_server_put_u16(uint8_t *buf,
                                   uint16_t *index,
                                   uint16_t value);

static void adc_tcp_server_sync_dac_param_block(uint8_t *buf,
                                                const device_dac_channel_config_t *config);

/* ============================ Public API =============================== */

void adc_tcp_server_init(void)
{
    struct tcp_pcb *pcb;
    err_t err;

    adc_tcp_server_sync_param_blocks_from_config();

    SEGGER_RTT_WriteString(0, "tcp init begin\r\n");

    pcb = tcp_new();
    if (NULL == pcb)
    {
        SEGGER_RTT_WriteString(0, "tcp_new failed\r\n");
        return;
    }

    err = tcp_bind(pcb, IP_ADDR_ANY, ADC_TCP_SERVER_PORT);
    if (ERR_OK != err)
    {
        SEGGER_RTT_WriteString(0, "tcp_bind failed\r\n");
        tcp_close(pcb);
        return;
    }

    pcb = tcp_listen(pcb);
    if (NULL == pcb)
    {
        SEGGER_RTT_WriteString(0, "tcp_listen failed\r\n");
        return;
    }

    s_listen_pcb = pcb;
    tcp_accept(s_listen_pcb, adc_tcp_server_accept);

    SEGGER_RTT_WriteString(0, "tcp listen 8080\r\n");
}

void adc_tcp_server_process(void)
{
    if (0U != s_adc_stream_enabled)
    {
        adc_tcp_server_pump_adc_stream();
    }

#if ADC_TCP_DEBUG_STATUS_ENABLE
    adc_tcp_server_send_debug_status();
#endif
}

uint8_t adc_tcp_server_has_client(void)
{
    return (NULL != s_client_pcb) ? 1U : 0U;
}

uint8_t adc_tcp_server_is_streaming(void)
{
    return (0U != s_adc_stream_enabled) ? 1U : 0U;
}

uint8_t adc_tcp_server_is_network_config_dirty(void)
{
    return s_network_config_dirty;
}

/* =========================== lwIP Callbacks ============================ */

static err_t adc_tcp_server_accept(void *arg,
                                   struct tcp_pcb *newpcb,
                                   err_t err)
{
    LWIP_UNUSED_ARG(arg);

    if ((ERR_OK != err) || (NULL == newpcb))
    {
        return ERR_VAL;
    }

    if (NULL != s_client_pcb)
    {
        tcp_close(newpcb);
        return ERR_OK;
    }

    s_client_pcb = newpcb;
    s_tcp_sndbuf_min = tcp_sndbuf(newpcb);
    s_adc_stream_enabled = 0U;
    s_rx_len = 0U;

    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, adc_tcp_server_recv);
    tcp_err(newpcb, adc_tcp_server_error);

    SEGGER_RTT_WriteString(0, "tcp accept\r\n");

    return ERR_OK;
}

static err_t adc_tcp_server_recv(void *arg,
                                 struct tcp_pcb *tpcb,
                                 struct pbuf *p,
                                 err_t err)
{
    LWIP_UNUSED_ARG(arg);

    if ((ERR_OK != err) || (NULL == tpcb))
    {
        if (NULL != p)
        {
            pbuf_free(p);
        }

        return ERR_OK;
    }

    if (NULL == p)
    {
        adc_tcp_server_close_client(tpcb);
        return ERR_OK;
    }

    adc_tcp_server_store_pbuf(tpcb, p);
    adc_tcp_server_parse_rx(tpcb);

    return ERR_OK;
}

static void adc_tcp_server_error(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    s_client_pcb = NULL;
    s_adc_stream_enabled = 0U;
    s_rx_len = 0U;
}

/* ========================= Connection Helpers ========================== */

static void adc_tcp_server_close_client(struct tcp_pcb *tpcb)
{
    if (NULL == tpcb)
    {
        return;
    }

    tcp_arg(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_err(tpcb, NULL);

    if (tpcb == s_client_pcb)
    {
        s_client_pcb = NULL;
        s_adc_stream_enabled = 0U;
        s_rx_len = 0U;
    }

    if (ERR_OK != tcp_close(tpcb))
    {
        tcp_abort(tpcb);
    }
}

/* ======================== Receive Buffer Helpers ======================= */

static void adc_tcp_server_store_pbuf(struct tcp_pcb *tpcb, struct pbuf *p)
{
    uint16_t copy_len;

    copy_len = p->tot_len;

    if ((s_rx_len + copy_len) > ADC_TCP_RX_BUF_SIZE)
    {
        s_rx_len = 0U;
        SEGGER_RTT_WriteString(0, "rx buf overflow\r\n");
    }
    else
    {
        pbuf_copy_partial(p, &s_rx_buf[s_rx_len], copy_len, 0U);
        s_rx_len = (uint16_t)(s_rx_len + copy_len);
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
}

static void adc_tcp_server_drop_one_rx_byte(void)
{
    if (s_rx_len > 0U)
    {
        memmove(&s_rx_buf[0], &s_rx_buf[1], s_rx_len - 1U);
        s_rx_len--;
    }
}

static void adc_tcp_server_remove_frame(uint16_t frame_len)
{
    if (frame_len >= s_rx_len)
    {
        s_rx_len = 0U;
        return;
    }

    memmove(s_rx_buf, &s_rx_buf[frame_len], s_rx_len - frame_len);
    s_rx_len = (uint16_t)(s_rx_len - frame_len);
}

/* =========================== Protocol Parser =========================== */

static uint32_t adc_proto_crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1UL) != 0UL)
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

static uint16_t adc_proto_payload_len(const uint8_t *frame)
{
    return (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
}

static uint8_t adc_proto_is_frame_valid(const uint8_t *frame,
                                        uint16_t frame_len)
{
    uint16_t payload_len;
    uint16_t crc_len;
    uint32_t crc_calc;
    uint32_t crc_recv;

    if (frame_len < ADC_PROTO_MIN_FRAME_SIZE)
    {
        return 0U;
    }

    if ((frame[0] != ADC_PROTO_SOF0) || (frame[1] != ADC_PROTO_SOF1))
    {
        return 0U;
    }

    if ((frame[frame_len - 2U] != ADC_PROTO_EOF0) ||
        (frame[frame_len - 1U] != ADC_PROTO_EOF1))
    {
        return 0U;
    }

    payload_len = adc_proto_payload_len(frame);
    if (payload_len > ADC_PROTO_MAX_PAYLOAD_SIZE)
    {
        return 0U;
    }

    if (frame_len != (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD))
    {
        return 0U;
    }

    crc_len = (uint16_t)(ADC_PROTO_HEADER_SIZE + payload_len);
    crc_calc = adc_proto_crc32(frame, crc_len);
    crc_recv = ((uint32_t)frame[ADC_PROTO_HEADER_SIZE + payload_len] << 24) |
               ((uint32_t)frame[ADC_PROTO_HEADER_SIZE + payload_len + 1U] << 16) |
               ((uint32_t)frame[ADC_PROTO_HEADER_SIZE + payload_len + 2U] << 8) |
               frame[ADC_PROTO_HEADER_SIZE + payload_len + 3U];

    return (crc_calc == crc_recv) ? 1U : 0U;
}

static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len)
{
    uint8_t *frame = s_tx_frame_buf;
    uint16_t frame_len;
    uint16_t crc_len;
    uint32_t crc;
    err_t err;

    if (NULL == tpcb)
    {
        return ERR_ARG;
    }

    if (payload_len > ADC_PROTO_MAX_PAYLOAD_SIZE)
    {
        return ERR_VAL;
    }

    if ((payload_len > 0U) && (NULL == payload))
    {
        return ERR_ARG;
    }

    frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);

    if (tcp_sndbuf(tpcb) < frame_len)
    {
        return ERR_MEM;
    }

    /* Protocol frame: 12 34 CMD LEN_H LEN_L PAYLOAD CRC32 56 78. */
    frame[0] = ADC_PROTO_SOF0;
    frame[1] = ADC_PROTO_SOF1;
    frame[2] = cmd;
    frame[3] = (uint8_t)(payload_len >> 8);
    frame[4] = (uint8_t)(payload_len & 0xFFU);

    if (payload_len > 0U)
    {
        memcpy(&frame[ADC_PROTO_HEADER_SIZE], payload, payload_len);
    }

    crc_len = (uint16_t)(ADC_PROTO_HEADER_SIZE + payload_len);
    crc = adc_proto_crc32(frame, crc_len);

    frame[ADC_PROTO_HEADER_SIZE + payload_len] = (uint8_t)(crc >> 24);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 1U] = (uint8_t)(crc >> 16);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 2U] = (uint8_t)(crc >> 8);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 3U] = (uint8_t)(crc & 0xFFU);
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 4U] = ADC_PROTO_EOF0;
    frame[ADC_PROTO_HEADER_SIZE + payload_len + 5U] = ADC_PROTO_EOF1;

    err = tcp_write(tpcb, frame, frame_len, TCP_WRITE_FLAG_COPY);
    if (ERR_OK != err)
    {
        return err;
    }

    return tcp_output(tpcb);
}

static adc_param_block_t *adc_tcp_server_find_param_block(uint16_t block_id)
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

static void adc_tcp_server_sync_dac_param_block(uint8_t *buf,
                                                const device_dac_channel_config_t *config)
{
    uint16_t index;

    if ((NULL == buf) || (NULL == config))
    {
        return;
    }

    index = 0U;

    buf[index++] = config->mode;

    adc_tcp_server_put_u32(buf,
                           &index,
                           (uint32_t)config->manual_raw);

    buf[index++] = config->adc_channel;

    adc_tcp_server_put_u32(buf,
                           &index,
                           (uint32_t)config->b_raw);
    adc_tcp_server_put_u32(buf,
                           &index,
                           (uint32_t)config->b_raw);
}

static void adc_tcp_server_sync_param_blocks_from_config(void)
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
        adc_tcp_server_put_u32(s_param_cal_data,
                               &index,
                               (uint32_t)cal_cfg->ch[ch].k_raw);
        adc_tcp_server_put_u32(s_param_cal_data,
                               &index,
                               (uint32_t)cal_cfg->ch[ch].b_raw);

        adc_tcp_server_sync_dac_param_block(s_param_da_ch1,
                                            &dac_cfg->ch[0]);
        adc_tcp_server_sync_dac_param_block(s_param_da_ch2,
                                            &dac_cfg->ch[1]);
        adc_tcp_server_sync_dac_param_block(s_param_da_ch3,
                                            &dac_cfg->ch[2]);
        adc_tcp_server_sync_dac_param_block(s_param_da_ch4,
                                            &dac_cfg->ch[3]);
    }
}

static err_t adc_tcp_server_send_write_status(struct tcp_pcb *tpcb,
                                              uint16_t block_id,
                                              uint8_t status)
{
    uint8_t payload[3];

    payload[0] = (uint8_t)(block_id >> 8);
    payload[1] = (uint8_t)(block_id & 0xFFU);
    payload[2] = status;

    return adc_tcp_server_send_frame(tpcb,
                                     ADC_PROTO_CMD_WRITE_PARAM,
                                     payload,
                                     sizeof(payload));
}

static uint8_t adc_tcp_server_check_write_param_len(uint16_t block_id,
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

static void adc_tcp_server_apply_control_param(const uint8_t *data,
                                               uint16_t len)
{
    if ((NULL == data) || (len < 2U))
    {
        return;
    }

    if (0U != data[0])
    {
        s_adc_stream_enabled = 1U;

        if (ADC_STREAM_TYPE_CONVERTED == data[1])
        {
            s_adc_stream_type = ADC_STREAM_TYPE_CONVERTED;
        }
        else
        {
            s_adc_stream_type = ADC_STREAM_TYPE_RAW;
        }

        // 启动时清零方便上位机判断
        s_adc_stream_seq = 0U;
        SEGGER_RTT_WriteString(0, "adc stream start\r\n");
    }
    else
    {
        s_adc_stream_enabled = 0U;
        SEGGER_RTT_WriteString(0, "adc stream stop\r\n");
    }
}

static int32_t adc_tcp_server_get_i32_be(const uint8_t *buf,
                                         uint16_t *index)
{
    int32_t value;

    value = (int32_t)(((uint32_t)buf[*index] << 24) |
                      ((uint32_t)buf[*index + 1U] << 16) |
                      ((uint32_t)buf[*index + 2U] << 8) |
                      ((uint32_t)buf[*index + 3U]));

    *index = (uint16_t)(*index + 4U);

    return value;
}

static uint8_t adc_tcp_server_apply_cal_param(const uint8_t *data,
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
        cal.ch[ch].k_raw = adc_tcp_server_get_i32_be(data, &index);
        cal.ch[ch].b_raw = adc_tcp_server_get_i32_be(data, &index);
    }

    ddevice_config_set_adc_calibration(&cal);
    adc_tcp_server_sync_param_blocks_from_config();

    if (HAL_OK == device_config_save_all())
    {
        SEGGER_RTT_WriteString(0, "cal param saved\r\n");
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    SEGGER_RTT_WriteString(0, "cal param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

static uint8_t adc_tcp_server_apply_dac_param(uint16_t block_id,
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

    dac_config.ch[dac_ch].manual_raw = adc_tcp_server_get_i32_be(data, &index);

    dac_config.ch[dac_ch].adc_channel = data[index++];

    dac_config.ch[dac_ch].k_raw = adc_tcp_server_get_i32_be(data, &index);

    dac_config.ch[dac_ch].b_raw = adc_tcp_server_get_i32_be(data, &index);

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
    adc_tcp_server_sync_param_blocks_from_config();

    SEGGER_RTT_WriteString(0, "dac param applied\r\n");

    if (HAL_OK == device_config_save_all())
    {
        SEGGER_RTT_WriteString(0, "dac param saved\r\n");
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    SEGGER_RTT_WriteString(0, "dac param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

static uint8_t adc_tcp_server_apply_network_param(uint16_t block_id,
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

static void adc_tcp_server_parse_rx(struct tcp_pcb *tpcb)
{
    uint16_t payload_len;
    uint16_t frame_len;

    /*
     * TCP 是连续字节流：
     * - 一次 recv 可能只有半帧，所以不够一帧时要等下一次 recv。
     * - 一次 recv 也可能有多帧，所以这里用 while 连续解析。
     */
    while (s_rx_len >= ADC_PROTO_MIN_FRAME_SIZE)
    {
        if ((s_rx_buf[0] != ADC_PROTO_SOF0) ||
            (s_rx_buf[1] != ADC_PROTO_SOF1))
        {
            adc_tcp_server_drop_one_rx_byte();
            continue;
        }

        payload_len = adc_proto_payload_len(s_rx_buf);
        if (payload_len > ADC_PROTO_MAX_PAYLOAD_SIZE)
        {
            s_rx_len = 0U;
            SEGGER_RTT_WriteString(0, "proto frame too large\r\n");
            return;
        }

        frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);

        if (s_rx_len < frame_len)
        {
            return; // 未接收完整等待下一次recv
        }

        if (0U == adc_proto_is_frame_valid(s_rx_buf, frame_len))
        {
            // 误判出来的假帧头，往后寻找真的帧头
            adc_tcp_server_drop_one_rx_byte();
            SEGGER_RTT_WriteString(0, "proto bad frame\r\n");
            continue;
        }

        adc_tcp_server_handle_frame(tpcb,
                                    s_rx_buf[2],
                                    &s_rx_buf[ADC_PROTO_HEADER_SIZE],
                                    payload_len);

        // 将已处理的帧从接收缓冲区移除
        adc_tcp_server_remove_frame(frame_len);
    }
}

/* ========================== Command Dispatch =========================== */
/*
收到一帧 TCP 协议数据：
cmd         = 命令类型
payload     = 命令附带的数据
payload_len = 数据长度
tpcb        = 用来回复的 TCP 连接
*/
static void adc_tcp_server_handle_frame(struct tcp_pcb *tpcb,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t payload_len)
{
    adc_param_block_t *block;
    uint16_t block_id;
    uint16_t response_len;
    uint16_t write_len;
    uint8_t write_status;
    uint8_t response_payload[ADC_TCP_RX_BUF_SIZE];

    if (ADC_PROTO_CMD_HEARTBEAT == cmd)
    {
        SEGGER_RTT_WriteString(0, "proto heartbeat\r\n");
        (void)adc_tcp_server_send_frame(tpcb,
                                        ADC_PROTO_CMD_HEARTBEAT,
                                        NULL,
                                        0U);
    }
    else if (ADC_PROTO_CMD_WRITE_PARAM == cmd)
    {
        SEGGER_RTT_WriteString(0, "proto write param\r\n");

        if (payload_len < 2U)
        {
            SEGGER_RTT_WriteString(0, "write param bad len\r\n");
            (void)adc_tcp_server_send_write_status(tpcb,
                                                   0xFFFFU,
                                                   ADC_PROTO_WRITE_STATUS_BAD_LEN);
            return;
        }

        // 取block_id
        block_id = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
        // 查参数表
        block = adc_tcp_server_find_param_block(block_id);
        if (NULL == block)
        {
            SEGGER_RTT_WriteString(0, "write param not found\r\n");
            (void)adc_tcp_server_send_write_status(tpcb,
                                                   block_id,
                                                   ADC_PROTO_WRITE_STATUS_NOT_FOUND);
            return;
        }

        // 检查写入长度，防止数据越界

        write_len = (uint16_t)(payload_len - 2U);
        if (write_len > block->max_len)
        {
            SEGGER_RTT_WriteString(0, "write param too long\r\n");
            (void)adc_tcp_server_send_write_status(tpcb,
                                                   block_id,
                                                   ADC_PROTO_WRITE_STATUS_TOO_LONG);
            return;
        }

        // 检查写入长度是否合法

        write_status = adc_tcp_server_check_write_param_len(block_id, write_len);
        if (ADC_PROTO_WRITE_STATUS_OK != write_status)
        {
            SEGGER_RTT_WriteString(0, "write param bad len\r\n");
            (void)adc_tcp_server_send_write_status(tpcb,
                                                   block_id,
                                                   write_status);
            return;
        }

        if (write_len > 0U)
        {
            memcpy(block->data, &payload[2], write_len);
        }
        block->len = write_len;

        // 根据不同参数块，执行不同应用逻辑

        if (ADC_PARAM_BLOCK_CONTROL == block_id)
        {
            // 启动/停止 ADC 数据流

            adc_tcp_server_apply_control_param(block->data, block->len);
            write_status = ADC_PROTO_WRITE_STATUS_OK;
        }
        else if (ADC_PARAM_BLOCK_CAL_DATA == block_id)
        {
            // 更新 device_config 里的 ADC 标定参数

            write_status = adc_tcp_server_apply_cal_param(block->data, block->len);
        }
        else if ((ADC_PARAM_BLOCK_DA_CH1 <= block_id) &&
                 (ADC_PARAM_BLOCK_DA_CH4 >= block_id))
        {
            write_status = adc_tcp_server_apply_dac_param(block_id,
                                                          block->data,
                                                          block->len);
        }
        else
        {
            // 尝试当作网络参数处理
            write_status = adc_tcp_server_apply_network_param(block_id,
                                                              block->data,
                                                              block->len);
        }

        // 回复写入结果
        (void)adc_tcp_server_send_write_status(tpcb,
                                               block_id,
                                               write_status);
    }
    else if (ADC_PROTO_CMD_READ_PARAM == cmd)
    {
        SEGGER_RTT_WriteString(0, "proto read param\r\n");

        if (2U != payload_len)
        {
            SEGGER_RTT_WriteString(0, "read param bad len\r\n");
            return;
        }

        block_id = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
        block = adc_tcp_server_find_param_block(block_id);
        if (NULL == block)
        {
            SEGGER_RTT_WriteString(0, "read param not found\r\n");
            return;
        }

        // 计算响应长度
        response_len = (uint16_t)(block->len + 2U);
        // 检查响应长度是否超过协议最大 payload
        if (response_len > ADC_PROTO_MAX_PAYLOAD_SIZE)
        {
            SEGGER_RTT_WriteString(0, "read param too long\r\n");
            return;
        }

        // 填充响应 payload
        response_payload[0] = (uint8_t)(block_id >> 8);
        response_payload[1] = (uint8_t)(block_id & 0xFFU);
        if (block->len > 0U)
        {
            memcpy(&response_payload[2], block->data, block->len);
        }

        // 发送读参数响应8

        (void)adc_tcp_server_send_frame(tpcb,
                                        ADC_PROTO_CMD_READ_PARAM,
                                        response_payload,
                                        response_len);
    }
    else
    {
        SEGGER_RTT_WriteString(0, "proto unknown cmd\r\n");
    }
}

static void adc_tcp_server_put_u32(uint8_t *buf,
                                   uint16_t *index,
                                   uint32_t value)
{
    buf[(*index)++] = (uint8_t)(value >> 24);
    buf[(*index)++] = (uint8_t)(value >> 16);
    buf[(*index)++] = (uint8_t)(value >> 8);
    buf[(*index)++] = (uint8_t)(value & 0xFFU);
}

static void adc_tcp_server_put_u16(uint8_t *buf,
                                   uint16_t *index,
                                   uint16_t value)
{
    buf[(*index)++] = (uint8_t)(value >> 8);
    buf[(*index)++] = (uint8_t)(value & 0xFFU);
}

static void adc_tcp_server_send_debug_status(void)
{
    adc_acq_stats_t stats;
    uint8_t payload[64];
    uint16_t index = 0U;
    uint32_t now_tick;

    if (NULL == s_client_pcb)
    {
        return;
    }

    now_tick = HAL_GetTick();

    if ((now_tick - s_debug_status_last_tick) < 1000U)
    {
        return;
    }

    s_debug_status_last_tick = now_tick;

    adc_acq_service_get_stats(&stats);

    adc_tcp_server_put_u32(payload, &index, stats.adc1_half_count);
    adc_tcp_server_put_u32(payload, &index, stats.adc2_half_count);
    adc_tcp_server_put_u32(payload, &index, stats.adc3_half_count);

    adc_tcp_server_put_u32(payload, &index, stats.adc1_full_count);
    adc_tcp_server_put_u32(payload, &index, stats.adc2_full_count);
    adc_tcp_server_put_u32(payload, &index, stats.adc3_full_count);

    adc_tcp_server_put_u32(payload, &index, stats.sample_seq);
    adc_tcp_server_put_u32(payload, &index, stats.no_block_count);

    adc_tcp_server_put_u32(payload, &index, s_tcp_send_fail_count);
    adc_tcp_server_put_u32(payload, &index, s_adc_stream_seq);
    adc_tcp_server_put_u16(payload, &index, s_tcp_sndbuf_min);

    (void)adc_tcp_server_send_frame(s_client_pcb,
                                    ADC_PROTO_RSP_DEBUG_STATUS,
                                    payload,
                                    index);
}

static void adc_tcp_server_pump_adc_stream(void)
{
    uint16_t payload_len;
    uint16_t frame_len;
    uint8_t frame_count;
    uint16_t sample_count;
    uint16_t sndbuf_now;
    uint16_t max_sample_count;

    if (NULL == s_client_pcb)
    {
        s_adc_stream_enabled = 0U;
        return;
    }

    for (frame_count = 0U;
         frame_count < ADC_STREAM_MAX_FRAMES_PER_PUMP;
         frame_count++)
    {
        sndbuf_now = tcp_sndbuf(s_client_pcb);

        if ((0U == s_tcp_sndbuf_min) || (sndbuf_now < s_tcp_sndbuf_min))
        {
            s_tcp_sndbuf_min = sndbuf_now;
        }

        if (sndbuf_now < (uint16_t)(ADC_FRAME_MAX_BYTES + ADC_PROTO_FRAME_OVERHEAD + ADC_STREAM_TCP_SEND_MARGIN))
        {
            return;
        }

        // raw 和 converted 每帧样本数不同,raw 12*2*32 converted 12*4*16
        if (ADC_STREAM_TYPE_CONVERTED == s_adc_stream_type)
        {
            max_sample_count = ADC_FRAME_CONVERTED_BATCH_GROUP_COUNT;
        }
        else
        {
            max_sample_count = ADC_FRAME_BATCH_GROUP_COUNT;
        }

        sample_count = 0U;
        while (sample_count < max_sample_count)
        {
            // 拿到了一组12通道样本
            if (0U == adc_acq_service_get_sample(&s_adc_stream_samples[sample_count]))
            {
                break;
            }

            dac_output_service_apply_adc_sample(&s_adc_stream_samples[sample_count]);
            sample_count++;
        }

        if (0U == sample_count)
        {
            return;
        }

        if (ADC_STREAM_TYPE_CONVERTED == s_adc_stream_type)
        {
            payload_len = adc_frame_builder_build_cal_float_batch(s_adc_stream_samples,
                                                                  sample_count,
                                                                  s_adc_stream_seq,
                                                                  0x0FFFU,
                                                                  device_config_get_adc_calibration(),
                                                                  s_adc_stream_payload,
                                                                  sizeof(s_adc_stream_payload));
        }
        else
        {
            payload_len = adc_frame_builder_build_raw_u16_batch(s_adc_stream_samples,
                                                                sample_count,
                                                                s_adc_stream_seq,
                                                                0x0FFFU,
                                                                s_adc_stream_payload,
                                                                sizeof(s_adc_stream_payload));
        }

        if (0U == payload_len)
        {
            return;
        }

        /*
         * Check lwIP send buffer again with the actual frame length.
         * If the buffer is tight, wait for the next main-loop pump.
         */
        frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);

        sndbuf_now = tcp_sndbuf(s_client_pcb);

        if ((0U == s_tcp_sndbuf_min) || (sndbuf_now < s_tcp_sndbuf_min))
        {
            s_tcp_sndbuf_min = sndbuf_now;
        }

        if (sndbuf_now < (uint16_t)(frame_len + ADC_STREAM_TCP_SEND_MARGIN))
        {
            return;
        }

        // 发送外层协议帧
        if (ERR_OK != adc_tcp_server_send_frame(s_client_pcb,
                                                s_adc_stream_type,
                                                s_adc_stream_payload,
                                                payload_len))
        {
            s_tcp_send_fail_count++;
            return;
        }

        // 更新流序列号
        s_adc_stream_seq += sample_count;
    }
}
