#include "adc_tcp_server.h"

#include "SEGGER_RTT.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include <string.h>

/*
 * TCP server run flow:
 * 1. adc_tcp_server_init() creates a listening PCB on port 8080.
 * 2. adc_tcp_server_accept() records the connected client PCB.
 * 3. adc_tcp_server_recv() copies TCP bytes into s_rx_buf, then frees pbuf.
 * 4. adc_tcp_server_parse_rx() splits continuous bytes into protocol frames.
 * 5. adc_tcp_server_handle_frame() dispatches the command.
 *
 * 中文速记：
 * 监听 PCB 只负责等客户端连进来；客户端 PCB 才是真正收发数据的连接。
 * TCP 是字节流，所以 recv 每次收到的内容不一定刚好是一帧，必须用缓存切帧。
 */

/* ========================= Protocol Definition ========================= */

#define ADC_TCP_SERVER_PORT 8080U

#define ADC_PROTO_SOF0 0x12U
#define ADC_PROTO_SOF1 0x34U
#define ADC_PROTO_EOF0 0x56U
#define ADC_PROTO_EOF1 0x78U

#define ADC_PROTO_CMD_WRITE_PARAM 0x01U
#define ADC_PROTO_CMD_READ_PARAM 0x02U
#define ADC_PROTO_CMD_HEARTBEAT 0x07U

/* Frame: SOF(2) + CMD(1) + LEN(2) + PAYLOAD(n) + SUM(2) + EOF(2) */
#define ADC_PROTO_HEADER_SIZE 5U
#define ADC_PROTO_TAIL_SIZE 4U
#define ADC_PROTO_FRAME_OVERHEAD (ADC_PROTO_HEADER_SIZE + ADC_PROTO_TAIL_SIZE)
#define ADC_PROTO_MIN_FRAME_SIZE ADC_PROTO_FRAME_OVERHEAD

#define ADC_TCP_RX_BUF_SIZE 512U
#define ADC_PROTO_MAX_PAYLOAD_SIZE (ADC_TCP_RX_BUF_SIZE - ADC_PROTO_FRAME_OVERHEAD)

/* ============================= Module State ============================ */

static struct tcp_pcb *s_listen_pcb;
static struct tcp_pcb *s_client_pcb;

/* 后续做连续采集上传时使用：1=允许周期发送 ADC 数据，0=停止。 */
static uint8_t s_adc_stream_enabled;

/* TCP 接收缓存：先把 pbuf 里的字节拷贝到这里，再释放 pbuf。 */
static uint8_t s_rx_buf[ADC_TCP_RX_BUF_SIZE];
static uint16_t s_rx_len;

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
static uint16_t adc_proto_checksum16(const uint8_t *data, uint16_t len);
static uint8_t adc_proto_is_frame_valid(const uint8_t *frame,
                                        uint16_t frame_len);
static uint16_t adc_proto_payload_len(const uint8_t *frame);
static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len);
static void adc_tcp_server_parse_rx(struct tcp_pcb *tpcb);
static void adc_tcp_server_handle_frame(struct tcp_pcb *tpcb,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t payload_len);

/* ============================ Public API =============================== */

void adc_tcp_server_init(void)
{
    struct tcp_pcb *pcb;
    err_t err;

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
    /*
     * 这里会长期被 main loop 调用。
     * 当前阶段先保留接口；下一步做 ADC 连续上传时，把“周期发送”逻辑放这里。
     */
    if (0U != s_adc_stream_enabled)
    {
        /* TODO: pump ADC stream frames. */
    }
}

uint8_t adc_tcp_server_has_client(void)
{
    return (NULL != s_client_pcb) ? 1U : 0U;
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

    /*
     * 当前先做单客户端模式：监听 PCB 继续存在，但同一时间只服务一个 client PCB。
     * 后面如果要多客户端，就不能只用一个全局 s_client_pcb，需要给每个连接分配上下文。
     */
    if (NULL != s_client_pcb)
    {
        tcp_close(newpcb);
        return ERR_OK;
    }

    s_client_pcb = newpcb;
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
        /*
         * 缓存满时先丢弃旧数据，避免协议解析一直卡在半包或脏数据上。
         * tcp_recved 仍然要调用，告诉 lwIP 这些字节已经被应用层处理掉。
         */
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

static uint16_t adc_proto_checksum16(const uint8_t *data, uint16_t len)
{
    uint32_t sum = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        sum += data[i];
    }

    return (uint16_t)sum;
}

static uint16_t adc_proto_payload_len(const uint8_t *frame)
{
    return (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
}

static uint8_t adc_proto_is_frame_valid(const uint8_t *frame,
                                        uint16_t frame_len)
{
    uint16_t payload_len;
    uint16_t checksum_calc;
    uint16_t checksum_recv;
    uint16_t checksum_len;

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

    /* 校验范围：帧头 + 命令 + 长度 + payload，不包含校验字段和帧尾。 */
    checksum_len = (uint16_t)(ADC_PROTO_HEADER_SIZE + payload_len);
    checksum_calc = adc_proto_checksum16(frame, checksum_len);
    checksum_recv = ((uint16_t)frame[ADC_PROTO_HEADER_SIZE + payload_len] << 8) |
                    frame[ADC_PROTO_HEADER_SIZE + payload_len + 1U];

    return (checksum_calc == checksum_recv) ? 1U : 0U;
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
            return;
        }

        if (0U == adc_proto_is_frame_valid(s_rx_buf, frame_len))
        {
            adc_tcp_server_drop_one_rx_byte();
            SEGGER_RTT_WriteString(0, "proto bad frame\r\n");
            continue;
        }

        adc_tcp_server_handle_frame(tpcb,
                                    s_rx_buf[2],
                                    &s_rx_buf[ADC_PROTO_HEADER_SIZE],
                                    payload_len);

        adc_tcp_server_remove_frame(frame_len);
    }
}

/* ========================== Command Dispatch =========================== */

static void adc_tcp_server_handle_frame(struct tcp_pcb *tpcb,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t payload_len)
{
    LWIP_UNUSED_ARG(tpcb);
    LWIP_UNUSED_ARG(payload);
    LWIP_UNUSED_ARG(payload_len);

    if (ADC_PROTO_CMD_HEARTBEAT == cmd)
    {
        SEGGER_RTT_WriteString(0, "proto heartbeat\r\n");
    }
    else if (ADC_PROTO_CMD_WRITE_PARAM == cmd)
    {
        SEGGER_RTT_WriteString(0, "proto write param\r\n");
    }
    else if (ADC_PROTO_CMD_READ_PARAM == cmd)
    {
        SEGGER_RTT_WriteString(0, "proto read param\r\n");
    }
    else
    {
        SEGGER_RTT_WriteString(0, "proto unknown cmd\r\n");
    }
}
