#include "adc_tcp_server.h"

#include "SEGGER_RTT.h"
#include "lwip.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "device_config.h"
#include "adc_proto.h"
#include "adc_param_store.h"
#include "adc_frame_builder.h"
#include "dac_output_service.h"
#include "app_log.h"

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

// cmd pc send to mcu
#define ADC_PROTO_CMD_WRITE_PARAM 0x01U
#define ADC_PROTO_CMD_READ_PARAM 0x02U
#define ADC_PROTO_CMD_HEARTBEAT 0x07U

/* 0x81/0x82 are reserved by the protocol document for ADC data stream frames. */
#define ADC_PROTO_RSP_RAW_DATA 0x81U
#define ADC_PROTO_RSP_CONVERTED_DATA 0x82U

#define ADC_LOG_DIAG_ACTION_CLEAR 0x01U
#define ADC_LOG_DIAG_ACTION_STOP_WATCHDOG_FEED 0xA5U

/* Debug-only command: write block 0x000D with DATA=A5 to verify IWDG reset. */
#define ADC_TCP_WATCHDOG_TEST_ENABLE 1U

#define ADC_TCP_RX_BUF_SIZE 1500U
#define ADC_PROTO_MAX_PAYLOAD_SIZE (ADC_TCP_RX_BUF_SIZE - ADC_PROTO_FRAME_OVERHEAD)

#define ADC_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define ADC_CAL_PARAM_BYTES (DEVICE_CONFIG_ADC_CHANNEL_COUNT * 2U * sizeof(int32_t))

#define ADC_STREAM_TYPE_RAW ADC_PROTO_RSP_RAW_DATA
#define ADC_STREAM_TYPE_CONVERTED ADC_PROTO_RSP_CONVERTED_DATA

#define ADC_STREAM_MAX_FRAMES_PER_PUMP 32U
#define ADC_STREAM_TCP_SEND_MARGIN 32U

#define ADC_PROTO_RSP_DEBUG_STATUS 0x83U

#define ADC_LOG_SNAPSHOT_VERSION 1U
#define ADC_LOG_SNAPSHOT_MAX_RECORDS 8U
#define ADC_LOG_SNAPSHOT_HEADER_BYTES 4U
#define ADC_LOG_RECORD_WIRE_BYTES 16U

#define DAC_PARAM_BYTES 14U

#define ADC_TCP_DEBUG_STATUS_ENABLE 0U

/* ============================= Module State ============================ */

static struct tcp_pcb *s_listen_pcb;
static struct tcp_pcb *s_client_pcb;

/* 后续做连续采集上传时使用：1=允许周期发送 ADC 数据，0=停止。 */
static uint8_t s_adc_stream_enabled;

static uint8_t s_adc_stream_type = ADC_STREAM_TYPE_RAW;
static uint32_t s_adc_stream_seq;

static uint32_t s_debug_status_last_tick;
static uint32_t s_tcp_send_fail_count;
static uint16_t s_tcp_sndbuf_min;
static uint8_t s_watchdog_feed_enabled = 1U;

/* TCP 接收缓存：先把 pbuf 里的字节拷贝到这里，再释放 pbuf。 */
static uint8_t s_rx_buf[ADC_TCP_RX_BUF_SIZE];
static uint16_t s_rx_len;

/* 这些缓冲区比较大，放到静态区，避免函数调用时占用过多栈空间。 */
static uint8_t s_tx_frame_buf[ADC_TCP_RX_BUF_SIZE];
static uint8_t s_adc_stream_payload[ADC_FRAME_MAX_BYTES];
static uint8_t s_fixed_cmd_payload[ADC_PROTO_FIXED_BLOCK_ID_SIZE +
                                   ADC_PROTO_FIXED_DATA_CAPACITY];
static uint8_t s_log_snapshot_payload[ADC_PROTO_FIXED_DATA_CAPACITY];
static app_log_record_t s_log_snapshot_records[ADC_LOG_SNAPSHOT_MAX_RECORDS];
static adc_acq_sample_t s_adc_stream_samples[ADC_FRAME_BATCH_GROUP_COUNT];

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

static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len);

static err_t adc_tcp_server_send_fixed_frame(struct tcp_pcb *tpcb,
                                             uint8_t cmd,
                                             uint16_t block_id,
                                             const uint8_t *data,
                                             uint16_t data_len);

static uint8_t adc_tcp_server_is_fixed_cmd(uint8_t cmd);

static void adc_tcp_server_apply_control_param(const uint8_t *data,
                                               uint16_t len);

static uint8_t adc_tcp_server_normalize_stream_type(uint8_t stream_type);

static void adc_tcp_server_start_stream(uint8_t stream_type);

static void adc_tcp_server_stop_stream(void);

static void adc_tcp_server_disable_stream(void);

static void adc_tcp_server_update_sndbuf_min(uint16_t sndbuf_now);

static uint8_t adc_tcp_server_can_send_bytes(uint16_t need_len);

static uint16_t adc_tcp_server_collect_stream_samples(uint16_t max_sample_count);

static uint16_t adc_tcp_server_build_stream_payload(uint16_t sample_count);

static uint16_t adc_tcp_server_get_stream_batch_count(void);

static void adc_tcp_server_pump_adc_stream(void);

static void adc_tcp_server_parse_rx(struct tcp_pcb *tpcb);

static void adc_tcp_server_handle_frame(struct tcp_pcb *tpcb,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t payload_len);

static void adc_tcp_server_handle_write_param(struct tcp_pcb *tpcb,
                                              const uint8_t *payload,
                                              uint16_t payload_len);

static void adc_tcp_server_handle_log_clear(struct tcp_pcb *tpcb,
                                            const uint8_t *payload,
                                            uint16_t payload_len);

static void adc_tcp_server_handle_read_param(struct tcp_pcb *tpcb,
                                             const uint8_t *payload,
                                             uint16_t payload_len);

static err_t adc_tcp_server_send_write_status(struct tcp_pcb *tpcb,
                                              uint16_t block_id,
                                              uint8_t status);

static err_t adc_tcp_server_send_read_status(struct tcp_pcb *tpcb,
                                             uint16_t block_id,
                                             uint8_t status);
static uint16_t adc_tcp_server_build_log_snapshot(uint8_t *payload,
                                                  uint16_t max_len);

static void adc_tcp_server_send_debug_status(void);

/* ============================ Public API =============================== */

void adc_tcp_server_init(void)
{
    struct tcp_pcb *pcb;
    err_t err;

    adc_param_store_init();

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

    if (NULL == s_client_pcb)
    {
        if ((0U != adc_param_store_is_network_dirty()) ||
            (0U == MX_LWIP_IsNetworkConfigApplied()))
        {
            MX_LWIP_ApplyNetworkConfig();
            adc_param_store_clear_network_dirty();
            SEGGER_RTT_WriteString(0, "netif param applied\r\n");
            app_log_record(APP_LOG_EVENT_NETIF_APPLIED,
                           0U,
                           0U,
                           0U);
        }
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
    return adc_param_store_is_network_dirty();
}

uint8_t adc_tcp_server_is_watchdog_feed_enabled(void)
{
    return (0U != s_watchdog_feed_enabled) ? 1U : 0U;
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
    adc_tcp_server_disable_stream();
    s_rx_len = 0U;

    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, adc_tcp_server_recv);
    tcp_err(newpcb, adc_tcp_server_error);

    SEGGER_RTT_WriteString(0, "tcp accept\r\n");
    app_log_record(APP_LOG_EVENT_TCP_ACCEPT,
                   0U,
                   0U,
                   0U);

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
    adc_tcp_server_disable_stream();
    s_rx_len = 0U;

    app_log_record(APP_LOG_EVENT_TCP_ERROR,
                   (uint16_t)err,
                   0U,
                   0U);
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
        adc_tcp_server_disable_stream();
        s_rx_len = 0U;

        app_log_record(APP_LOG_EVENT_TCP_CLOSE,
                       0U,
                       0U,
                       0U);
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
        uint16_t old_rx_len;

        old_rx_len = s_rx_len;
        s_rx_len = 0U;
        SEGGER_RTT_WriteString(0, "rx buf overflow\r\n");

        app_log_record(APP_LOG_EVENT_TCP_RX_OVERFLOW,
                       0U,
                       old_rx_len,
                       copy_len);
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

static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len)
{
    uint8_t *frame = s_tx_frame_buf;
    uint16_t frame_len;
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
    frame_len = adc_proto_build_frame(frame,
                                      sizeof(s_tx_frame_buf),
                                      cmd,
                                      payload,
                                      payload_len);

    if (0U == frame_len)
    {
        return ERR_VAL;
    }

    if (tcp_sndbuf(tpcb) < frame_len)
    {
        return ERR_MEM;
    }

    err = tcp_write(tpcb, frame, frame_len, TCP_WRITE_FLAG_COPY);
    if (ERR_OK != err)
    {
        return err;
    }

    return tcp_output(tpcb);
}

static err_t adc_tcp_server_send_fixed_frame(struct tcp_pcb *tpcb,
                                             uint8_t cmd,
                                             uint16_t block_id,
                                             const uint8_t *data,
                                             uint16_t data_len)
{
    uint16_t frame_len;
    err_t err;

    if (NULL == tpcb)
    {
        return ERR_ARG;
    }

    frame_len = adc_proto_build_fixed_frame(s_tx_frame_buf,
                                            sizeof(s_tx_frame_buf),
                                            cmd,
                                            block_id,
                                            data,
                                            data_len);

    if (0U == frame_len)
    {
        return ERR_VAL;
    }

    if (tcp_sndbuf(tpcb) < frame_len)
    {
        return ERR_MEM;
    }

    err = tcp_write(tpcb, s_tx_frame_buf, frame_len, TCP_WRITE_FLAG_COPY);
    if (ERR_OK != err)
    {
        return err;
    }

    return tcp_output(tpcb);
}

static uint8_t adc_tcp_server_is_fixed_cmd(uint8_t cmd)
{
    if ((ADC_PROTO_CMD_WRITE_PARAM == cmd) ||
        (ADC_PROTO_CMD_READ_PARAM == cmd) ||
        (ADC_PROTO_CMD_HEARTBEAT == cmd))
    {
        return 1U;
    }

    return 0U;
}

static err_t adc_tcp_server_send_write_status(struct tcp_pcb *tpcb,
                                              uint16_t block_id,
                                              uint8_t status)
{
    return adc_tcp_server_send_fixed_frame(tpcb,
                                           ADC_PROTO_CMD_WRITE_PARAM,
                                           block_id,
                                           &status,
                                           sizeof(status));
}

static err_t adc_tcp_server_send_read_status(struct tcp_pcb *tpcb,
                                             uint16_t block_id,
                                             uint8_t status)
{
    return adc_tcp_server_send_fixed_frame(tpcb,
                                           ADC_PROTO_CMD_READ_PARAM,
                                           block_id,
                                           &status,
                                           sizeof(status));
}

static uint8_t adc_tcp_server_normalize_stream_type(uint8_t stream_type)
{
    if (ADC_STREAM_TYPE_CONVERTED == stream_type)
    {
        return ADC_STREAM_TYPE_CONVERTED;
    }

    return ADC_STREAM_TYPE_RAW;
}

static void adc_tcp_server_start_stream(uint8_t stream_type)
{
    s_adc_stream_type = adc_tcp_server_normalize_stream_type(stream_type);
    s_adc_stream_seq = 0U;
    s_adc_stream_enabled = 1U;

    SEGGER_RTT_WriteString(0, "adc stream start\r\n");
    app_log_record(APP_LOG_EVENT_ADC_STREAM_START,
                   0U,
                   0U,
                   0U);
}

static void adc_tcp_server_stop_stream(void)
{
    adc_tcp_server_disable_stream();

    SEGGER_RTT_WriteString(0, "adc stream stop\r\n");
    app_log_record(APP_LOG_EVENT_ADC_STREAM_STOP,
                   0U,
                   0U,
                   0U);
}

static void adc_tcp_server_disable_stream(void)
{
    s_adc_stream_enabled = 0U;
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
        adc_tcp_server_start_stream(data[1]);
    }
    else
    {
        adc_tcp_server_stop_stream();
    }
}

static void adc_tcp_server_parse_rx(struct tcp_pcb *tpcb)
{
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

        if (0U != adc_tcp_server_is_fixed_cmd(s_rx_buf[2]))
        {
            uint16_t data_len;
            uint16_t block_id;
            uint8_t bad_cmd;

            frame_len = ADC_PROTO_FIXED_FRAME_SIZE;

            if (s_rx_len < frame_len)
            {
                return;
            }

            bad_cmd = s_rx_buf[2];

            if (0U == adc_proto_is_fixed_frame_valid(s_rx_buf))
            {
                adc_tcp_server_drop_one_rx_byte();
                SEGGER_RTT_WriteString(0, "proto bad fixed frame\r\n");
                app_log_record(APP_LOG_EVENT_TCP_BAD_FRAME,
                               bad_cmd,
                               s_rx_len,
                               0U);
                continue;
            }

            data_len = adc_proto_fixed_data_len(s_rx_buf);
            block_id = adc_proto_fixed_block_id(s_rx_buf);

            if ((uint16_t)(data_len + ADC_PROTO_FIXED_BLOCK_ID_SIZE) >
                sizeof(s_fixed_cmd_payload))
            {
                adc_tcp_server_remove_frame(frame_len);
                SEGGER_RTT_WriteString(0, "fixed payload too large\r\n");
                app_log_record(APP_LOG_EVENT_TCP_BAD_FRAME,
                               s_rx_buf[2],
                               data_len,
                               sizeof(s_fixed_cmd_payload));
                continue;
            }

            s_fixed_cmd_payload[0] = (uint8_t)(block_id >> 8);
            s_fixed_cmd_payload[1] = (uint8_t)(block_id & 0xFFU);

            if (data_len > 0U)
            {
                memcpy(&s_fixed_cmd_payload[2],
                       &s_rx_buf[ADC_PROTO_FIXED_DATA_OFFSET + ADC_PROTO_FIXED_BLOCK_ID_SIZE],
                       data_len);
            }

            adc_tcp_server_handle_frame(tpcb,
                                        s_rx_buf[2],
                                        s_fixed_cmd_payload,
                                        (uint16_t)(data_len + ADC_PROTO_FIXED_BLOCK_ID_SIZE));

            // 将已处理的帧从接收缓冲区移除
            adc_tcp_server_remove_frame(frame_len);
            continue;
        }

        /*
         * 现在上位机下发的普通命令都应使用固定 150 字节帧。
         * 如果这里出现未知 cmd，不能继续按 LEN 等待，否则一条错误短帧
         * 可能把后续正确命令全部堵在接收缓冲区后面。
         */
        adc_tcp_server_drop_one_rx_byte();
        SEGGER_RTT_WriteString(0, "proto drop unknown rx cmd\r\n");
        continue;
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

    if (ADC_PROTO_CMD_HEARTBEAT == cmd)
    {
        uint8_t status = ADC_PROTO_WRITE_STATUS_OK;

        SEGGER_RTT_WriteString(0, "proto heartbeat\r\n");
        app_log_record(APP_LOG_EVENT_PROTO_HEARTBEAT,
                       0U,
                       0U,
                       0U);
        (void)adc_tcp_server_send_fixed_frame(tpcb,
                                              ADC_PROTO_CMD_HEARTBEAT,
                                              0U,
                                              &status,
                                              sizeof(status));
    }
    else if (ADC_PROTO_CMD_WRITE_PARAM == cmd)
    {
        adc_tcp_server_handle_write_param(tpcb, payload, payload_len);
    }
    else if (ADC_PROTO_CMD_READ_PARAM == cmd)
    {
        adc_tcp_server_handle_read_param(tpcb, payload, payload_len);
    }
    else
    {
        SEGGER_RTT_WriteString(0, "proto unknown cmd\r\n");
    }
}

static void adc_tcp_server_handle_log_clear(struct tcp_pcb *tpcb,
                                            const uint8_t *payload,
                                            uint16_t payload_len)
{
    uint16_t block_id;
    uint8_t status;

    block_id = ADC_PARAM_BLOCK_LOG_SNAPSHOT;
    status = ADC_PROTO_WRITE_STATUS_BAD_LEN;

    if ((3U == payload_len) &&
        (ADC_LOG_DIAG_ACTION_CLEAR == payload[2]))
    {
        app_log_clear();
        app_log_record(APP_LOG_EVENT_LOG_CLEARED,
                       0U,
                       0U,
                       0U);
        status = ADC_PROTO_WRITE_STATUS_OK;
    }
#if ADC_TCP_WATCHDOG_TEST_ENABLE
    else if ((3U == payload_len) &&
             (ADC_LOG_DIAG_ACTION_STOP_WATCHDOG_FEED == payload[2]))
    {
        s_watchdog_feed_enabled = 0U;
        app_log_record(APP_LOG_EVENT_WATCHDOG_TEST,
                       payload[2],
                       0U,
                       0U);
        status = ADC_PROTO_WRITE_STATUS_OK;
    }
#endif

    (void)adc_tcp_server_send_write_status(tpcb,
                                           block_id,
                                           status);

    app_log_record(APP_LOG_EVENT_PARAM_WRITE_RESULT,
                   status,
                   block_id,
                   (payload_len >= 2U) ? (uint32_t)(payload_len - 2U) : 0U);
}

static void adc_tcp_server_handle_write_param(struct tcp_pcb *tpcb,
                                              const uint8_t *payload,
                                              uint16_t payload_len)
{
    adc_param_block_t *block;
    uint16_t block_id;
    uint16_t write_len;
    uint8_t write_status;

    SEGGER_RTT_WriteString(0, "proto write param\r\n");
    app_log_record(APP_LOG_EVENT_PROTO_WRITE_PARAM,
                   0U,
                   payload_len,
                   0U);

    if (payload_len < 2U)
    {
        SEGGER_RTT_WriteString(0, "write param bad len\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               0xFFFFU,
                                               ADC_PROTO_WRITE_STATUS_BAD_LEN);
        return;
    }

    block_id = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    if (ADC_PARAM_BLOCK_LOG_SNAPSHOT == block_id)
    {
        adc_tcp_server_handle_log_clear(tpcb, payload, payload_len);
        return;
    }
    block = adc_param_store_find_block(block_id);
    if (NULL == block)
    {
        SEGGER_RTT_WriteString(0, "write param not found\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               block_id,
                                               ADC_PROTO_WRITE_STATUS_NOT_FOUND);
        return;
    }

    write_len = (uint16_t)(payload_len - 2U);
    if (write_len > block->max_len)
    {
        SEGGER_RTT_WriteString(0, "write param too long\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               block_id,
                                               ADC_PROTO_WRITE_STATUS_TOO_LONG);
        return;
    }

    write_status = adc_param_store_check_write_len(block_id, write_len);
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

    if (ADC_PARAM_BLOCK_CONTROL == block_id)
    {
        adc_tcp_server_apply_control_param(block->data, block->len);
        write_status = ADC_PROTO_WRITE_STATUS_OK;
    }
    else if (ADC_PARAM_BLOCK_CAL_DATA == block_id)
    {
        write_status = adc_param_store_apply_cal_param(block->data, block->len);
    }
    else if ((ADC_PARAM_BLOCK_DA_CH1 <= block_id) &&
             (ADC_PARAM_BLOCK_DA_CH4 >= block_id))
    {
        write_status = adc_param_store_apply_dac_param(block_id,
                                                       block->data,
                                                       block->len);
    }
    else
    {
        write_status = adc_param_store_apply_network_param(block_id,
                                                           block->data,
                                                           block->len);
    }

    (void)adc_tcp_server_send_write_status(tpcb,
                                           block_id,
                                           write_status);

    app_log_record(APP_LOG_EVENT_PARAM_WRITE_RESULT,
                   write_status,
                   block_id,
                   write_len);
}

static uint16_t adc_tcp_server_build_log_snapshot(uint8_t *payload,
                                                  uint16_t max_len)
{
    uint16_t index;
    uint16_t count;
    uint16_t i;

    if ((NULL == payload) || (max_len < ADC_LOG_SNAPSHOT_HEADER_BYTES))
    {
        return 0U;
    }

    count = app_log_snapshot(s_log_snapshot_records,
                             ADC_LOG_SNAPSHOT_MAX_RECORDS);

    index = 0U;
    payload[index++] = ADC_LOG_SNAPSHOT_VERSION;
    payload[index++] = (uint8_t)count;
    payload[index++] = ADC_LOG_RECORD_WIRE_BYTES;
    payload[index++] = 0U;

    for (i = 0U; i < count; i++)
    {
        if ((uint16_t)(index + ADC_LOG_RECORD_WIRE_BYTES) > max_len)
        {
            break;
        }

        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].tick_ms);
        adc_proto_put_u16_be(payload, &index, s_log_snapshot_records[i].event);
        adc_proto_put_u16_be(payload, &index, s_log_snapshot_records[i].arg0);
        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].arg1);
        adc_proto_put_u32_be(payload, &index, s_log_snapshot_records[i].arg2);
    }

    return index;
}

static void adc_tcp_server_handle_read_param(struct tcp_pcb *tpcb,
                                             const uint8_t *payload,
                                             uint16_t payload_len)
{
    adc_param_block_t *block;
    uint16_t block_id;

    SEGGER_RTT_WriteString(0, "proto read param\r\n");
    app_log_record(APP_LOG_EVENT_PROTO_READ_PARAM,
                   0U,
                   payload_len,
                   0U);

    if (2U != payload_len)
    {
        SEGGER_RTT_WriteString(0, "read param bad len\r\n");
        (void)adc_tcp_server_send_read_status(tpcb,
                                              0xFFFFU,
                                              ADC_PROTO_WRITE_STATUS_BAD_LEN);
        return;
    }

    block_id = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    if (ADC_PARAM_BLOCK_LOG_SNAPSHOT == block_id)
    {
        uint16_t log_len;

        log_len = adc_tcp_server_build_log_snapshot(s_log_snapshot_payload,
                                                    sizeof(s_log_snapshot_payload));

        (void)adc_tcp_server_send_fixed_frame(tpcb,
                                              ADC_PROTO_CMD_READ_PARAM,
                                              block_id,
                                              s_log_snapshot_payload,
                                              log_len);
        return;
    }
    block = adc_param_store_find_block(block_id);
    if (NULL == block)
    {
        SEGGER_RTT_WriteString(0, "read param not found\r\n");
        (void)adc_tcp_server_send_read_status(tpcb,
                                              block_id,
                                              ADC_PROTO_WRITE_STATUS_NOT_FOUND);
        return;
    }

    if (block->len > ADC_PROTO_FIXED_DATA_CAPACITY)
    {
        SEGGER_RTT_WriteString(0, "read param too long\r\n");
        (void)adc_tcp_server_send_read_status(tpcb,
                                              block_id,
                                              ADC_PROTO_WRITE_STATUS_TOO_LONG);
        return;
    }

    (void)adc_tcp_server_send_fixed_frame(tpcb,
                                          ADC_PROTO_CMD_READ_PARAM,
                                          block_id,
                                          block->data,
                                          block->len);
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

    adc_proto_put_u32_be(payload, &index, stats.adc1_half_count);
    adc_proto_put_u32_be(payload, &index, stats.adc2_half_count);
    adc_proto_put_u32_be(payload, &index, stats.adc3_half_count);

    adc_proto_put_u32_be(payload, &index, stats.adc1_full_count);
    adc_proto_put_u32_be(payload, &index, stats.adc2_full_count);
    adc_proto_put_u32_be(payload, &index, stats.adc3_full_count);

    adc_proto_put_u32_be(payload, &index, stats.sample_seq);
    adc_proto_put_u32_be(payload, &index, stats.no_block_count);

    adc_proto_put_u32_be(payload, &index, s_tcp_send_fail_count);
    adc_proto_put_u32_be(payload, &index, s_adc_stream_seq);
    adc_proto_put_u16_be(payload, &index, s_tcp_sndbuf_min);

    (void)adc_tcp_server_send_frame(s_client_pcb,
                                    ADC_PROTO_RSP_DEBUG_STATUS,
                                    payload,
                                    index);
}

static void adc_tcp_server_update_sndbuf_min(uint16_t sndbuf_now)
{
    if ((0U == s_tcp_sndbuf_min) || (sndbuf_now < s_tcp_sndbuf_min))
    {
        s_tcp_sndbuf_min = sndbuf_now;
    }
}

static uint8_t adc_tcp_server_can_send_bytes(uint16_t need_len)
{
    uint16_t sndbuf_now;

    if (NULL == s_client_pcb)
    {
        return 0U;
    }

    sndbuf_now = tcp_sndbuf(s_client_pcb);
    adc_tcp_server_update_sndbuf_min(sndbuf_now);

    if (sndbuf_now < (uint16_t)(need_len + ADC_STREAM_TCP_SEND_MARGIN))
    {
        return 0U;
    }

    return 1U;
}

static uint16_t adc_tcp_server_collect_stream_samples(uint16_t max_sample_count)
{
    uint16_t sample_count = 0U;

    while (sample_count < max_sample_count)
    {
        if (0U == adc_acq_service_get_sample(&s_adc_stream_samples[sample_count]))
        {
            break;
        }

        /*
         * ADC-to-DAC cascade must use the same fresh sample that is about
         * to be sent. The DAC service decides whether each channel is in
         * manual mode or cascade mode.
         */
        dac_output_service_apply_adc_sample(&s_adc_stream_samples[sample_count]);

        sample_count++;
    }

    return sample_count;
}

static uint16_t adc_tcp_server_build_stream_payload(uint16_t sample_count)
{
    if (ADC_STREAM_TYPE_CONVERTED == s_adc_stream_type)
    {
        return adc_frame_builder_build_cal_float_batch(s_adc_stream_samples,
                                                       sample_count,
                                                       s_adc_stream_seq,
                                                       0x0FFFU,
                                                       device_config_get_adc_calibration(),
                                                       s_adc_stream_payload,
                                                       sizeof(s_adc_stream_payload));
    }

    return adc_frame_builder_build_raw_u16_batch(s_adc_stream_samples,
                                                 sample_count,
                                                 s_adc_stream_seq,
                                                 0x0FFFU,
                                                 s_adc_stream_payload,
                                                 sizeof(s_adc_stream_payload));
}

static uint16_t adc_tcp_server_get_stream_batch_count(void)
{
    if (ADC_STREAM_TYPE_CONVERTED == s_adc_stream_type)
    {
        return ADC_FRAME_CONVERTED_BATCH_GROUP_COUNT;
    }

    return ADC_FRAME_BATCH_GROUP_COUNT;
}

static void adc_tcp_server_pump_adc_stream(void)
{
    uint16_t payload_len;
    uint16_t frame_len;
    uint8_t frame_count;
    uint16_t sample_count;
    uint16_t max_sample_count;

    if (NULL == s_client_pcb)
    {
        adc_tcp_server_disable_stream();
        return;
    }

    max_sample_count = adc_tcp_server_get_stream_batch_count();

    for (frame_count = 0U;
         frame_count < ADC_STREAM_MAX_FRAMES_PER_PUMP;
         frame_count++)
    {
        if (0U == adc_tcp_server_can_send_bytes((uint16_t)(ADC_FRAME_MAX_BYTES +
                                                           ADC_PROTO_FRAME_OVERHEAD)))
        {
            return;
        }

        // raw 和 converted 每帧样本数不同,raw 12*2*32 converted 12*4*16
        sample_count = adc_tcp_server_collect_stream_samples(max_sample_count);

        if (0U == sample_count)
        {
            return;
        }

        payload_len = adc_tcp_server_build_stream_payload(sample_count);

        if (0U == payload_len)
        {
            return;
        }

        /*
         * Check lwIP send buffer again with the actual frame length.
         * If the buffer is tight, wait for the next main-loop pump.
         */
        frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);

        if (0U == adc_tcp_server_can_send_bytes(frame_len))
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
