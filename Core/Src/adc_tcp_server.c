#include "adc_tcp_server.h"

#include "SEGGER_RTT.h"
#include "lwip.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "device_config.h"
#include "adc_proto.h"
#include "adc_param_store.h"
#include "adc_diag_payload.h"
#include "adc_frame_builder.h"
#include "dac_output_service.h"
#include "app_log.h"

#include <string.h>

/* ========================= TCP Server 工作流程说明 ========================
 *
 * TCP 服务器运行流程：
 * 1. adc_tcp_server_init() 在 8080 端口创建监听 PCB
 * 2. adc_tcp_server_accept() 记录连接的客户端 PCB
 * 3. adc_tcp_server_recv() 将 TCP 字节流复制到 s_rx_buf，然后释放 pbuf
 * 4. adc_tcp_server_parse_rx() 将连续字节流分割成协议帧
 * 5. adc_tcp_server_handle_frame() 分发执行命令
 *
 * PCB 说明：
 * - s_listen_pcb:  监听 PCB，只负责接受客户端连接
 * - s_client_pcb:  客户端 PCB，实际的数据通路
 *
 * 重要说明：
 * - TCP 是字节流协议，一次 recv 可能只有半帧，所以需要缓冲和分帧
 * - 一次 recv 也可能包含多帧，所以接收处理使用 while 循环连续解析
 */

/* ========================= Protocol Definition ========================= */

/** @brief TCP 服务器监听端口号 */
#define ADC_TCP_SERVER_PORT 8080U

/** @brief PC 发送给 MCU 的协议命令 */
#define ADC_PROTO_CMD_WRITE_PARAM 0x01U // 写入参数
#define ADC_PROTO_CMD_READ_PARAM 0x02U  // 读取参数
#define ADC_PROTO_CMD_HEARTBEAT 0x07U   // 心跳包

/** @brief MCU 发送给 PC 的响应帧命令（由协议文档预留） */
#define ADC_PROTO_RSP_RAW_DATA 0x81U       // 原始 ADC 数据流
#define ADC_PROTO_RSP_CONVERTED_DATA 0x82U // 转换后的 ADC 数据流

/** @brief 日志诊断命令 */
#define ADC_LOG_DIAG_ACTION_CLEAR 0x01U              // 清空日志
#define ADC_LOG_DIAG_ACTION_STOP_WATCHDOG_FEED 0xA5U // 停止看门狗喂狗（调试用）

/** @brief 调试命令：写入块 0x000D，DATA=A5，可以验证 IWDG 复位 */
#define ADC_TCP_WATCHDOG_TEST_ENABLE 1U

/** @brief TCP 接收缓冲区大小 */
#define ADC_TCP_RX_BUF_SIZE 1500U

/** @brief 协议最大负载大小 = 缓冲区大小 - 帧开销 */
#define ADC_PROTO_MAX_PAYLOAD_SIZE (ADC_TCP_RX_BUF_SIZE - ADC_PROTO_FRAME_OVERHEAD)

/** @brief 获取数组元素个数 */
#define ADC_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/** @brief ADC 校准参数字节数 = 12 通道 × 2 (k_raw + b_raw) × 4 字节 */
#define ADC_CAL_PARAM_BYTES (DEVICE_CONFIG_ADC_CHANNEL_COUNT * 2U * sizeof(int32_t))

/** @brief ADC 数据流类型定义 */
#define ADC_STREAM_TYPE_RAW ADC_PROTO_RSP_RAW_DATA             // 原始数据
#define ADC_STREAM_TYPE_CONVERTED ADC_PROTO_RSP_CONVERTED_DATA // 转换数据

/** @brief ADC 数据流发送控制参数 */
#define ADC_STREAM_MAX_FRAMES_PER_PUMP 32U // 每次循环最多发送 32 帧
#define ADC_STREAM_TCP_SEND_MARGIN 32U     // TCP 发送缓冲区预留边际（字节）

/** @brief 调试状态帧命令 */
#define ADC_PROTO_RSP_DEBUG_STATUS 0x83U

/** @brief DAC 参数字节数 */
#define DAC_PARAM_BYTES 14U

/** @brief 是否启用调试状态输出（0=禁用，1=启用） */
#define ADC_TCP_DEBUG_STATUS_ENABLE 0U

/* ============================= Module State ============================ */

/** @brief 监听 PCB（只负责接受连接） */
static struct tcp_pcb *s_listen_pcb;

/** @brief 客户端 PCB（实际数据通路） */
static struct tcp_pcb *s_client_pcb;

/**
 * @brief ADC 数据流启用标志
 * @details 1 = 允许周期发送 ADC 数据流到 PC，0 = 停止发送
 */
static uint8_t s_adc_stream_enabled;

/** @brief 当前 ADC 数据流类型（原始或转换） */
static uint8_t s_adc_stream_type = ADC_STREAM_TYPE_RAW;

/** @brief ADC 数据流序列号（用于确保数据完整性） */
static uint32_t s_adc_stream_seq;

/** @brief 上次发送调试状态的时间戳 */
#if ADC_TCP_DEBUG_STATUS_ENABLE
static uint32_t s_debug_status_last_tick;
#endif

/** @brief TCP 发送失败计数（调试用） */
static uint32_t s_tcp_send_fail_count;

/** @brief TCP 发送缓冲区最小值（调试用） */
static uint16_t s_tcp_sndbuf_min;

/**
 * @brief 看门狗喂狗启用标志
 * @details 1 = main.c 正常喂狗，0 = 禁用喂狗（调试用）
 * @note 通过 TCP 命令可以改变此标志，用于验证看门狗复位功能
 */
static uint8_t s_watchdog_feed_enabled = 1U;

/**
 * @brief TCP 接收缓冲区
 * @details 先将 pbuf 中的数据复制到这里，然后释放 pbuf
 * @note TCP 是字节流，需要缓冲后再分帧处理
 */
static uint8_t s_rx_buf[ADC_TCP_RX_BUF_SIZE];
static uint16_t s_rx_len; // 缓冲区中的有效数据长度

/**
 * @brief TCP 发送帧缓冲区
 * @details 用于构建和发送协议帧
 */
static uint8_t s_tx_frame_buf[ADC_TCP_RX_BUF_SIZE];

/**
 * @brief ADC 数据流负载缓冲区
 * @details 存储要发送的 ADC 数据帧内容
 */
static uint8_t s_adc_stream_payload[ADC_FRAME_MAX_BYTES];

/**
 * @brief 固定命令负载缓冲区
 * @details 用于存储读写参数命令的数据
 */
static uint8_t s_fixed_cmd_payload[ADC_PROTO_FIXED_BLOCK_ID_SIZE +
                                   ADC_PROTO_FIXED_DATA_CAPACITY];

/**
 * @brief 诊断负载缓冲区
 * @details 用于存储日志快照、DAC 状态等诊断数据
 */
static uint8_t s_diag_payload[ADC_PROTO_FIXED_DATA_CAPACITY];

/**
 * @brief ADC 数据流采样缓冲区
 * @details 临时存储从 ADC 采集服务读取的采样数据
 */
static adc_acq_sample_t s_adc_stream_samples[ADC_FRAME_BATCH_GROUP_COUNT];

/* ========================== Function Prototypes ======================== */

/* 公共入口点 */
void adc_tcp_server_init(void);
void adc_tcp_server_process(void);
uint8_t adc_tcp_server_has_client(void);

/* TCP 生命周期：监听、连接、断开、错误 */
static err_t adc_tcp_server_accept(void *arg,
                                   struct tcp_pcb *newpcb,
                                   err_t err);
static err_t adc_tcp_server_recv(void *arg,
                                 struct tcp_pcb *tpcb,
                                 struct pbuf *p,
                                 err_t err);
static void adc_tcp_server_error(void *arg, err_t err);
static void adc_tcp_server_close_client(struct tcp_pcb *tpcb);

/* 接收缓冲区操作函数 */
static void adc_tcp_server_store_pbuf(struct tcp_pcb *tpcb, struct pbuf *p);
static void adc_tcp_server_drop_one_rx_byte(void);
static void adc_tcp_server_remove_frame(uint16_t frame_len);

/* TX 协议发送：普通帧、固定 150 字节帧、状态回复 */
static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len);
static err_t adc_tcp_server_send_fixed_frame(struct tcp_pcb *tpcb,
                                             uint8_t cmd,
                                             uint16_t block_id,
                                             const uint8_t *data,
                                             uint16_t data_len);
static err_t adc_tcp_server_send_write_status(struct tcp_pcb *tpcb,
                                              uint16_t block_id,
                                              uint8_t status);
static err_t adc_tcp_server_send_read_status(struct tcp_pcb *tpcb,
                                             uint16_t block_id,
                                             uint8_t status);
static uint8_t adc_tcp_server_is_fixed_cmd(uint8_t cmd);

/* 协议命令处理函数 */
static void adc_tcp_server_apply_control_param(const uint8_t *data,
                                               uint16_t len);
static void adc_tcp_server_start_stream(uint8_t stream_type);
static void adc_tcp_server_stop_stream(void);
static void adc_tcp_server_disable_stream(void);

/* ADC 数据流处理函数 */
static void adc_tcp_server_update_sndbuf_min(uint16_t sndbuf_now);
static uint8_t adc_tcp_server_can_send_bytes(uint16_t need_len);
static uint16_t adc_tcp_server_collect_stream_samples(uint16_t max_sample_count);
static uint16_t adc_tcp_server_build_stream_payload(uint16_t sample_count);
static uint16_t adc_tcp_server_get_stream_batch_count(void);
static void adc_tcp_server_pump_adc_stream(void);

/* 协议解析和命令分发函数 */
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

#if ADC_TCP_DEBUG_STATUS_ENABLE
static void adc_tcp_server_send_debug_status(void);
#endif
/* ============================ Public API =============================== */

/**
 * @brief 初始化 TCP 服务器
 * @details 创建监听 PCB，绑定到 8080 端口，设置接受回调
 * @retval None
 *
 * @note 调用流程：
 *       1. 初始化参数存储模块
 *       2. 创建 TCP PCB
 *       3. 绑定到 ANY 地址和 8080 端口
 *       4. 设为监听状态
 *       5. 注册接受回调函数
 */
void adc_tcp_server_init(void)
{
    struct tcp_pcb *pcb;
    err_t err;

    // ① 初始化参数存储（从配置读取所有参数）
    adc_param_store_init();

    SEGGER_RTT_WriteString(0, "tcp init begin\r\n");

    // ② 创建新的 TCP PCB
    pcb = tcp_new();
    if (NULL == pcb)
    {
        SEGGER_RTT_WriteString(0, "tcp_new failed\r\n");
        return;
    }

    // ③ 绑定到指定端口
    err = tcp_bind(pcb, IP_ADDR_ANY, ADC_TCP_SERVER_PORT);
    if (ERR_OK != err)
    {
        SEGGER_RTT_WriteString(0, "tcp_bind failed\r\n");
        tcp_close(pcb);
        return;
    }

    // ④ 设为监听状态（返回新的 PCB）
    pcb = tcp_listen(pcb);
    if (NULL == pcb)
    {
        SEGGER_RTT_WriteString(0, "tcp_listen failed\r\n");
        return;
    }

    s_listen_pcb = pcb;

    // ⑤ 注册客户端连接接受回调
    tcp_accept(s_listen_pcb, adc_tcp_server_accept);

    SEGGER_RTT_WriteString(0, "tcp listen 8080\r\n");
}

/**
 * @brief TCP 服务器主处理函数
 * @details 在 main() 主循环中频繁调用，处理数据流发送和网络配置
 * @retval None
 *
 * @note 主要工作：
 *       1. 如果启用了 ADC 数据流，发送待发送的数据
 *       2. 如果网络配置被修改但未应用，应用新配置
 *       3. 输出调试状态信息（可选）
 */
void adc_tcp_server_process(void)
{
    // ① 如果 ADC 数据流启用，发送数据流帧
    if (0U != s_adc_stream_enabled)
    {
        adc_tcp_server_pump_adc_stream();
    }

    // ② 如果没有客户端连接，检查和应用网络配置
    if (NULL == s_client_pcb)
    {
        // 检查网络配置是否脏（已修改）或未应用
        if ((0U != adc_param_store_is_network_dirty()) ||
            (0U == MX_LWIP_IsNetworkConfigApplied()))
        {
            // 应用新的网络配置
            MX_LWIP_ApplyNetworkConfig();
            adc_param_store_clear_network_dirty();
            SEGGER_RTT_WriteString(0, "netif param applied\r\n");
            app_log_record(APP_LOG_EVENT_NETIF_APPLIED,
                           0U,
                           0U,
                           0U);
        }
    }

    // ③ 可选：输出调试状态信息
#if ADC_TCP_DEBUG_STATUS_ENABLE
    adc_tcp_server_send_debug_status();
#endif
}

/* ==================== Public Query Functions ==================== */

/**
 * @brief 检查是否有客户端连接
 * @retval uint8_t
 *         1 = 有客户端连接
 *         0 = 无客户端连接
 */
uint8_t adc_tcp_server_has_client(void)
{
    return (NULL != s_client_pcb) ? 1U : 0U;
}

/**
 * @brief 检查是否正在进行 ADC 数据流传输
 * @retval uint8_t
 *         1 = 正在流传输
 *         0 = 未流传输
 */
uint8_t adc_tcp_server_is_streaming(void)
{
    return (0U != s_adc_stream_enabled) ? 1U : 0U;
}

/**
 * @brief 检查看门狗喂狗是否启用
 * @details 通过 TCP 命令可以禁用看门狗喂狗（调试用）
 * @retval uint8_t
 *         1 = 喂狗启用，main.c 正常刷新看门狗
 *         0 = 喂狗禁用，main.c 不刷新看门狗（用于测试看门狗复位）
 */
uint8_t adc_tcp_server_is_watchdog_feed_enabled(void)
{
    return (0U != s_watchdog_feed_enabled) ? 1U : 0U;
}

/**
 * @brief 检查网络配置是否需要应用
 * @retval uint8_t
 *         1 = 网络配置已修改，需要应用
 *         0 = 网络配置已同步
 */
uint8_t adc_tcp_server_is_network_config_dirty(void)
{
    return adc_param_store_is_network_dirty();
}

/* =========================== lwIP Callbacks ============================ */

/**
 * @brief TCP 客户端接受回调
 * @details 当有新的客户端连接时由 lwIP 调用
 * @param[in] arg:    用户参数（此处未使用）
 * @param[in] newpcb: 新连接的 PCB
 * @param[in] err:    错误码
 * @retval err_t 错误码
 *
 * @note 工作步骤：
 *       1. 验证参数和错误码
 *       2. 如果已有客户端连接，拒绝新连接
 *       3. 否则记录新客户端 PCB
 *       4. 初始化接收状态
 *       5. 注册接收和错误回调
 */
static err_t adc_tcp_server_accept(void *arg,
                                   struct tcp_pcb *newpcb,
                                   err_t err)
{
    LWIP_UNUSED_ARG(arg);

    // ① 验证参数
    if ((ERR_OK != err) || (NULL == newpcb))
    {
        return ERR_VAL;
    }

    // ② 如果已有客户端连接，拒绝新连接
    if (NULL != s_client_pcb)
    {
        tcp_close(newpcb);
        return ERR_OK;
    }

    // ③ 记录新客户端 PCB
    s_client_pcb = newpcb;

    // ④ 记录当前发送缓冲区大小，用于监控
    s_tcp_sndbuf_min = tcp_sndbuf(newpcb);

    // ⑤ 禁用之前的数据流（如果有的话）
    adc_tcp_server_disable_stream();

    // ⑥ 清空接收缓冲区
    s_rx_len = 0U;

    // ⑦ 注册回调函数
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, adc_tcp_server_recv); // 接收回调
    tcp_err(newpcb, adc_tcp_server_error); // 错误回调

    SEGGER_RTT_WriteString(0, "tcp accept\r\n");
    app_log_record(APP_LOG_EVENT_TCP_ACCEPT,
                   0U,
                   0U,
                   0U);

    return ERR_OK;
}

/**
 * @brief TCP 数据接收回调
 * @details 当有数据到达时由 lwIP 调用
 * @param[in] arg:   用户参数（此处未使用）
 * @param[in] tpcb:  TCP PCB
 * @param[in] p:     数据包（pbuf 链表）
 * @param[in] err:   错误码
 * @retval err_t 错误码
 *
 * @note 工作步骤：
 *       1. 验证参数
 *       2. 如果 p 为 NULL，说明连接断开
 *       3. 否则将 pbuf 数据复制到接收缓冲区
 *       4. 释放 pbuf，告知 lwIP 已接收
 *       5. 解析接收缓冲区中的协议帧
 */
static err_t adc_tcp_server_recv(void *arg,
                                 struct tcp_pcb *tpcb,
                                 struct pbuf *p,
                                 err_t err)
{
    LWIP_UNUSED_ARG(arg);

    // ① 验证参数
    if ((ERR_OK != err) || (NULL == tpcb))
    {
        if (NULL != p)
        {
            pbuf_free(p);
        }
        return ERR_OK;
    }

    // ② 如果 p 为 NULL，连接已断开
    if (NULL == p)
    {
        adc_tcp_server_close_client(tpcb);
        return ERR_OK;
    }

    // ③ 将 pbuf 数据复制到接收缓冲区
    adc_tcp_server_store_pbuf(tpcb, p);

    // ④ 解析接收缓冲区中的协议帧
    adc_tcp_server_parse_rx(tpcb);

    return ERR_OK;
}

/**
 * @brief TCP 错误回调
 * @details 当连接出错时由 lwIP 调用
 * @param[in] arg: 用户参数（此处未使用）
 * @param[in] err: 错误码
 * @retval None
 *
 * @note 错误通常意味着连接已断开
 */
static void adc_tcp_server_error(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    // 清除客户端 PCB
    s_client_pcb = NULL;

    // 禁用数据流
    adc_tcp_server_disable_stream();

    // 清空接收缓冲区
    s_rx_len = 0U;

    // 记录错误日志
    app_log_record(APP_LOG_EVENT_TCP_ERROR,
                   (uint16_t)err,
                   0U,
                   0U);
}

/* ========================= Connection Helpers ========================== */

/**
 * @brief 关闭客户端连接
 * @param[in] tpcb: TCP PCB
 * @retval None
 *
 * @note 工作步骤：
 *       1. 注销所有回调函数
 *       2. 如果是当前客户端，清除状态
 *       3. 关闭或中止 TCP 连接
 */
static void adc_tcp_server_close_client(struct tcp_pcb *tpcb)
{
    if (NULL == tpcb)
    {
        return;
    }

    // ① 注销所有回调函数
    tcp_arg(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_err(tpcb, NULL);

    // ② 如果是当前客户端，清除状态
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

    // ③ 关闭连接
    if (ERR_OK != tcp_close(tpcb))
    {
        // 如果 close 失败（连接还有数据待发送），强制中止
        tcp_abort(tpcb);
    }
}

/* ======================== Receive Buffer Helpers ======================= */

/**
 * @brief 将 pbuf 数据复制到接收缓冲区
 * @param[in] tpcb: TCP PCB（用于通知接收）
 * @param[in] p:    数据包
 * @retval None
 *
 * @note 工作步骤：
 *       1. 检查接收缓冲区是否有足够空间
 *       2. 如果空间不足，清空缓冲区并记录溢出
 *       3. 否则将 pbuf 数据复制到缓冲区
 *       4. 通知 lwIP 已接收数据
 *       5. 释放 pbuf
 */
static void adc_tcp_server_store_pbuf(struct tcp_pcb *tpcb, struct pbuf *p)
{
    uint16_t copy_len;

    copy_len = p->tot_len;

    // ① 检查缓冲区溢出
    if ((s_rx_len + copy_len) > ADC_TCP_RX_BUF_SIZE)
    {
        uint16_t old_rx_len;

        old_rx_len = s_rx_len;
        s_rx_len = 0U; // 清空缓冲区

        SEGGER_RTT_WriteString(0, "rx buf overflow\r\n");

        // 记录溢出信息
        app_log_record(APP_LOG_EVENT_TCP_RX_OVERFLOW,
                       0U,
                       old_rx_len,
                       copy_len);
    }
    else
    {
        // ② 复制数据
        pbuf_copy_partial(p, &s_rx_buf[s_rx_len], copy_len, 0U);
        s_rx_len = (uint16_t)(s_rx_len + copy_len);
    }

    // ③ 通知 lwIP 已接收数据
    tcp_recved(tpcb, p->tot_len);

    // ④ 释放 pbuf
    pbuf_free(p);
}

/**
 * @brief 丢弃接收缓冲区的第一个字节
 * @details 用于跳过无效字节，重新同步帧边界
 * @retval None
 */
static void adc_tcp_server_drop_one_rx_byte(void)
{
    if (s_rx_len > 0U)
    {
        // 将后续数据前移一个字节
        memmove(&s_rx_buf[0], &s_rx_buf[1], s_rx_len - 1U);
        s_rx_len--;
    }
}

/**
 * @brief 从接收缓冲区移除已处理的帧
 * @param[in] frame_len: 要移除的帧长度
 * @retval None
 */
static void adc_tcp_server_remove_frame(uint16_t frame_len)
{
    if (frame_len >= s_rx_len)
    {
        // 帧长度超过或等于缓冲区内容，清空缓冲区
        s_rx_len = 0U;
        return;
    }

    // 将后续数据前移
    memmove(s_rx_buf, &s_rx_buf[frame_len], s_rx_len - frame_len);
    s_rx_len = (uint16_t)(s_rx_len - frame_len);
}

/* =========================== Protocol Parser =========================== */

/**
 * @brief 构建并发送协议帧
 * @param[in] tpcb:        TCP PCB
 * @param[in] cmd:         命令字节
 * @param[in] payload:     负载数据
 * @param[in] payload_len: 负载长度
 * @retval err_t 错误码
 *
 * @note 帧格式：[SOF0] [SOF1] [CMD] [LEN_H] [LEN_L] [PAYLOAD] [CRC32] [EOF0] [EOF1]
 *       本函数自动计算 CRC32 和添加帧边界
 */
static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len)
{
    uint8_t *frame = s_tx_frame_buf;
    uint16_t frame_len;
    err_t err;

    // ① 验证参数
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

    // ② 检查 TCP 发送缓冲区是否有足够空间
    frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);

    if (tcp_sndbuf(tpcb) < frame_len)
    {
        return ERR_MEM; // 发送缓冲区满
    }

    // ③ 构建协议帧
    frame_len = adc_proto_build_frame(frame,
                                      sizeof(s_tx_frame_buf),
                                      cmd,
                                      payload,
                                      payload_len);

    if (0U == frame_len)
    {
        return ERR_VAL; // 构建失败
    }

    // ④ 再次检查缓冲区
    if (tcp_sndbuf(tpcb) < frame_len)
    {
        return ERR_MEM;
    }

    // ⑤ 发送数据
    err = tcp_write(tpcb, frame, frame_len, TCP_WRITE_FLAG_COPY);
    if (ERR_OK != err)
    {
        return err;
    }

    // ⑥ 触发输出（可能合并多个 tcp_write 的数据）
    return tcp_output(tpcb);
}

/**
 * @brief 构建并发送固定格式协议帧
 * @details 用于读写参数、心跳等固定格式命令
 * @param[in] tpcb:     TCP PCB
 * @param[in] cmd:      命令字节
 * @param[in] block_id: 参数块 ID
 * @param[in] data:     参数数据
 * @param[in] data_len: 数据长度
 * @retval err_t 错误码
 *
 * @note 固定帧大小为 150 字节，包含 block_id 和数据
 */
static err_t adc_tcp_server_send_fixed_frame(struct tcp_pcb *tpcb,
                                             uint8_t cmd,
                                             uint16_t block_id,
                                             const uint8_t *data,
                                             uint16_t data_len)
{
    uint16_t frame_len;
    err_t err;

    // ① 验证参数
    if (NULL == tpcb)
    {
        return ERR_ARG;
    }

    // ② 构建固定格式帧
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

    // ③ 检查 TCP 发送缓冲区
    if (tcp_sndbuf(tpcb) < frame_len)
    {
        return ERR_MEM;
    }

    // ④ 发送数据
    err = tcp_write(tpcb, s_tx_frame_buf, frame_len, TCP_WRITE_FLAG_COPY);
    if (ERR_OK != err)
    {
        return err;
    }

    // ⑤ 触发输出
    return tcp_output(tpcb);
}

/**
 * @brief 检查命令是否为固定格式命令
 * @param[in] cmd: 命令字节
 * @retval uint8_t
 *         1 = 是固定格式命令
 *         0 = 不是固定格式命令
 *
 * @note 固定格式命令有：WRITE_PARAM、READ_PARAM、HEARTBEAT
 */
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

/**
 * @brief 发送写入参数响应
 * @param[in] tpcb:     TCP PCB
 * @param[in] block_id: 参数块 ID
 * @param[in] status:   写入状态码
 * @retval err_t 错误码
 */
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

/**
 * @brief 发送读取参数响应
 * @param[in] tpcb:     TCP PCB
 * @param[in] block_id: 参数块 ID
 * @param[in] status:   读取状态码
 * @retval err_t 错误码
 */
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

/**
 * @brief 启动 ADC 数据流
 * @param[in] stream_type: 数据流类型（原始或转换）
 * @retval None
 */
static void adc_tcp_server_start_stream(uint8_t stream_type)
{
    if (ADC_STREAM_TYPE_CONVERTED == stream_type)
    {
        s_adc_stream_type = ADC_STREAM_TYPE_CONVERTED;
    }
    else
    {
        s_adc_stream_type = ADC_STREAM_TYPE_RAW;
    }
    s_adc_stream_seq = 0U; // 重置序列号
    s_adc_stream_enabled = 1U;

    SEGGER_RTT_WriteString(0, "adc stream start\r\n");
    app_log_record(APP_LOG_EVENT_ADC_STREAM_START,
                   0U,
                   0U,
                   0U);
}

/**
 * @brief 停止 ADC 数据流
 * @retval None
 */
static void adc_tcp_server_stop_stream(void)
{
    adc_tcp_server_disable_stream();

    SEGGER_RTT_WriteString(0, "adc stream stop\r\n");
    app_log_record(APP_LOG_EVENT_ADC_STREAM_STOP,
                   0U,
                   0U,
                   0U);
}

/**
 * @brief 禁用 ADC 数据流（内部函数）
 * @retval None
 */
static void adc_tcp_server_disable_stream(void)
{
    s_adc_stream_enabled = 0U;
}

/**
 * @brief 应用控制参数
 * @details 根据 CONTROL 参数块启动或停止 ADC 数据流
 * @param[in] data: 控制参数数据
 * @param[in] len:  数据长度
 * @retval None
 *
 * @note 格式：[启用标志] [流类型]
 *       - 启用标志：非 0 = 启动，0 = 停止
 *       - 流类型：0x81 = 原始，0x82 = 转换
 */
static void adc_tcp_server_apply_control_param(const uint8_t *data,
                                               uint16_t len)
{
    if ((NULL == data) || (len < 2U))
    {
        return;
    }

    if (0U != data[0])
    {
        // 启动数据流，使用指定的流类型
        adc_tcp_server_start_stream(data[1]);
    }
    else
    {
        // 停止数据流
        adc_tcp_server_stop_stream();
    }
}

/**
 * @brief 解析接收缓冲区中的协议帧
 * @details 从接收缓冲区中识别和提取完整帧，然后调用命令处理函数
 * @param[in] tpcb: TCP PCB（用于发送响应）
 * @retval None
 *
 * @note 工作流程：
 *       1. 检查缓冲区中是否有足够的字节构成帧头
 *       2. 查找帧起始符（SOF0, SOF1）
 *       3. 如果是固定格式帧，提取 block_id 和数据
 *       4. 验证帧的完整性和 CRC
 *       5. 调用命令处理函数
 *       6. 从缓冲区移除已处理的帧
 *       7. 如果缓冲区仍有数据，继续处理下一帧
 */
static void adc_tcp_server_parse_rx(struct tcp_pcb *tpcb)
{
    uint16_t frame_len;

    /*
     * TCP 是连续字节流，所以：
     * - 一次 recv 可能只有半帧，需要等待下一个 recv
     * - 一次 recv 也可能包含多帧，所以使用 while 循环
     */
    while (s_rx_len >= ADC_PROTO_MIN_FRAME_SIZE)
    {
        // ① 检查帧起始符
        if ((s_rx_buf[0] != ADC_PROTO_SOF0) ||
            (s_rx_buf[1] != ADC_PROTO_SOF1))
        {
            // 起始符不匹配，丢弃第一个字节，重新同步
            adc_tcp_server_drop_one_rx_byte();
            continue;
        }

        // ② 检查是否为固定格式命令
        if (0U != adc_tcp_server_is_fixed_cmd(s_rx_buf[2]))
        {
            uint16_t data_len;
            uint16_t block_id;
            uint8_t bad_cmd;

            // 固定格式帧大小为 150 字节
            frame_len = ADC_PROTO_FIXED_FRAME_SIZE;

            // 检查缓冲区中是否有完整帧
            if (s_rx_len < frame_len)
            {
                return; // 等待更多数据
            }

            bad_cmd = s_rx_buf[2];

            // 验证固定帧的完整性（检查 CRC、EOF 等）
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

            // 提取数据长度和参数块 ID
            data_len = adc_proto_fixed_data_len(s_rx_buf);
            block_id = adc_proto_fixed_block_id(s_rx_buf);

            // 检查提取的数据是否超过缓冲区大小
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

            // 从固定帧中提取 block_id（大端序）
            s_fixed_cmd_payload[0] = (uint8_t)(block_id >> 8);
            s_fixed_cmd_payload[1] = (uint8_t)(block_id & 0xFFU);

            // 从固定帧中提取数据
            if (data_len > 0U)
            {
                memcpy(&s_fixed_cmd_payload[2],
                       &s_rx_buf[ADC_PROTO_FIXED_DATA_OFFSET + ADC_PROTO_FIXED_BLOCK_ID_SIZE],
                       data_len);
            }

            // 调用命令处理函数
            adc_tcp_server_handle_frame(tpcb,
                                        s_rx_buf[2],
                                        s_fixed_cmd_payload,
                                        (uint16_t)(data_len + ADC_PROTO_FIXED_BLOCK_ID_SIZE));

            // 从接收缓冲区移除已处理的帧
            adc_tcp_server_remove_frame(frame_len);
            continue;
        }

        /*
         * 现在所有上位机命令都应使用固定 150 字节帧。
         * 如果出现未知 cmd，不能继续按 LEN 等待，
         * 否则一条错误短帧会把后续正确命令全部堵住。
         * 所以直接丢弃此字节。
         */
        adc_tcp_server_drop_one_rx_byte();
        SEGGER_RTT_WriteString(0, "proto drop unknown rx cmd\r\n");
        continue;
    }
}

/* ========================== Command Dispatch =========================== */

/**
 * @brief 处理解析完毕的协议帧
 * @details 根据命令类型分发到对应的处理函数
 * @param[in] tpcb:        TCP PCB（用于发送响应）
 * @param[in] cmd:         命令字节
 * @param[in] payload:     负载数据（包含 block_id）
 * @param[in] payload_len: 负载长度
 * @retval None
 */
static void adc_tcp_server_handle_frame(struct tcp_pcb *tpcb,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t payload_len)
{
    // ① 心跳命令处理
    if (ADC_PROTO_CMD_HEARTBEAT == cmd)
    {
        uint8_t status = ADC_PROTO_WRITE_STATUS_OK;

        SEGGER_RTT_WriteString(0, "proto heartbeat\r\n");
        app_log_record(APP_LOG_EVENT_PROTO_HEARTBEAT,
                       0U,
                       0U,
                       0U);

        // 回复心跳
        (void)adc_tcp_server_send_fixed_frame(tpcb,
                                              ADC_PROTO_CMD_HEARTBEAT,
                                              0U,
                                              &status,
                                              sizeof(status));
    }
    // ② 写入参数命令处理
    else if (ADC_PROTO_CMD_WRITE_PARAM == cmd)
    {
        adc_tcp_server_handle_write_param(tpcb, payload, payload_len);
    }
    // ③ 读取参数命令处理
    else if (ADC_PROTO_CMD_READ_PARAM == cmd)
    {
        adc_tcp_server_handle_read_param(tpcb, payload, payload_len);
    }
    // ④ 未知命令
    else
    {
        SEGGER_RTT_WriteString(0, "proto unknown cmd\r\n");
    }
}

/**
 * @brief 处理日志清空命令
 * @param[in] tpcb:        TCP PCB
 * @param[in] payload:     负载数据
 * @param[in] payload_len: 负载长度
 * @retval None
 */
static void adc_tcp_server_handle_log_clear(struct tcp_pcb *tpcb,
                                            const uint8_t *payload,
                                            uint16_t payload_len)
{
    uint16_t block_id;
    uint8_t status;

    block_id = ADC_PARAM_BLOCK_LOG_SNAPSHOT;
    status = ADC_PROTO_WRITE_STATUS_BAD_LEN;

    // ① 检查清空日志命令
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
    // ② 调试命令：停止看门狗喂狗
#if ADC_TCP_WATCHDOG_TEST_ENABLE
    else if ((3U == payload_len) &&
             (ADC_LOG_DIAG_ACTION_STOP_WATCHDOG_FEED == payload[2]))
    {
        s_watchdog_feed_enabled = 0U; // 禁用看门狗喂狗
        app_log_record(APP_LOG_EVENT_WATCHDOG_TEST,
                       payload[2],
                       0U,
                       0U);
        status = ADC_PROTO_WRITE_STATUS_OK;
    }
#endif

    // 回复状态
    (void)adc_tcp_server_send_write_status(tpcb,
                                           block_id,
                                           status);

    app_log_record(APP_LOG_EVENT_PARAM_WRITE_RESULT,
                   status,
                   block_id,
                   (payload_len >= 2U) ? (uint32_t)(payload_len - 2U) : 0U);
}

/**
 * @brief 处理写入参数命令
 * @param[in] tpcb:        TCP PCB
 * @param[in] payload:     负载数据（block_id + 参数）
 * @param[in] payload_len: 负载长度
 * @retval None
 *
 * @note 工作步骤：
 *       1. 提取 block_id
 *       2. 查找参数块
 *       3. 验证数据长度
 *       4. 根据参数块类型调用相应的应用函数
 *       5. 发送写入结果
 */
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

    // ① 验证负载长度（至少需要 2 字节的 block_id）
    if (payload_len < 2U)
    {
        SEGGER_RTT_WriteString(0, "write param bad len\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               0xFFFFU,
                                               ADC_PROTO_WRITE_STATUS_BAD_LEN);
        return;
    }

    // ② 从负载中提取 block_id（大端序）
    block_id = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);

    // ③ 特殊处理：日志清空命令
    if (ADC_PARAM_BLOCK_LOG_SNAPSHOT == block_id)
    {
        adc_tcp_server_handle_log_clear(tpcb, payload, payload_len);
        return;
    }

    // ④ 在参数表中查找对应的参数块
    block = adc_param_store_find_block(block_id);
    if (NULL == block)
    {
        SEGGER_RTT_WriteString(0, "write param not found\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               block_id,
                                               ADC_PROTO_WRITE_STATUS_NOT_FOUND);
        return;
    }

    // ⑤ 计算写入数据长度（去掉 block_id）
    write_len = (uint16_t)(payload_len - 2U);

    // ⑥ 检查写入长度是否超过参数块最大大小
    if (write_len > block->max_len)
    {
        SEGGER_RTT_WriteString(0, "write param too long\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               block_id,
                                               ADC_PROTO_WRITE_STATUS_TOO_LONG);
        return;
    }

    // ⑦ 检查写入数据的格式是否合法
    write_status = adc_param_store_check_write_len(block_id, write_len);
    if (ADC_PROTO_WRITE_STATUS_OK != write_status)
    {
        SEGGER_RTT_WriteString(0, "write param bad len\r\n");
        (void)adc_tcp_server_send_write_status(tpcb,
                                               block_id,
                                               write_status);
        return;
    }

    // ⑧ 复制写入数据到参数块
    if (write_len > 0U)
    {
        memcpy(block->data, &payload[2], write_len);
    }
    block->len = write_len;

    // ⑨ 根据参数块类型调用相应的应用函数
    if (ADC_PARAM_BLOCK_CONTROL == block_id)
    {
        // 控制参数：启停 ADC 数据流
        adc_tcp_server_apply_control_param(block->data, block->len);
        write_status = ADC_PROTO_WRITE_STATUS_OK;
    }
    else if (ADC_PARAM_BLOCK_CAL_DATA == block_id)
    {
        // ADC 校准参数
        write_status = adc_param_store_apply_cal_param(block->data, block->len);
    }
    else if ((ADC_PARAM_BLOCK_DA_CH1 <= block_id) &&
             (ADC_PARAM_BLOCK_DA_CH4 >= block_id))
    {
        // DAC 参数
        write_status = adc_param_store_apply_dac_param(block_id,
                                                       block->data,
                                                       block->len);
    }
    else
    {
        // 网络参数（IP、MAC、端口等）
        write_status = adc_param_store_apply_network_param(block_id,
                                                           block->data,
                                                           block->len);
    }

    // ⑩ 回复写入结果
    (void)adc_tcp_server_send_write_status(tpcb,
                                           block_id,
                                           write_status);

    app_log_record(APP_LOG_EVENT_PARAM_WRITE_RESULT,
                   write_status,
                   block_id,
                   write_len);
}

/**
 * @brief 处理读取参数命令
 * @param[in] tpcb:        TCP PCB
 * @param[in] payload:     负载数据（block_id）
 * @param[in] payload_len: 负载长度
 * @retval None
 *
 * @note 工作步骤：
 *       1. 验证负载长度
 *       2. 提取 block_id
 *       3. 特殊处理诊断参数（DAC 状态、日志快照）
 *       4. 在参数表中查找参数块
 *       5. 发送参数数据
 */
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

    // ① 验证负载长度（应该是 2 字节的 block_id）
    if (2U != payload_len)
    {
        SEGGER_RTT_WriteString(0, "read param bad len\r\n");
        (void)adc_tcp_server_send_read_status(tpcb,
                                              0xFFFFU,
                                              ADC_PROTO_WRITE_STATUS_BAD_LEN);
        return;
    }

    // ② 提取 block_id（大端序）
    block_id = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);

    // ③ 特殊处理：DAC 状态诊断
    if (ADC_PARAM_BLOCK_DAC_STATUS == block_id)
    {
        uint16_t status_len;

        // 构建 DAC 状态数据
        status_len = adc_diag_payload_build_dac_status(s_diag_payload,
                                                       sizeof(s_diag_payload));

        // 发送 DAC 状态
        (void)adc_tcp_server_send_fixed_frame(tpcb,
                                              ADC_PROTO_CMD_READ_PARAM,
                                              block_id,
                                              s_diag_payload,
                                              status_len);
        return;
    }

    // ④ 特殊处理：日志快照诊断
    if (ADC_PARAM_BLOCK_LOG_SNAPSHOT == block_id)
    {
        uint16_t log_len;

        // 构建日志快照数据
        log_len = adc_diag_payload_build_log_snapshot(s_diag_payload,
                                                      sizeof(s_diag_payload));

        // 发送日志快照
        (void)adc_tcp_server_send_fixed_frame(tpcb,
                                              ADC_PROTO_CMD_READ_PARAM,
                                              block_id,
                                              s_diag_payload,
                                              log_len);
        return;
    }

    // ⑤ 在参数表中查找参数块
    block = adc_param_store_find_block(block_id);
    if (NULL == block)
    {
        SEGGER_RTT_WriteString(0, "read param not found\r\n");
        (void)adc_tcp_server_send_read_status(tpcb,
                                              block_id,
                                              ADC_PROTO_WRITE_STATUS_NOT_FOUND);
        return;
    }

    // ⑥ 检查参数数据大小是否超过响应帧容量
    if (block->len > ADC_PROTO_FIXED_DATA_CAPACITY)
    {
        SEGGER_RTT_WriteString(0, "read param too long\r\n");
        (void)adc_tcp_server_send_read_status(tpcb,
                                              block_id,
                                              ADC_PROTO_WRITE_STATUS_TOO_LONG);
        return;
    }

    // ⑦ 发送参数数据
    (void)adc_tcp_server_send_fixed_frame(tpcb,
                                          ADC_PROTO_CMD_READ_PARAM,
                                          block_id,
                                          block->data,
                                          block->len);
}

/**
 * @brief 发送调试状态信息
 * @details 定期发送 ADC、TCP 的统计信息到 PC（可选功能）
 * @retval None
 *
 * @note 仅在 ADC_TCP_DEBUG_STATUS_ENABLE == 1 时调用
 *       每 1 秒发送一次统计信息
 */
#if ADC_TCP_DEBUG_STATUS_ENABLE
static void adc_tcp_server_send_debug_status(void)
{
    adc_acq_stats_t stats;
    uint8_t payload[64];
    uint16_t index = 0U;
    uint32_t now_tick;

    // ① 检查是否有客户端连接
    if (NULL == s_client_pcb)
    {
        return;
    }

    // ② 获取当前时间戳
    now_tick = HAL_GetTick();

    // ③ 检查是否已到发送时间（每 1000ms 发送一次）
    if ((now_tick - s_debug_status_last_tick) < 1000U)
    {
        return;
    }

    s_debug_status_last_tick = now_tick;

    // ④ 获取 ADC 采集统计信息
    adc_acq_service_get_stats(&stats);

    // ⑤ 构建负载
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

    // ⑥ 发送调试状态帧
    (void)adc_tcp_server_send_frame(s_client_pcb,
                                    ADC_PROTO_RSP_DEBUG_STATUS,
                                    payload,
                                    index);
}
#endif

/**
 * @brief 更新 TCP 发送缓冲区最小值
 * @details 用于监控 TCP 发送缓冲区是否接近满
 * @param[in] sndbuf_now: 当前发送缓冲区空闲大小
 * @retval None
 */
static void adc_tcp_server_update_sndbuf_min(uint16_t sndbuf_now)
{
    if ((0U == s_tcp_sndbuf_min) || (sndbuf_now < s_tcp_sndbuf_min))
    {
        s_tcp_sndbuf_min = sndbuf_now;
    }
}

/**
 * @brief 检查 TCP 发送缓冲区是否有足够空间
 * @param[in] need_len: 需要的字节数
 * @retval uint8_t
 *         1 = 缓冲区有足够空间
 *         0 = 缓冲区空间不足
 *
 * @note 会预留 ADC_STREAM_TCP_SEND_MARGIN 字节的安全边际
 */
static uint8_t adc_tcp_server_can_send_bytes(uint16_t need_len)
{
    uint16_t sndbuf_now;

    // ① 检查是否有客户端
    if (NULL == s_client_pcb)
    {
        return 0U;
    }

    // ② 获取当前缓冲区剩余大小
    sndbuf_now = tcp_sndbuf(s_client_pcb);

    // ③ 更新最小值（用于调试）
    adc_tcp_server_update_sndbuf_min(sndbuf_now);

    // ④ 检查是否有足够空间（需要加上安全边际）
    if (sndbuf_now < (uint16_t)(need_len + ADC_STREAM_TCP_SEND_MARGIN))
    {
        return 0U; // 空间不足
    }

    return 1U; // 空间充足
}

/**
 * @brief 从 ADC 采集服务收集采样数据
 * @details 读取最新的 ADC 采样，并应用到 DAC 级联模式
 * @param[in] max_sample_count: 最多收集的采样数
 * @retval uint16_t 实际收集的采样数
 *
 * @note 重要：此函数同时处理 ADC-to-DAC 级联
 *       - 读取最新的 ADC 采样
 *       - 应用到 DAC 输出服务（如果 DAC 通道配置为级联模式）
 */
static uint16_t adc_tcp_server_collect_stream_samples(uint16_t max_sample_count)
{
    uint16_t sample_count = 0U;

    while (sample_count < max_sample_count)
    {
        // ① 尝试读取一个采样
        if (0U == adc_acq_service_get_sample(&s_adc_stream_samples[sample_count]))
        {
            break; // 无可用采样
        }

        /*
         * ② ADC-to-DAC 级联：立即将采样应用到 DAC 输出
         * DAC 服务会判断每个通道是否在级联模式
         * - 如果在级联模式：输出该 ADC 通道的值
         * - 如果在手动模式：保持手动设置的输出值
         */
        dac_output_service_apply_adc_sample(&s_adc_stream_samples[sample_count]);

        sample_count++;
    }

    return sample_count;
}

/**
 * @brief 构建 ADC 数据流负载
 * @details 根据流类型（原始或转换），构建要发送的 ADC 数据帧
 * @param[in] sample_count: 采样数
 * @retval uint16_t 构建的负载大小（字节）
 *
 * @note 原始数据流和转换数据流的每帧采样数不同：
 *       - 原始（12 channels × 2 bytes）：32 样本/帧
 *       - 转换（12 channels × 4 bytes）：16 样本/帧
 */
static uint16_t adc_tcp_server_build_stream_payload(uint16_t sample_count)
{
    if (ADC_STREAM_TYPE_CONVERTED == s_adc_stream_type)
    {
        // 构建转换数据流（浮点格式）
        return adc_frame_builder_build_cal_float_batch(s_adc_stream_samples,
                                                       sample_count,
                                                       s_adc_stream_seq,
                                                       0x0FFFU,
                                                       device_config_get_adc_calibration(),
                                                       s_adc_stream_payload,
                                                       sizeof(s_adc_stream_payload));
    }

    // 构建原始数据流（16 位整数格式）
    return adc_frame_builder_build_raw_u16_batch(s_adc_stream_samples,
                                                 sample_count,
                                                 s_adc_stream_seq,
                                                 0x0FFFU,
                                                 s_adc_stream_payload,
                                                 sizeof(s_adc_stream_payload));
}

/**
 * @brief 获取 ADC 数据流每批的采样数
 * @retval uint16_t 每批采样数
 *
 * @note 原始和转换格式的批大小不同
 */
static uint16_t adc_tcp_server_get_stream_batch_count(void)
{
    if (ADC_STREAM_TYPE_CONVERTED == s_adc_stream_type)
    {
        return ADC_FRAME_CONVERTED_BATCH_GROUP_COUNT;
    }

    return ADC_FRAME_BATCH_GROUP_COUNT;
}

/**
 * @brief 发送 ADC 数据流
 * @details 在主循环中频繁调用，将缓冲的 ADC 数据发送给 PC
 * @retval None
 *
 * @note 工作流程：
 *       1. 检查是否有客户端连接
 *       2. 计算单次可发送的最大采样数
 *       3. 循环收集采样、构建帧、发送
 *       4. 当 TCP 缓冲区满或无采样时停止
 *       5. 更新流序列号
 */
static void adc_tcp_server_pump_adc_stream(void)
{
    uint16_t payload_len;
    uint16_t frame_len;
    uint8_t frame_count;
    uint16_t sample_count;
    uint16_t max_sample_count;

    // ① 检查是否有客户端
    if (NULL == s_client_pcb)
    {
        adc_tcp_server_disable_stream();
        return;
    }

    // ② 计算单次可发送的最大采样数
    max_sample_count = adc_tcp_server_get_stream_batch_count();

    // ③ 循环发送多个帧（每次最多 ADC_STREAM_MAX_FRAMES_PER_PUMP 帧）
    for (frame_count = 0U;
         frame_count < ADC_STREAM_MAX_FRAMES_PER_PUMP;
         frame_count++)
    {
        // ④ 检查 TCP 缓冲区是否有足够空间发送一个完整帧
        if (0U == adc_tcp_server_can_send_bytes((uint16_t)(ADC_FRAME_MAX_BYTES +
                                                           ADC_PROTO_FRAME_OVERHEAD)))
        {
            return; // 缓冲区满，停止发送
        }

        // ⑤ 收集采样数据
        sample_count = adc_tcp_server_collect_stream_samples(max_sample_count);

        if (0U == sample_count)
        {
            return; // 无采样，停止发送
        }

        // ⑥ 构建数据流负载
        payload_len = adc_tcp_server_build_stream_payload(sample_count);

        if (0U == payload_len)
        {
            return; // 构建失败，停止发送
        }

        /*
         * ⑦ 再次检查 TCP 缓冲区，使用实际帧长度
         * 如果缓冲区紧张，等待下一次主循环泵
         */
        frame_len = (uint16_t)(payload_len + ADC_PROTO_FRAME_OVERHEAD);

        if (0U == adc_tcp_server_can_send_bytes(frame_len))
        {
            return;
        }

        // ⑧ 发送外层协议帧
        if (ERR_OK != adc_tcp_server_send_frame(s_client_pcb,
                                                s_adc_stream_type,
                                                s_adc_stream_payload,
                                                payload_len))
        {
            s_tcp_send_fail_count++; // 记录发送失败
            return;
        }

        // ⑨ 更新流序列号
        s_adc_stream_seq += sample_count;
    }
}
