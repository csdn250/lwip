#include "adc_param_store.h"
#include "device_config.h"
#include "adc_proto.h"
#include "SEGGER_RTT.h"

#include <string.h>

/* ==================== Macro Definitions ==================== */

/**
 * @brief 获取数组元素个数
 * @param array: 数组
 * @retval 数组元素个数
 */
#define ADC_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/**
 * @brief ADC 校准参数字节数
 * @details 12 个通道 × 2（k + b） × 4 字节/参数 = 96 字节
 */
#define ADC_CAL_PARAM_BYTES (DEVICE_CONFIG_ADC_CHANNEL_COUNT * 2U * sizeof(float))

/**
 * @brief DAC 参数字节数（每个通道）
 * @details mode(1) + voltage(4) + adc_ch(1) + k(4) + b(4) = 14 字节
 */
#define DAC_PARAM_BYTES 14U

/* ==================== Static Parameter Storage ==================== */

/**
 * @brief ADC 校准数据缓冲区
 * @details 存储 12 个通道的校准参数（k, b）
 */
static uint8_t s_param_cal_data[128];

/**
 * @brief 控制参数缓冲区
 * @details 用于存储系统控制参数（如看门狗启用/禁用等）
 */
static uint8_t s_param_control[8];

/**
 * @brief 配置参数缓冲区
 * @details 系统配置参数（如工作模式等）
 * @note 默认值：{1, 0, 0x05, 0xB4}
 */
static uint8_t s_param_config[4] = {1U, 0U, 0x05U, 0xB4U};

/**
 * @brief IP 地址参数缓冲区
 * @details 格式：[设备IP (4字节)] [网关IP (4字节)]
 * @note 默认值：192.168.1.21 (设备) / 192.168.1.20 (网关)
 */
static uint8_t s_param_ip_addr[8] = {192U, 168U, 1U, 21U, 192U, 168U, 1U, 20U};

/**
 * @brief MAC 地址参数缓冲区
 * @details 以太网 MAC 地址（6 字节）
 * @note 默认值：02:00:00:00:00:21
 */
static uint8_t s_param_mac_addr[6] = {0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x21U};

/**
 * @brief TCP 端口号参数缓冲区
 * @details 大端序存储的 16 位端口号
 * @note 默认值：0x1F90 = 8080 (十进制)
 */
static uint8_t s_param_port[4] = {0x1FU, 0x90U, 0x00U, 0x00U};

/**
 * @brief 子网掩码参数缓冲区
 * @details 格式：[子网掩码IPv4 (4字节)] [子网掩码IPv6 (4字节)]
 * @note 默认值：255.255.255.0
 */
static uint8_t s_param_netmask[8] = {255U, 255U, 255U, 0U, 255U, 255U, 255U, 0U};

/**
 * @brief 网关地址参数缓冲区
 * @details 格式：[网关IPv4 (4字节)] [网关IPv6 (4字节)]
 * @note 默认值：192.168.1.1
 */
static uint8_t s_param_gateway[8] = {192U, 168U, 1U, 1U, 192U, 168U, 1U, 1U};

/**
 * @brief DAC 通道 1 参数缓冲区
 * @details 存储 DAC CH1 的工作模式、输出电压、校准参数等
 */
static uint8_t s_param_da_ch1[DAC_PARAM_BYTES];

/**
 * @brief DAC 通道 2 参数缓冲区
 */
static uint8_t s_param_da_ch2[DAC_PARAM_BYTES];

/**
 * @brief DAC 通道 3 参数缓冲区
 */
static uint8_t s_param_da_ch3[DAC_PARAM_BYTES];

/**
 * @brief DAC 通道 4 参数缓冲区
 */
static uint8_t s_param_da_ch4[DAC_PARAM_BYTES];

/**
 * @brief 设备名参数缓冲区
 * @details 存储设备名（与 device_config 中的 name 字段同步）。
 *          UDP 广播也会发送此名，上位机可通过 0x000F block 读写。
 */
static uint8_t s_param_name[DEVICE_CONFIG_NAME_MAX_LEN];

/* ==================== Parameter Table ==================== */

/**
 * @brief 参数块表
 * @details 定义所有参数块的信息：
 *          - block_id:   参数块唯一标识符
 *          - data:       参数数据存放地址
 *          - len:        当前有效长度
 *          - max_len:    最大允许长度
 *
 * @note 表的顺序决定了参数块的查询顺序
 */
static adc_param_block_t s_param_table[] =
    {
        // 校准数据块（12 个通道的 k 和 b）
        {ADC_PARAM_BLOCK_CAL_DATA, s_param_cal_data,
         ADC_CAL_PARAM_BYTES, sizeof(s_param_cal_data)},

        // 控制参数块
        {ADC_PARAM_BLOCK_CONTROL, s_param_control,
         sizeof(s_param_control), sizeof(s_param_control)},

        // 配置参数块
        {ADC_PARAM_BLOCK_CONFIG, s_param_config,
         sizeof(s_param_config), sizeof(s_param_config)},

        // 网络配置块
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

        // DAC 参数块
        {ADC_PARAM_BLOCK_DA_CH1, s_param_da_ch1,
         sizeof(s_param_da_ch1), sizeof(s_param_da_ch1)},

        {ADC_PARAM_BLOCK_DA_CH2, s_param_da_ch2,
         sizeof(s_param_da_ch2), sizeof(s_param_da_ch2)},

        {ADC_PARAM_BLOCK_DA_CH3, s_param_da_ch3,
         sizeof(s_param_da_ch3), sizeof(s_param_da_ch3)},

        {ADC_PARAM_BLOCK_DA_CH4, s_param_da_ch4,
         sizeof(s_param_da_ch4), sizeof(s_param_da_ch4)},

        // 设备名块（与 device_config.name 同步，可读可写）
        {ADC_PARAM_BLOCK_DEVICE_NAME, s_param_name,
         sizeof(s_param_name), sizeof(s_param_name)},
};

/* ==================== Network Configuration Dirty Flag ==================== */

/**
 * @brief 网络配置脏标记
 * @details 1 = 网络配置已修改但未应用，0 = 网络配置已同步
 *
 * @note 用于跟踪网络参数的变化，main.c 中会定期检查此标记
 *       并在无 TCP 连接时自动应用新的网络配置
 */
static uint8_t s_network_config_dirty;

/* ==================== Public API Functions ==================== */

/**
 * @brief 初始化参数存储模块
 * @details 从 device_config 同步所有参数到本地缓冲区
 * @retval None
 */
void adc_param_store_init(void)
{
    // 将 device_config 中的参数同步到参数表
    adc_param_store_sync_from_config();
}
/**
 * @brief 在参数表中查找指定 ID 的参数块
 * @details 遍历参数表，查找匹配的 block_id
 * @param[in] block_id: 参数块 ID（见 adc_param_store.h 中的定义）
 * @retval adc_param_block_t* 指向参数块的指针，未找到返回 NULL
 */
adc_param_block_t *adc_param_store_find_block(uint16_t block_id)
{
    uint16_t i;

    // 遍历参数表
    for (i = 0U; i < ADC_ARRAY_SIZE(s_param_table); i++)
    {
        if (s_param_table[i].block_id == block_id)
        {
            return &s_param_table[i]; // ✓ 找到匹配的参数块
        }
    }

    return NULL; // ✗ 未找到
}

/* ==================== Parameter Synchronization ==================== */

/**
 * @brief 同步单个 DAC 参数块
 * @details 将 device_config 中的 DAC 通道配置转换为参数块格式
 *
 * @param[out] buf:    输出缓冲区
 * @param[in]  config: 源配置结构（来自 device_config）
 * @retval None
 *
 * @note 格式：[mode(1)] [voltage(4)] [adc_ch(1)] [k(4)] [b(4)]
 */
static void adc_param_store_sync_dac_param_block(uint8_t *buf,
                                                 const device_dac_channel_config_t
                                                     *dac_ch_config)
{
    uint16_t index;

    if ((NULL == buf) || (NULL == dac_ch_config))
    {
        return;
    }

    index = 0U;

    // ① 工作模式（1 字节）
    buf[index++] = dac_ch_config->mode;

    // ② 手动输出电压（4 字节，浮点数）
    adc_proto_put_float_be(buf,
                           &index,
                           dac_ch_config->manual_voltage);

    // ③ ADC 级联通道号（1 字节）
    buf[index++] = dac_ch_config->adc_channel;

    // ④ 校准参数 k（4 字节，大端序）
    adc_proto_put_float_be(buf,
                           &index,
                           dac_ch_config->k);

    // ⑤ 校准参数 b（4 字节，大端序）
    adc_proto_put_float_be(buf,
                           &index,
                           dac_ch_config->b);
}

/**
 * @brief 从 device_config 同步所有参数到参数表
 * @details 将 device_config 中的所有配置复制到对应的参数块缓冲区
 *
 * @retval None
 *
 * @note 在以下情况调用：
 *       - adc_param_store_init()：初始化时
 *       - 写入校准参数或 DAC 参数后
 */
void adc_param_store_sync_from_config(void)
{
    const device_network_config_t *network_config;
    const device_adc_cal_config_t *adc_cal_config;
    const device_dac_config_t *dac_config;

    uint16_t index;
    uint8_t ch;

    // ① 获取各模块的配置
    network_config = device_config_get_network();
    adc_cal_config = device_config_get_adc_calibration();
    dac_config = device_config_get_dac_config();

    // ② 同步网络配置
    memcpy(s_param_ip_addr, network_config->ip, sizeof(network_config->ip));
    memcpy(s_param_mac_addr, network_config->mac, sizeof(network_config->mac));

    s_param_port[0] = (uint8_t)(network_config->tcp_port >> 8);    // 高字节
    s_param_port[1] = (uint8_t)(network_config->tcp_port & 0xFFU); // 低字节

    memcpy(s_param_netmask, network_config->netmask, sizeof(network_config->netmask));
    memcpy(s_param_gateway, network_config->gateway, sizeof(network_config->gateway));

    // 同步设备名（读路径：READ_PARAM 0x000F 返回当前名，全长 32 字节）
    memcpy(s_param_name, network_config->name, DEVICE_CONFIG_NAME_MAX_LEN);

    // ③ 同步 ADC 校准参数
    index = 0U;
    for (ch = 0U; ch < DEVICE_CONFIG_ADC_CHANNEL_COUNT; ch++)
    {
        // 对每个通道存储 k（4 字节）和 b（4 字节）
        adc_proto_put_float_be(s_param_cal_data,
                               &index,
                               adc_cal_config->ch[ch].k);
        adc_proto_put_float_be(s_param_cal_data,
                               &index,
                               adc_cal_config->ch[ch].b);
    }

    // ④ 同步 DAC 参数
    adc_param_store_sync_dac_param_block(s_param_da_ch1,
                                         &dac_config->ch[0]);
    adc_param_store_sync_dac_param_block(s_param_da_ch2,
                                         &dac_config->ch[1]);
    adc_param_store_sync_dac_param_block(s_param_da_ch3,
                                         &dac_config->ch[2]);
    adc_param_store_sync_dac_param_block(s_param_da_ch4,
                                         &dac_config->ch[3]);
}

/* ==================== Parameter Write Validation ==================== */

/**
 * @brief 检查写入参数的长度是否合法
 * @details 确保写入的数据大小与参数块定义一致
 *
 * @param[in] block_id: 参数块 ID
 * @param[in] len:      数据长度（字节）
 * @retval uint8_t
 *         ADC_PROTO_WRITE_STATUS_OK      = 长度正确
 *         ADC_PROTO_WRITE_STATUS_BAD_LEN = 长度不匹配
 */
uint8_t adc_param_store_check_write_len(uint16_t block_id,
                                        uint16_t len)
{
    // ① 校准参数必须是完整的 96 字节
    if (ADC_PARAM_BLOCK_CAL_DATA == block_id)
    {
        return (ADC_CAL_PARAM_BYTES == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // ② DAC 参数（4 个通道）必须是 14 字节
    if ((ADC_PARAM_BLOCK_DA_CH1 <= block_id) &&
        (ADC_PARAM_BLOCK_DA_CH4 >= block_id))
    {
        return (DAC_PARAM_BYTES == len)
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // ③ 网络配置参数大小检查
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

    // ④ 设备名：允许变长，1..DEVICE_CONFIG_NAME_MAX_LEN 字节
    if (ADC_PARAM_BLOCK_DEVICE_NAME == block_id)
    {
        return ((len >= 1U) && (len <= DEVICE_CONFIG_NAME_MAX_LEN))
                   ? ADC_PROTO_WRITE_STATUS_OK
                   : ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    return ADC_PROTO_WRITE_STATUS_OK;
}

/* ==================== Network Parameter Application ==================== */

/**
 * @brief 应用网络参数
 * @details 将新的网络参数写入 device_config 并保存到 EEPROM
 *
 * @param[in] block_id: 参数块 ID（IP、MAC、端口、掩码、网关之一）
 * @param[in] data:     新的参数数据
 * @param[in] len:      参数数据长度
 * @retval uint8_t
 *         ADC_PROTO_WRITE_STATUS_OK            = 应用成功并已保存
 *         ADC_PROTO_WRITE_STATUS_SAVE_PENDING  = 应用成功但保存待处理
 *         ADC_PROTO_WRITE_STATUS_BAD_LEN       = 参数长度不合法
 *
 * @note 此函数会：
 *       1. 验证输入数据
 *       2. 更新 device_config
 *       3. 设置网络脏标记
 *       4. 尝试保存到 EEPROM
 */
uint8_t adc_param_store_apply_network_param(uint16_t block_id,
                                            const uint8_t *data,
                                            uint16_t len)
{
    device_network_config_t next_network_config;

    if (NULL == data)
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // ① 复制当前网络配置
    memcpy(&next_network_config,
           device_config_get_network(),
           sizeof(next_network_config));

    // ② 根据参数块 ID 更新配置
    if (ADC_PARAM_BLOCK_IP_ADDR == block_id)
    {
        if (len < sizeof(next_network_config.ip))
        {
            SEGGER_RTT_WriteString(0, "net param ip bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }
        memcpy(next_network_config.ip, data, sizeof(next_network_config.ip));
    }
    else if (ADC_PARAM_BLOCK_MAC_ADDR == block_id)
    {
        if (len < sizeof(next_network_config.mac))
        {
            SEGGER_RTT_WriteString(0, "net param mac bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }
        memcpy(next_network_config.mac, data, sizeof(next_network_config.mac));
    }
    else if (ADC_PARAM_BLOCK_PORT == block_id)
    {
        if (len < 2U)
        {
            SEGGER_RTT_WriteString(0, "net param port bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }
        // 从大端序字节构建端口号
        next_network_config.tcp_port = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    }
    else if (ADC_PARAM_BLOCK_NETMASK == block_id)
    {
        if (len < sizeof(next_network_config.netmask))
        {
            SEGGER_RTT_WriteString(0, "net param mask bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }
        memcpy(next_network_config.netmask, data, sizeof(next_network_config.netmask));
    }
    else if (ADC_PARAM_BLOCK_GATEWAY == block_id)
    {
        if (len < sizeof(next_network_config.gateway))
        {
            SEGGER_RTT_WriteString(0, "net param gateway bad len\r\n");
            return ADC_PROTO_WRITE_STATUS_BAD_LEN;
        }
        memcpy(next_network_config.gateway, data, sizeof(next_network_config.gateway));
    }
    else
    {
        return ADC_PROTO_WRITE_STATUS_OK;
    }

    // ③ 更新配置并设置脏标记
    device_config_set_network(&next_network_config);
    s_network_config_dirty = 1U; // ← 标记网络参数已修改（驱动 netif 重配）

    // ④ 请求延迟保存到 EEPROM（实际写入在主循环空闲点完成，不在此阻塞）
    device_config_request_save();

    SEGGER_RTT_WriteString(0, "net param save pending\r\n");
    SEGGER_RTT_WriteString(0, "net param applied\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

/**
 * @brief 获取网络配置脏标记
 * @details 检查网络参数是否已修改但未应用
 * @retval uint8_t 1 = 有未应用的修改，0 = 已同步
 *
 * @note main.c 中的 adc_tcp_server_process() 会定期检查此标记
 *       并在无客户端连接时调用 MX_LWIP_ApplyNetworkConfig()
 */
uint8_t adc_param_store_is_network_dirty(void)
{
    return s_network_config_dirty;
}

/**
 * @brief 清除网络配置脏标记
 * @details 在网络参数成功应用后调用
 * @retval None
 */
void adc_param_store_clear_network_dirty(void)
{
    s_network_config_dirty = 0U;
}

/* ==================== Device Name Application ==================== */

/**
 * @brief 应用设备名参数
 * @details 将新的设备名写入 device_config 并请求延迟保存到 EEPROM
 *
 * @param[in] data: 设备名原始字节（不要求以 '\0' 结尾）
 * @param[in] len:  字节数（1..DEVICE_CONFIG_NAME_MAX_LEN）
 * @retval uint8_t
 *         ADC_PROTO_WRITE_STATUS_SAVE_PENDING = 应用成功，保存待处理
 *         ADC_PROTO_WRITE_STATUS_BAD_LEN      = 参数非法
 *
 * @note 名字最长保留 DEVICE_CONFIG_NAME_MAX_LEN-1 个字符，强制末尾 '\0'，
 *       避免上位机未带终止符导致 UDP 广播 / 后续读取越界。
 */
uint8_t adc_param_store_apply_name_param(const uint8_t *data,
                                         uint16_t len)
{
    device_network_config_t next_network_config;
    uint16_t copy_len;

    // ① 验证输入
    if ((NULL == data) || (0U == len) || (len > DEVICE_CONFIG_NAME_MAX_LEN))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // ② 复制当前网络配置
    memcpy(&next_network_config,
           device_config_get_network(),
           sizeof(next_network_config));

    // ③ 截断到可用空间并强制 NUL 结尾
    copy_len = len;
    if (copy_len > (DEVICE_CONFIG_NAME_MAX_LEN - 1U))
    {
        copy_len = (uint16_t)(DEVICE_CONFIG_NAME_MAX_LEN - 1U);
    }

    memset(next_network_config.name, 0, sizeof(next_network_config.name));
    memcpy(next_network_config.name, data, copy_len);
    // next_network_config.name 已 memset 清零，第 copy_len 字节即为 '\0'

    // ④ 更新 device_config 并同步参数表
    device_config_set_network(&next_network_config);
    adc_param_store_sync_from_config();

    // ⑤ 请求延迟保存（实际写入在主循环空闲点完成）
    device_config_request_save();

    SEGGER_RTT_WriteString(0, "name param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

/* ==================== ADC Calibration Parameter Application ==================== */

/**
 * @brief 应用 ADC 校准参数
 * @details 将新的校准参数写入 device_config 并保存到 EEPROM
 *
 * @param[in] data: 校准参数数据（96 字节，12 通道 × 2 参数 × 4 字节）
 * @param[in] len:  参数长度
 * @retval uint8_t
 *         ADC_PROTO_WRITE_STATUS_OK            = 应用成功并已保存
 *         ADC_PROTO_WRITE_STATUS_SAVE_PENDING  = 应用成功但保存待处理
 *         ADC_PROTO_WRITE_STATUS_BAD_LEN       = 参数长度不合法
 *
 * @note 校准参数格式：
 *       [CH0_k(4)] [CH0_b(4)] ... [CH11_k(4)] [CH11_b(4)]
 */
uint8_t adc_param_store_apply_cal_param(const uint8_t *data,
                                        uint16_t len)
{
    device_adc_cal_config_t next_adc_cal_config;
    uint16_t index;
    uint8_t ch;

    // ① 验证参数长度
    if ((NULL == data) || (ADC_CAL_PARAM_BYTES != len))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    index = 0U;

    // ② 解析校准参数
    for (ch = 0U; ch < DEVICE_CONFIG_ADC_CHANNEL_COUNT; ch++)
    {
        // 从大端序数据流解析 k 和 b
        next_adc_cal_config.ch[ch].k = adc_proto_get_float_be(data, &index);
        next_adc_cal_config.ch[ch].b = adc_proto_get_float_be(data, &index);
    }

    // ③ 更新 device_config
    device_config_set_adc_calibration(&next_adc_cal_config);

    // ④ 同步参数表
    adc_param_store_sync_from_config();

    // ⑤ 请求延迟保存到 EEPROM（实际写入在主循环空闲点完成）
    device_config_request_save();

    SEGGER_RTT_WriteString(0, "cal param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}

/* ==================== DAC Parameter Application ==================== */

/**
 * @brief 应用 DAC 参数
 * @details 将新的 DAC 通道参数写入 device_config 并保存到 EEPROM
 *
 * @param[in] block_id: 参数块 ID（ADC_PARAM_BLOCK_DA_CH1~DA_CH4 之一）
 * @param[in] data:     DAC 参数数据（14 字节）
 * @param[in] len:      参数长度
 * @retval uint8_t
 *         ADC_PROTO_WRITE_STATUS_OK            = 应用成功并已保存
 *         ADC_PROTO_WRITE_STATUS_SAVE_PENDING  = 应用成功但保存待处理
 *         ADC_PROTO_WRITE_STATUS_BAD_LEN       = 参数长度不合法
 *         ADC_PROTO_WRITE_STATUS_NOT_FOUND     = 参数块 ID 无效
 *
 * @note DAC 参数格式：
 *       [mode(1)] [voltage(4)] [adc_ch(1)] [k(4)] [b(4)]
 *
 *       其中 mode 可以是：
 *       - DEVICE_CONFIG_DAC_MODE_MANUAL      = 手动输出模式
 *       - DEVICE_CONFIG_DAC_MODE_ADC_CASCADE = ADC 级联模式
 */
uint8_t adc_param_store_apply_dac_param(uint16_t block_id,
                                        const uint8_t *data,
                                        uint16_t len)
{
    device_dac_config_t next_dac_config;
    uint16_t index;
    uint8_t dac_ch;

    // ① 验证输入参数
    if ((NULL == data) || (DAC_PARAM_BYTES != len))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // ② 验证参数块 ID 是否为 DAC 通道之一
    if ((block_id < ADC_PARAM_BLOCK_DA_CH1) ||
        (block_id > ADC_PARAM_BLOCK_DA_CH4))
    {
        return ADC_PROTO_WRITE_STATUS_NOT_FOUND;
    }

    // ③ 计算 DAC 通道索引（0~3）
    dac_ch = (uint8_t)(block_id - ADC_PARAM_BLOCK_DA_CH1);

    // ④ 复制当前 DAC 配置
    memcpy(&next_dac_config,
           device_config_get_dac_config(),
           sizeof(next_dac_config));

    index = 0U;

    // ⑤ 解析 DAC 参数
    next_dac_config.ch[dac_ch].mode = data[index++]; // 工作模式

    next_dac_config.ch[dac_ch].manual_voltage = adc_proto_get_float_be(data, &index); // 输出电压

    next_dac_config.ch[dac_ch].adc_channel = data[index++]; // ADC 级联通道

    next_dac_config.ch[dac_ch].k = adc_proto_get_float_be(data, &index); // 校准参数 k

    next_dac_config.ch[dac_ch].b = adc_proto_get_float_be(data, &index); // 校准参数 b

    // ⑥ 参数合法性检查
    // 检查工作模式
    if (next_dac_config.ch[dac_ch].mode > DEVICE_CONFIG_DAC_MODE_ADC_CASCADE)
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // 检查 ADC 级联模式中的 ADC 通道号
    if ((DEVICE_CONFIG_DAC_MODE_ADC_CASCADE == next_dac_config.ch[dac_ch].mode) &&
        (next_dac_config.ch[dac_ch].adc_channel >= DEVICE_CONFIG_ADC_CHANNEL_COUNT))
    {
        return ADC_PROTO_WRITE_STATUS_BAD_LEN;
    }

    // ⑦ 更新 device_config
    device_config_set_dac_config(&next_dac_config);

    // ⑧ 同步参数表
    adc_param_store_sync_from_config();

    SEGGER_RTT_WriteString(0, "dac param applied\r\n");

    // ⑨ 请求延迟保存到 EEPROM（实际写入在主循环空闲点完成）
    device_config_request_save();

    SEGGER_RTT_WriteString(0, "dac param save pending\r\n");
    return ADC_PROTO_WRITE_STATUS_SAVE_PENDING;
}
