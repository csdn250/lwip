#ifndef ADC_TCP_SERVER_H
#define ADC_TCP_SERVER_H

#include <stdint.h>
#include "lwip/err.h"
#include "lwip/tcp.h"

/* ==================== Public API ==================== */

/**
 * @brief 初始化 TCP 服务器
 * @details 创建监听 socket，开始监听客户端连接
 * @retval None
 */
void adc_tcp_server_init(void);

/**
 * @brief TCP 服务器主处理函数
 * @details 在主循环中频繁调用，处理网络事件和数据发送
 * @retval None
 */
void adc_tcp_server_process(void);

/**
 * @brief 检查是否有客户端连接
 * @retval 1 = 有客户端连接，0 = 无客户端
 */
uint8_t adc_tcp_server_has_client(void);

/**
 * @brief 检查是否正在进行 ADC 数据流传输
 * @details 返回 1 表示 TCP 客户端正在接收 ADC 数据流
 * @retval 1 = 正在流传输，0 = 未流传输
 */
uint8_t adc_tcp_server_is_streaming(void);

/**
 * @brief 检查看门狗喂狗是否启用
 * @details 通过 TCP 命令可以禁用看门狗喂狗（调试用）
 * @retval 1 = 喂狗启用，0 = 喂狗禁用
 */
uint8_t adc_tcp_server_is_watchdog_feed_enabled(void);

/**
 * @brief 获取网络配置是否脏（需要应用）
 * @retval 1 = 需要应用网络配置，0 = 配置已同步
 */
uint8_t adc_tcp_server_is_network_config_dirty(void);

#endif /* ADC_TCP_SERVER_H */
