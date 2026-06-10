#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "SEGGER_RTT.h"

#include "app_log.h"
#include "adc_acq_service.h"
#include "adc_tcp_server.h"
#include "eeprom_storage.h"
#include "device_config.h"
#include "udp_discovery.h"
#include "dac_output_service.h"

/* ================== Function Prototypes ================== */

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_IWDG1_Init(void);

/* ================== Module State ================== */

// 看门狗句柄
static IWDG_HandleTypeDef hiwdg1;

// 复位原因标志位定义
#define APP_RESET_CAUSE_PIN (1UL << 0)   // 引脚复位
#define APP_RESET_CAUSE_POR (1UL << 1)   // 上电复位
#define APP_RESET_CAUSE_BOR (1UL << 2)   // 低功耗复位
#define APP_RESET_CAUSE_SOFT (1UL << 3)  // 软件复位
#define APP_RESET_CAUSE_IWDG (1UL << 4)  // 看门狗复位
#define APP_RESET_CAUSE_WWDG (1UL << 5)  // 窗口看门狗复位
#define APP_RESET_CAUSE_LPWR1 (1UL << 6) // 低功耗模式复位1
#define APP_RESET_CAUSE_LPWR2 (1UL << 7) // 低功耗模式复位2

// 主循环监控计数器
static uint32_t s_loop_count = 0U;

// DAC 级联模式状态追踪
static uint8_t s_prev_streaming_state = 0U;

// 看门狗喂狗失败计数（用于调试）
static uint32_t s_iwdg_refresh_fail_count = 0U;

/* ================== Function Prototypes (Internal) ================== */

static void App_LogResetCause(void);
static HAL_StatusTypeDef App_InitializeAllPeripherals(void);
static void App_PrintStartupBanner(void);
static void App_MonitorLoopHealth(void);

/* ================== PUBLIC MAIN FUNCTION ================== */

/**
 * @brief Main entry point
 * @retval int (never returns)
 */
int main(void)
{
  // ====== Stage 1: Boot Configuration ======
  MPU_Config();       // 内存保护配置
  SCB_EnableICache(); // 启用指令缓存
  SCB_EnableDCache(); // 启用数据缓存

  // ====== Stage 2: HAL & System Clock ======
  HAL_Init();                 // 初始化 HAL 库
  SystemClock_Config();       // 系统时钟配置（400MHz）
  PeriphCommonClock_Config(); // 外设公共时钟配置

  // ====== Stage 3: Logging & Diagnostics ======
  app_log_init(); // 初始化日志系统
  app_log_key_event(APP_LOG_EVENT_LOGGER_STARTED,
                    "logger started");

  App_LogResetCause(); // 记录复位原因（用于故障排查）

  device_config_init_defaults(); // 初始化设备配置（使用默认值）

  // ====== Stage 4: Hardware Peripherals ======
  // GPIO 和 DMA 必须首先初始化，因为其他外设可能依赖它们
  MX_GPIO_Init(); // GPIO 初始化
  MX_DMA_Init();  // DMA 初始化

  // ADC 初始化（三个通道）
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();

  // 其他外设初始化
  MX_SPI1_Init(); // SPI 总线
  MX_TIM6_Init(); // 定时器 6
  MX_I2C2_Init(); // I2C 总线

  // ====== Stage 5: Storage & Configuration ======
  eeprom_storage_init(); // EEPROM 初始化

  // 从 EEPROM 加载配置，如果失败则使用默认值
  if (HAL_OK == device_config_load_all())
  {
    SEGGER_RTT_WriteString(0, "✓ Device config loaded from EEPROM\r\n");
    app_log_key_event(APP_LOG_EVENT_CONFIG_LOADED, "config loaded");
  }
  else
  {
    SEGGER_RTT_WriteString(0, "⚠ Device config not found, using defaults\r\n");
    app_log_key_event(APP_LOG_EVENT_CONFIG_DEFAULT, "config default");
  }

  // ====== Stage 6: Networking ======
  MX_USART2_UART_Init(); // UART 2（调试串口）
  MX_LWIP_Init();        // LWIP TCP/IP 栈初始化

  app_log_key_event(APP_LOG_EVENT_PERIPHERALS_READY,
                    "peripherals init done");

  // ====== Stage 7: Application Services ======
  // TCP 服务器：处理网络命令和 ADC 数据流
  adc_tcp_server_init();

  // UDP 发现服务：当无 TCP 连接时，广播设备信息
  udp_discovery_init();

  // ADC 采集服务
  adc_acq_service_init();
  adc_acq_service_start();

  // DAC 输出服务
  dac_output_service_init();

  // ====== Stage 8: Watchdog Timer ======
  // 看门狗必须最后初始化，防止初始化过程被看门狗中断
  MX_IWDG1_Init();
  SEGGER_RTT_WriteString(0, "✓ Watchdog timer initialized (~4s timeout)\r\n");

  // ====== Stage 9: System Ready ======
  App_PrintStartupBanner(); // 打印启动信息
  app_log_key_event(APP_LOG_EVENT_SYSTEM_READY, "system startup complete");

  s_prev_streaming_state = 0U; // 初始化状态追踪

  // ===================================
  // ========== MAIN LOOP =============
  // ===================================

  while (1)
  {
    // ① 网络处理（必须频繁调用，LWIP 对实时性敏感）
    MX_LWIP_Process();

    // ② TCP 服务器处理
    // 功能：接收 PC 命令、发送 ADC 数据流、更新网络配置
    adc_tcp_server_process();

    // ③ DAC 输出处理（基础）
    dac_output_service_process();

    // ④ DAC 级联模式
    // 当 TCP 流不活跃时，DAC 直接消耗 ADC 采样
    // 当 TCP 流活跃时，TCP 已负责发送 ADC 数据，DAC 不再消耗
    uint8_t current_streaming = adc_tcp_server_is_streaming();

    // 检测状态变化
    if (s_prev_streaming_state != current_streaming)
    {
      if (0U == current_streaming)
      {
        SEGGER_RTT_WriteString(0, "[STATE] TCP streaming stopped, switching to cascade mode\r\n");
        app_log_record(APP_LOG_EVENT_TCP_STREAM_STOPPED, 0U, 0U, 0U);
      }
      else
      {
        SEGGER_RTT_WriteString(0, "[STATE] TCP streaming started, disabling cascade mode\r\n");
        app_log_record(APP_LOG_EVENT_TCP_STREAM_STARTED, 0U, 0U, 0U);
      }
      s_prev_streaming_state = current_streaming;
    }

    // 只在非流模式下调用级联处理
    if (0U == current_streaming)
    {
      dac_output_service_process_adc_cascade();
    }

    // ⑤ UDP 发现（仅在无 TCP 连接时广播）
    udp_discovery_process();

    // ⑥ 看门狗喂狗
    // 看门狗超时约 4 秒，必须频繁刷新
    // 如果喂狗被禁用（通过 TCP 命令），记录警告但不停止主循环
    if (0U != adc_tcp_server_is_watchdog_feed_enabled())
    {
      if (HAL_IWDG_Refresh(&hiwdg1) != HAL_OK)
      {
        s_iwdg_refresh_fail_count++;
        SEGGER_RTT_printf(0, "✗ Watchdog refresh failed (count: %u)\r\n",
                          s_iwdg_refresh_fail_count);
        app_log_record(APP_LOG_EVENT_IWDG_REFRESH_FAILED,
                       (uint16_t)s_iwdg_refresh_fail_count,
                       0U, 0U);

        // 如果连续失败，触发错误处理
        if (s_iwdg_refresh_fail_count > 3U)
        {
          Error_Handler();
        }
      }
      else
      {
        // 清空失败计数（看门狗恢复正常）
        if (s_iwdg_refresh_fail_count > 0U)
        {
          s_iwdg_refresh_fail_count = 0U;
          SEGGER_RTT_WriteString(0, "✓ Watchdog refresh recovered\r\n");
        }
      }
    }
    else
    {
      // 看门狗喂狗被禁用（调试用）
      if ((s_loop_count % 5000U) == 0U) // 每 5000 次循环输出一次
      {
        SEGGER_RTT_WriteString(0, "⚠ Warning: Watchdog feed is disabled\r\n");
      }
    }

    // ⑦ 循环健康监控（定期输出系统状态）
    App_MonitorLoopHealth();

    s_loop_count++;
  }

  return 0; // 不会执行到这里
}

/* ================== PRIVATE FUNCTIONS ================== */

/**
 * @brief 记录系统复位原因
 * @details 检查 RCC 寄存器中的复位标志，记录到日志系统
 * @retval None
 */
static void App_LogResetCause(void)
{
  uint32_t cause = 0U;

  // 逐个检查各种复位原因
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U)
  {
    cause |= APP_RESET_CAUSE_PIN;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U)
  {
    cause |= APP_RESET_CAUSE_POR;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U)
  {
    cause |= APP_RESET_CAUSE_BOR;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U)
  {
    cause |= APP_RESET_CAUSE_SOFT;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != 0U)
  {
    cause |= APP_RESET_CAUSE_IWDG;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST) != 0U)
  {
    cause |= APP_RESET_CAUSE_WWDG;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST) != 0U)
  {
    cause |= APP_RESET_CAUSE_LPWR1;
  }

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR2RST) != 0U)
  {
    cause |= APP_RESET_CAUSE_LPWR2;
  }

  // 记录到日志系统
  app_log_record(APP_LOG_EVENT_RESET_CAUSE, 0U, cause, 0U);

  // 清除所有复位标志，为下次复位做准备
  __HAL_RCC_CLEAR_RESET_FLAGS();

  // 输出复位原因（便于调试）
  SEGGER_RTT_printf(0, "Reset cause: 0x%02X\r\n", (unsigned int)cause);
}

/**
 * @brief 初始化所有外设
 * @details 集中管理所有外设初始化，便于后续添加新模块
 * @retval HAL_StatusTypeDef
 */
static HAL_StatusTypeDef App_InitializeAllPeripherals(void)
{
  // 此函数在这个版本中未使用（保留用于未来扩展）
  // 未来可以实现模块表机制，动态初始化各模块
  return HAL_OK;
}

/**
 * @brief 打印启动横幅
 * @details 输出系统配置和就绪状态
 * @retval None
 */
static void App_PrintStartupBanner(void)
{
  SEGGER_RTT_WriteString(0, "\r\n");
  SEGGER_RTT_WriteString(0, "╔════════════════════════════════════════════╗\r\n");
  SEGGER_RTT_WriteString(0, "║      ADC Data Acquisition System Ready     ║\r\n");
  SEGGER_RTT_WriteString(0, "╚════════════════════════════════════════════╝\r\n");
  SEGGER_RTT_WriteString(0, "\r\n");

  SEGGER_RTT_WriteString(0, "System Information:\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ CPU Clock:     400 MHz\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ TCP Server:    Port 8080\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ ADC Channels:  3 (ADC1/2/3)\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ DAC Channels:  4 (Output)\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ Watchdog:      ~4 seconds timeout\r\n");
  SEGGER_RTT_WriteString(0, "  └─ Storage:       EEPROM + SDRAM\r\n");
  SEGGER_RTT_WriteString(0, "\r\n");

  SEGGER_RTT_WriteString(0, "Network Status:\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ LWIP:          Initialized\r\n");
  SEGGER_RTT_WriteString(0, "  ├─ UDP Discovery: Listening\r\n");
  SEGGER_RTT_WriteString(0, "  └─ TCP Listener:  Waiting for clients...\r\n");
  SEGGER_RTT_WriteString(0, "\r\n");

  SEGGER_RTT_WriteString(0, "Ready for operation. Connect via TCP:8080\r\n");
  SEGGER_RTT_WriteString(0, "═══════════════════════════════════════════════\r\n\r\n");
}

/**
 * @brief 主循环健康监控
 * @details 定期输出系统运行状态（调试用）
 * @retval None
 */
static void App_MonitorLoopHealth(void)
{
#define MONITOR_INTERVAL 50000U // 每 50000 次循环输出一次

  // 避免频繁输出，每 MONITOR_INTERVAL 次循环输出一次
  if ((s_loop_count % MONITOR_INTERVAL) != 0U)
  {
    return;
  }

  // 采集当前系统状态
  uint8_t has_client = adc_tcp_server_has_client();
  uint8_t is_streaming = adc_tcp_server_is_streaming();
  uint8_t iwdg_enabled = adc_tcp_server_is_watchdog_feed_enabled();

  // 输出系统状态
  SEGGER_RTT_printf(0, "[HEALTH] Loop: %u | Client: %u | Stream: %u | IWDG: %u\r\n",
                    (unsigned int)s_loop_count,
                    has_client,
                    is_streaming,
                    iwdg_enabled);

  // 输出看门狗失败计数（如果有）
  if (s_iwdg_refresh_fail_count > 0U)
  {
    SEGGER_RTT_printf(0, "         IWDG Fail Count: %u\r\n",
                      (unsigned int)s_iwdg_refresh_fail_count);
  }
}

/**
 * @brief 看门狗初始化
 * @details 配置独立看门狗（IWDG1）
 *          超时时间 = (Reload × Prescaler) / 时钟频率
 *                  = (2000 × 64) / 32000 ≈ 4 秒
 * @retval None
 */
static void MX_IWDG1_Init(void)
{
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_64; // 分频因子：64
  hiwdg1.Init.Reload = 2000U;                // 重装值：2000
  hiwdg1.Init.Window = IWDG_WINDOW_DISABLE;  // 禁用窗口模式

  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    SEGGER_RTT_WriteString(0, "✗ Watchdog init failed\r\n");
    Error_Handler();
  }
}

/* ================== SYSTEM CONFIGURATION FUNCTIONS ================== */

/**
 * @brief System Clock Configuration
 * @details 配置 STM32H7 系统时钟为 400MHz
 *          外部晶振 25MHz + PLL 倍频
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ================== MPU CONFIGURATION ================== */

/**
 * @brief MPU Configuration
 * @details 配置内存保护单元
 *          区域 0: 全局禁止访问（4GB）
 *          区域 1: 以太网 DMA 描述符（1KB，允许访问）
 * @retval None
 */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* Region 0: 默认 4GB 区域（禁止访问） */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region 1: ETH DMA 描述符区（允许访问） */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* ================== ERROR HANDLING ================== */

/**
 * @brief  This function is executed in case of error occurrence.
 * @details 当发生不可恢复错误时调用此函数
 *          关闭所有中断，进入无限循环等待复位
 * @retval None
 */
void Error_Handler(void)
{
  SEGGER_RTT_WriteString(0, "\r\n");
  SEGGER_RTT_WriteString(0, "╔════════════════════════════════════════════╗\r\n");
  SEGGER_RTT_WriteString(0, "║          ERROR_HANDLER TRIGGERED!          ║\r\n");
  SEGGER_RTT_WriteString(0, "╚════════════════════════════════════════════╝\r\n");
  SEGGER_RTT_WriteString(0, "System will be reset by watchdog...\r\n");

  app_log_record(APP_LOG_EVENT_ERROR_HANDLER, 0U, 0U, 0U);

  __disable_irq();
  while (1)
  {
    // 等待看门狗复位系统
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  SEGGER_RTT_printf(0, "Assert failed: %s:%u\r\n", (const char *)file, (unsigned int)line);
  Error_Handler();
}
#endif /* USE_FULL_ASSERT */
