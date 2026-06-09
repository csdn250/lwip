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

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);

static void MX_IWDG1_Init(void);

static IWDG_HandleTypeDef hiwdg1;

#define APP_RESET_CAUSE_PIN (1UL << 0)
#define APP_RESET_CAUSE_POR (1UL << 1)
#define APP_RESET_CAUSE_BOR (1UL << 2)
#define APP_RESET_CAUSE_SOFT (1UL << 3)
#define APP_RESET_CAUSE_IWDG (1UL << 4)
#define APP_RESET_CAUSE_WWDG (1UL << 5)
#define APP_RESET_CAUSE_LPWR1 (1UL << 6)
#define APP_RESET_CAUSE_LPWR2 (1UL << 7)

static void App_LogResetCause(void);

int main(void)
{

  MPU_Config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();

  app_log_init();
  app_log_key_event(APP_LOG_EVENT_LOGGER_STARTED,
                    "logger started");
  App_LogResetCause();
  device_config_init_defaults();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  eeprom_storage_init();
  if (HAL_OK == device_config_load_all())
  {
    SEGGER_RTT_WriteString(0, "device config loaded\r\n");
  }
  else
  {
    SEGGER_RTT_WriteString(0, "device config default\r\n");
  }

  MX_USART2_UART_Init();

  MX_LWIP_Init();

  app_log_key_event(APP_LOG_EVENT_PERIPHERALS_READY,
                    "peripherals init done");

  adc_tcp_server_init();
  udp_discovery_init();

  adc_acq_service_init();
  adc_acq_service_start();
  dac_output_service_init();

  MX_IWDG1_Init();

  while (1)
  {
    /* lwIP raw API must be polled frequently in the main loop. */
    MX_LWIP_Process();

    /* TCP handles command parsing and ADC data streaming. */
    adc_tcp_server_process();

    dac_output_service_process();

    /*
     * DAC ADC-cascade mode is enabled by DA parameter blocks 0x0009~0x000C.
     * When TCP ADC streaming is off, DAC service consumes ADC samples itself.
     * When TCP ADC streaming is on, TCP pump feeds samples to DAC to avoid double-consuming samples.
     */
    if (0U == adc_tcp_server_is_streaming())
    {
      dac_output_service_process_adc_cascade();
    }

    /* UDP broadcasts device info only when no TCP client is connected. */
    udp_discovery_process();

    if ((0U != adc_tcp_server_is_watchdog_feed_enabled()) &&
        (HAL_IWDG_Refresh(&hiwdg1) != HAL_OK))
    {
      Error_Handler();
    }
  }
}

static void App_LogResetCause(void)
{
  uint32_t cause = 0U;

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

  app_log_record(APP_LOG_EVENT_RESET_CAUSE,
                 0U,
                 cause,
                 0U);

  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static void MX_IWDG1_Init(void)
{
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg1.Init.Reload = 2000U;
  hiwdg1.Init.Window = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
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

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
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

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* 默认 4GB 区域 */
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

  /* ETH DMA 描述符区 */
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

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
