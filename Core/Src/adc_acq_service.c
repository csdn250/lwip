#include "adc_acq_service.h"
#include "adc.h"
#include "tim.h"

#define ADC1_CHANNELS_PER_GROUP 4U
#define ADC2_CHANNELS_PER_GROUP 1U
#define ADC3_CHANNELS_PER_GROUP 7U

#define ADC_ACQ_HALF_COUNT ADC_ACQ_GROUPS_PER_HALF
#define ADC_ACQ_TOTAL_GROUP_COUNT (ADC_ACQ_GROUPS_PER_HALF * 2U)

#define ADC1_DMA_LENGTH (ADC_ACQ_TOTAL_GROUP_COUNT * ADC1_CHANNELS_PER_GROUP)
#define ADC2_DMA_LENGTH (ADC_ACQ_TOTAL_GROUP_COUNT * ADC2_CHANNELS_PER_GROUP)
#define ADC3_DMA_LENGTH (ADC_ACQ_TOTAL_GROUP_COUNT * ADC3_CHANNELS_PER_GROUP)

static uint16_t s_adc1_dma_buf[ADC1_DMA_LENGTH];
static uint16_t s_adc2_dma_buf[ADC2_DMA_LENGTH];
static uint16_t s_adc3_dma_buf[ADC3_DMA_LENGTH];

static volatile uint8_t s_adc1_half_ready;
static volatile uint8_t s_adc2_half_ready;
static volatile uint8_t s_adc3_half_ready;

static volatile uint8_t s_adc1_full_ready;
static volatile uint8_t s_adc2_full_ready;
static volatile uint8_t s_adc3_full_ready;

//当前应该从哪个“块”读数据？0 代表前半部分（Block 0），1 代表后半部分（Block 1）。
static uint8_t s_read_block;
//在当前这个块里，我已经读到第几个“组”了
static uint16_t s_read_group_index;
//当前是否有整个块的数据锁定了，可以安心读取？（1 表示锁定了，0 表示空闲或正在等待转换完毕）。
static uint8_t s_block_ready;
static uint32_t s_sample_seq;

void adc_acq_service_init(void)
{
    s_adc1_half_ready = 0U;
    s_adc2_half_ready = 0U;
    s_adc3_half_ready = 0U;

    s_adc1_full_ready = 0U;
    s_adc2_full_ready = 0U;
    s_adc3_full_ready = 0U;

    s_read_block = 0U;
    s_read_group_index = 0U;
    s_block_ready = 0U;
    s_sample_seq = 0U;
}

void adc_acq_service_start(void)
{
    HAL_TIM_Base_Stop(&htim6);

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma_buf, ADC1_DMA_LENGTH);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma_buf, ADC2_DMA_LENGTH);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)s_adc3_dma_buf, ADC3_DMA_LENGTH);

    HAL_TIM_Base_Start(&htim6);
}

uint8_t adc_acq_service_get_sample(adc_acq_sample_t *sample)
{
    uint32_t adc1_base;
    uint32_t adc2_base;
    uint32_t adc3_base;
    uint32_t group_base;

    if (NULL == sample)
    {
        return 0U;
    }

    if (0U == s_block_ready)
    {
        if ((0U != s_adc1_half_ready) &&
            (0U != s_adc2_half_ready) &&
            (0U != s_adc3_half_ready))
        {
            s_adc1_half_ready = 0U;
            s_adc2_half_ready = 0U;
            s_adc3_half_ready = 0U;

            s_read_block = 0U;
            s_read_group_index = 0U;
            s_block_ready = 1U;
        }
        else if ((0U != s_adc1_full_ready) &&
                 (0U != s_adc2_full_ready) &&
                 (0U != s_adc3_full_ready))
        {
            s_adc1_full_ready = 0U;
            s_adc2_full_ready = 0U;
            s_adc3_full_ready = 0U;

            s_read_block = 1U;
            s_read_group_index = 0U;
            s_block_ready = 1U;
        }
        else
        {
            return 0U;
        }
    }

    group_base = ((uint32_t)s_read_block * ADC_ACQ_HALF_COUNT) + s_read_group_index;

    adc1_base = group_base * ADC1_CHANNELS_PER_GROUP;
    adc2_base = group_base * ADC2_CHANNELS_PER_GROUP;
    adc3_base = group_base * ADC3_CHANNELS_PER_GROUP;

    sample->timestamp_us = s_sample_seq;

    sample->raw[0] = s_adc1_dma_buf[adc1_base + 0U];
    sample->raw[1] = s_adc1_dma_buf[adc1_base + 1U];
    sample->raw[2] = s_adc1_dma_buf[adc1_base + 2U];
    sample->raw[3] = s_adc1_dma_buf[adc1_base + 3U];

    sample->raw[4] = s_adc2_dma_buf[adc2_base + 0U];

    sample->raw[5] = s_adc3_dma_buf[adc3_base + 0U];
    sample->raw[6] = s_adc3_dma_buf[adc3_base + 1U];
    sample->raw[7] = s_adc3_dma_buf[adc3_base + 2U];
    sample->raw[8] = s_adc3_dma_buf[adc3_base + 3U];
    sample->raw[9] = s_adc3_dma_buf[adc3_base + 4U];
    sample->raw[10] = s_adc3_dma_buf[adc3_base + 5U];
    sample->raw[11] = s_adc3_dma_buf[adc3_base + 6U];

    ++s_sample_seq;
    ++s_read_group_index;

    if (s_read_group_index >= ADC_ACQ_HALF_COUNT)
    {
        s_block_ready = 0U;
        s_read_group_index = 0U;
    }

    return 1U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_adc1_half_ready = 1U;
    }
    else if (hadc->Instance == ADC2)
    {
        s_adc2_half_ready = 1U;
    }
    else if (hadc->Instance == ADC3)
    {
        s_adc3_half_ready = 1U;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_adc1_full_ready = 1U;
    }
    else if (hadc->Instance == ADC2)
    {
        s_adc2_full_ready = 1U;
    }
    else if (hadc->Instance == ADC3)
    {
        s_adc3_full_ready = 1U;
    }
}


