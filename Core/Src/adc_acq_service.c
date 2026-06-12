#include "adc_acq_service.h"
#include "adc_status_service.h"
#include "adc.h"
#include "tim.h"

/* ========================= Configuration ========================= */

/**
 * @brief ADC1 每个采样组中的通道数
 * @details ADC1 负责采集 4 个模拟通道
 */
#define ADC1_CHANNELS_PER_GROUP 4U

/**
 * @brief ADC2 每个采样组中的通道数
 * @details ADC2 负责采集 1 个模拟通道
 */
#define ADC2_CHANNELS_PER_GROUP 1U

/**
 * @brief ADC3 每个采样组中的通道数
 * @details ADC3 负责采集 7 个模拟通道
 */
#define ADC3_CHANNELS_PER_GROUP 7U

/** @brief DMA 双缓冲的前半部分标记 */
#define ADC_ACQ_BLOCK_HALF 0U

/** @brief DMA 双缓冲的后半部分标记 */
#define ADC_ACQ_BLOCK_FULL 1U

/**
 * @brief 每个半缓冲区中的采样组数
 * @details 等于 ADC_ACQ_GROUPS_PER_HALF（在头文件中定义）
 */
#define ADC_ACQ_HALF_COUNT ADC_ACQ_GROUPS_PER_HALF

/**
 * @brief 完整 DMA 缓冲区中的总采样组数
 * @details = 前半 + 后半 = 2 × ADC_ACQ_GROUPS_PER_HALF
 */
#define ADC_ACQ_TOTAL_GROUP_COUNT (ADC_ACQ_GROUPS_PER_HALF * 2U)

/**
 * @brief ADC1 DMA 缓冲区长度
 * @details = 总采样组数 × ADC1 通道数
 */
#define ADC1_DMA_LENGTH (ADC_ACQ_TOTAL_GROUP_COUNT * ADC1_CHANNELS_PER_GROUP)

/**
 * @brief ADC2 DMA 缓冲区长度
 * @details = 总采样组数 × ADC2 通道数
 */
#define ADC2_DMA_LENGTH (ADC_ACQ_TOTAL_GROUP_COUNT * ADC2_CHANNELS_PER_GROUP)

/**
 * @brief ADC3 DMA 缓冲区长度
 * @details = 总采样组数 × ADC3 通道数
 */
#define ADC3_DMA_LENGTH (ADC_ACQ_TOTAL_GROUP_COUNT * ADC3_CHANNELS_PER_GROUP)

/* ========================= DMA Buffers ========================= */

/**
 * @brief ADC1 DMA 缓冲区（双缓冲）
 * @details 结构：
 *          [前半部分] [后半部分]
 *          ① DMA 硬件自动向此缓冲区写数据
 *          ② 当前半完成时，触发 HAL_ADC_ConvHalfCpltCallback
 *          ③ 当后半完成时，触发 HAL_ADC_ConvCpltCallback
 *          ④ 前后两个半缓冲区互不干扰，可安全读取已完成的半缓冲
 */
static uint16_t s_adc1_dma_buf[ADC1_DMA_LENGTH];

/**
 * @brief ADC2 DMA 缓冲区（双缓冲）
 * @details 与 ADC1 保持同步
 */
static uint16_t s_adc2_dma_buf[ADC2_DMA_LENGTH];

/**
 * @brief ADC3 DMA 缓冲区（双缓冲）
 * @details 与 ADC1 保持同步
 */
static uint16_t s_adc3_dma_buf[ADC3_DMA_LENGTH];

/* ========================= Ready Flags ========================= */

/**
 * @brief ADC1 前半缓冲区就绪标志
 * @details 由 HAL_ADC_ConvHalfCpltCallback 设置为 1
 *          由 adc_acq_service_try_lock_block 清除为 0
 */
static volatile uint8_t s_adc1_half_ready;

/**
 * @brief ADC2 前半缓冲区就绪标志
 */
static volatile uint8_t s_adc2_half_ready;

/**
 * @brief ADC3 前半缓冲区就绪标志
 */
static volatile uint8_t s_adc3_half_ready;

/**
 * @brief ADC1 后半缓冲区就绪标志
 * @details 由 HAL_ADC_ConvCpltCallback 设置为 1
 *          由 adc_acq_service_try_lock_block 清除为 0
 */
static volatile uint8_t s_adc1_full_ready;

/**
 * @brief ADC2 后半缓冲区就绪标志
 */
static volatile uint8_t s_adc2_full_ready;

/**
 * @brief ADC3 后半缓冲区就绪标志
 */
static volatile uint8_t s_adc3_full_ready;

/* ========================= Statistics ========================= */

/**
 * @brief ADC1 前半缓冲区完成计数
 * @details 每次 DMA 前半完成时加 1
 *          用于监控 ADC1 的采集速率
 */
static volatile uint32_t s_adc1_half_count;

/**
 * @brief ADC2 前半缓冲区完成计数
 */
static volatile uint32_t s_adc2_half_count;

/**
 * @brief ADC3 前半缓冲区完成计数
 */
static volatile uint32_t s_adc3_half_count;

/**
 * @brief ADC1 后半缓冲区完成计数
 * @details 每次 DMA 后半完成时加 1
 */
static volatile uint32_t s_adc1_full_count;

/**
 * @brief ADC2 后半缓冲区完成计数
 */
static volatile uint32_t s_adc2_full_count;

/**
 * @brief ADC3 后半缓冲区完成计数
 */
static volatile uint32_t s_adc3_full_count;

/**
 * @brief 缓冲区无可用块计数
 * @details 当需要读取采样但两个半缓冲区都未就绪时加 1
 *          用于检测是否有数据丢失或读取跟不上
 */
static uint32_t s_no_block_count;

/* ========================= Read State ========================= */

/**
 * @brief 优先的下一个读取块
 * @details 值为 ADC_ACQ_BLOCK_HALF 或 ADC_ACQ_BLOCK_FULL
 *          用于实现"乒乓"缓冲读取：
 *          - 读完前半后，下一个优先读后半
 *          - 读完后半后，下一个优先读前半
 *          - 如果优先块未就绪，则尝试备用块
 */
static uint8_t s_next_read_block;

/**
 * @brief 当前被锁定的读取块
 * @details 值为 ADC_ACQ_BLOCK_HALF 或 ADC_ACQ_BLOCK_FULL
 *          表示当前正在从哪个半缓冲区读取采样
 *          在 s_read_group_index 读完所有采样组后，此标志被清除
 */
static uint8_t s_read_block;

/**
 * @brief 当前读取块内的采样组索引
 * @details 范围：0 ~ (ADC_ACQ_HALF_COUNT - 1)
 *          每读取一个采样组就加 1
 *          当达到 ADC_ACQ_HALF_COUNT 时重置为 0，并清除 s_block_ready
 */
static uint16_t s_read_group_index;

/**
 * @brief 缓冲块就绪标志
 * @details 1 = 一个完整的半缓冲区已锁定，可以安全读取
 *          0 = 没有可用的半缓冲区，需要等待新的DMA完成
 *
 * @note 读取流程：
 *       ① adc_acq_service_get_sample() 检查 s_block_ready
 *       ② 如果为 0，调用 adc_acq_service_lock_next_block() 锁定新块
 *       ③ 如果成功锁定，s_block_ready 被设为 1
 *       ④ 每读完一个采样组，检查 s_read_group_index 是否达到限值
 *       ⑤ 如果达到限值，清除 s_block_ready 并重置索引
 */
static uint8_t s_block_ready;

/**
 * @brief 采样序列号（微秒或采样计数）
 * @details 每读取一个采样就加 1
 *          用于时间戳标记和数据完整性检查
 */
static uint32_t s_sample_seq;

/* ========================= Function Prototypes ========================= */

/**
 * @brief 尝试锁定指定的 DMA 块
 * @details 检查指定块是否就绪（所有 ADC 的对应半缓冲区都完成）
 * @param[in] block: ADC_ACQ_BLOCK_HALF 或 ADC_ACQ_BLOCK_FULL
 * @retval uint8_t
 *         1 = 成功锁定（块已就绪）
 *         0 = 无法锁定（块未就绪）
 */
static uint8_t adc_acq_service_try_lock_block(uint8_t block);

/**
 * @brief 锁定下一个可用的 DMA 块
 * @details 优先尝试 s_next_read_block，失败则尝试备用块
 * @retval uint8_t
 *         1 = 成功锁定
 *         0 = 两个块都未就绪
 */
static uint8_t adc_acq_service_lock_next_block(void);

/* ========================= Public API ========================= */

/**
 * @brief 初始化 ADC 采集服务
 * @details 重置所有状态变量、就绪标志、计数器
 * @retval None
 *
 * @note 调用时机：系统初始化阶段，在 adc_acq_service_start() 之前
 */
void adc_acq_service_init(void)
{
    // ① 初始化前半缓冲区就绪标志
    s_adc1_half_ready = 0U;
    s_adc2_half_ready = 0U;
    s_adc3_half_ready = 0U;

    // ② 初始化后半缓冲区就绪标志
    s_adc1_full_ready = 0U;
    s_adc2_full_ready = 0U;
    s_adc3_full_ready = 0U;

    // ③ 初始化前半完成计数
    s_adc1_half_count = 0U;
    s_adc2_half_count = 0U;
    s_adc3_half_count = 0U;

    // ④ 初始化后半完成计数
    s_adc1_full_count = 0U;
    s_adc2_full_count = 0U;
    s_adc3_full_count = 0U;

    // ⑤ 初始化无可用块计数
    s_no_block_count = 0U;

    // ⑥ 初始化读取状态
    s_next_read_block = ADC_ACQ_BLOCK_HALF; // 优先从前半开始
    s_read_block = ADC_ACQ_BLOCK_HALF;
    s_read_group_index = 0U;
    s_block_ready = 0U;
    s_sample_seq = 0U;
}
/**
 * @brief 启动 ADC 采集服务
 * @details 执行 ADC 校准并启动 DMA 转移
 * @retval None
 *
 * @note 工作步骤：
 *       ① 停止之前的定时器和 DMA（以防重复启动）
 *       ② 对 3 个 ADC 执行偏移校准
 *       ③ 启动 3 个 ADC 的 DMA 转移
 *       ④ 启动定时器驱动 ADC 采样
 *
 * @warning 调用前必须先调用 adc_acq_service_init()
 */
void adc_acq_service_start(void)
{
    // ① 停止定时器（如果之前启动过）
    HAL_TIM_Base_Stop(&htim6);

    // ② 停止 3 个 ADC 的 DMA 转移（如果之前启动过）
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);

    // ③ 执行 ADC 偏移校准
    //    校准模式：ADC_CALIB_OFFSET = 偏移校准
    //    校准类型：ADC_DIFFERENTIAL_ENDED = 差分模式
    //    校准通常耗时 400~500 us
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);

    // ④ 启动 3 个 ADC 的 DMA 转移
    //    ADC1：4 通道 × 512 组 = 2048 个 16 位数据
    //    ADC2：1 通道 × 512 组 = 512 个 16 位数据
    //    ADC3：7 通道 × 512 组 = 3584 个 16 位数据
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma_buf, ADC1_DMA_LENGTH);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma_buf, ADC2_DMA_LENGTH);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)s_adc3_dma_buf, ADC3_DMA_LENGTH);

    // ⑤ 启动定时器
    //    定时器用于周期性启动 ADC 转换
    //    频率应该与 ADC 采样率相匹配
    HAL_TIM_Base_Start(&htim6);
}

/**
 * @brief 获取一个 ADC 采样
 * @details 从已完成的 DMA 缓冲区中读取一个采样组（12 通道）
 * @param[out] sample: 输出采样结构
 * @retval uint8_t
 *         1 = 成功读取一个采样
 *         0 = 没有可用的采样（缓冲区未就绪或已全部读完）
 *
 * @note 工作流程：
 *       ① 检查是否有已锁定的缓冲块
 *       ② 如果没有，尝试锁定下一个就绪的块
 *       ③ 如果无可用块，记录丢失计数并返回 0
 *       ④ 根据读取位置计算 ADC1、ADC2、ADC3 的数据偏移
 *       ⑤ 设置采样的时间戳和 12 个通道的原始值
 *       ⑥ 更新读取索引
 *       ⑦ 当块中所有采样读完时，清除就绪标志
 *       ⑧ 返回 1 表示成功
 */
uint8_t adc_acq_service_get_sample(adc_acq_sample_t *sample)
{
    uint32_t adc1_base;
    uint32_t adc2_base;
    uint32_t adc3_base;
    uint32_t group_base;

    // ① 验证输入指针
    if (NULL == sample)
    {
        return 0U;
    }

    // ② 如果当前没有就绪的块，尝试锁定下一个
    if (0U == s_block_ready)
    {
        if (0U == adc_acq_service_lock_next_block())
        {
            // 没有就绪的块，记录丢失
            s_no_block_count++;
            return 0U;
        }
    }

    // ③ 计算当前采样组的绝对索引
    //    group_base = (当前块的起始索引) + (块内采样组索引)
    //    = (s_read_block × ADC_ACQ_HALF_COUNT) + s_read_group_index
    group_base = ((uint32_t)s_read_block * ADC_ACQ_HALF_COUNT) + s_read_group_index;

    // ④ 计算在各 ADC DMA 缓冲区中的偏移
    //    每个 ADC 通道按顺序存储，所以需要乘以通道数
    adc1_base = group_base * ADC1_CHANNELS_PER_GROUP;
    adc2_base = group_base * ADC2_CHANNELS_PER_GROUP;
    adc3_base = group_base * ADC3_CHANNELS_PER_GROUP;

    // ⑤ 填充采样结构
    sample->timestamp_us = s_sample_seq;

    // ⑥ 从 ADC1 DMA 缓冲区读取 4 个通道 (CH0~CH3)
    sample->raw[0] = s_adc1_dma_buf[adc1_base + 0U];
    sample->raw[1] = s_adc1_dma_buf[adc1_base + 1U];
    sample->raw[2] = s_adc1_dma_buf[adc1_base + 2U];
    sample->raw[3] = s_adc1_dma_buf[adc1_base + 3U];

    // ⑦ 从 ADC2 DMA 缓冲区读取 1 个通道 (CH4)
    sample->raw[4] = s_adc2_dma_buf[adc2_base + 0U];

    // ⑧ 从 ADC3 DMA 缓冲区读取 7 个通道 (CH5~CH11)
    sample->raw[5] = s_adc3_dma_buf[adc3_base + 0U];
    sample->raw[6] = s_adc3_dma_buf[adc3_base + 1U];
    sample->raw[7] = s_adc3_dma_buf[adc3_base + 2U];
    sample->raw[8] = s_adc3_dma_buf[adc3_base + 3U];
    sample->raw[9] = s_adc3_dma_buf[adc3_base + 4U];
    sample->raw[10] = s_adc3_dma_buf[adc3_base + 5U];
    sample->raw[11] = s_adc3_dma_buf[adc3_base + 6U];

    /*
     * 这组 12 路 ADC 数据已经完整取出。
     * 状态服务只做统计缓存，后续心跳可以直接读取最近一次平均值。
     */
    adc_status_service_update_sample(sample);

    // ⑨ 更新序列号和读取索引
    s_sample_seq++;
    s_read_group_index++;

    // ⑩ 检查是否已读完当前块
    if (s_read_group_index >= ADC_ACQ_HALF_COUNT)
    {
        // 块已读完，清除就绪标志，重置索引，准备读下一个块
        s_block_ready = 0U;
        s_read_group_index = 0U;
    }

    return 1U;
}

/* ========================= DMA Callbacks ========================= */

/**
 * @brief ADC DMA 前半缓冲区完成回调
 * @details 当 DMA 完成前半缓冲区转移时由 HAL 库调用（在中断中）
 * @param[in] hadc: ADC 句柄指针
 * @retval None
 *
 * @note 工作流程：
 *       ① 识别是哪个 ADC 的前半缓冲完成
 *       ② 设置对应的就绪标志为 1
 *       ③ 递增对应的完成计数
 *
 * @warning 此函数在中断上下文中运行，应保持简短
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // ① 检查是 ADC1 的前半缓冲完成
    if (hadc->Instance == ADC1)
    {
        s_adc1_half_ready = 1U;
        s_adc1_half_count++;
    }
    // ② 检查是 ADC2 的前半缓冲完成
    else if (hadc->Instance == ADC2)
    {
        s_adc2_half_ready = 1U;
        s_adc2_half_count++;
    }
    // ③ 检查是 ADC3 的前半缓冲完成
    else if (hadc->Instance == ADC3)
    {
        s_adc3_half_ready = 1U;
        s_adc3_half_count++;
    }
}

/**
 * @brief ADC DMA 后半缓冲区完成回调
 * @details 当 DMA 完成后半缓冲区转移时由 HAL 库调用（在中断中）
 * @param[in] hadc: ADC 句柄指针
 * @retval None
 *
 * @note 工作流程：
 *       ① 识别是哪个 ADC 的后半缓冲完成
 *       ② 设置对应的就绪标志为 1
 *       ③ 递增对应的完成计数
 *
 * @warning 此函数在中断上下文中运行，应保持简短
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // ① 检查是 ADC1 的后半缓冲完成
    if (hadc->Instance == ADC1)
    {
        s_adc1_full_ready = 1U;
        s_adc1_full_count++;
    }
    // ② 检查是 ADC2 的后半缓冲完成
    else if (hadc->Instance == ADC2)
    {
        s_adc2_full_ready = 1U;
        s_adc2_full_count++;
    }
    // ③ 检查是 ADC3 的后半缓冲完成
    else if (hadc->Instance == ADC3)
    {
        s_adc3_full_ready = 1U;
        s_adc3_full_count++;
    }
}

/* ========================= Statistics Query ========================= */

/**
 * @brief 获取 ADC 采集统计信息
 * @details 返回 DMA 完成计数、采样序列号和丢失计数
 * @param[out] stats: 输出统计结构
 * @retval None
 *
 * @note 内容：
 *       - adc1_half_count：ADC1 前半缓冲完成次数
 *       - adc2_half_count：ADC2 前半缓冲完成次数
 *       - adc3_half_count：ADC3 前半缓冲完成次数
 *       - adc1_full_count：ADC1 后半缓冲完成次数
 *       - adc2_full_count：ADC2 后半缓冲完成次数
 *       - adc3_full_count：ADC3 后半缓冲完成次数
 *       - sample_seq：已读取的采样总数
 *       - no_block_count：无可用块的次数（数据丢失指示）
 */
void adc_acq_service_get_stats(adc_acq_stats_t *stats)
{
    // ① 验证输入指针
    if (NULL == stats)
    {
        return;
    }

    // ② 复制前半缓冲完成计数
    stats->adc1_half_count = s_adc1_half_count;
    stats->adc2_half_count = s_adc2_half_count;
    stats->adc3_half_count = s_adc3_half_count;

    // ③ 复制后半缓冲完成计数
    stats->adc1_full_count = s_adc1_full_count;
    stats->adc2_full_count = s_adc2_full_count;
    stats->adc3_full_count = s_adc3_full_count;

    // ④ 复制采样序列号和丢失计数
    stats->sample_seq = s_sample_seq;
    stats->no_block_count = s_no_block_count;
}

/* ========================= Internal Helpers ========================= */

/**
 * @brief 尝试锁定指定的 DMA 块
 * @details 检查指定块是否被 3 个 ADC 都标记为就绪
 *          如果就绪，则锁定此块，清除就绪标志，更新下次优先块
 * @param[in] block: ADC_ACQ_BLOCK_HALF 或 ADC_ACQ_BLOCK_FULL
 * @retval uint8_t
 *         1 = 成功锁定（所有 ADC 对应块都就绪）
 *         0 = 无法锁定（至少一个 ADC 的块未就绪）
 *
 * @note 锁定逻辑（以前半块为例）：
 *       ① 检查 s_adc1_half_ready、s_adc2_half_ready、s_adc3_half_ready
 *       ② 全部为 1，则所有 ADC 前半缓冲都完成，可以锁定
 *       ③ 锁定后：
 *          - 设置 s_read_block = ADC_ACQ_BLOCK_HALF
 *          - 清除三个就绪标志
 *          - 设置 s_next_read_block = ADC_ACQ_BLOCK_FULL（下次优先读后半）
 *          - 重置读取索引和就绪标志
 */
static uint8_t adc_acq_service_try_lock_block(uint8_t block)
{
    // ① 前半块锁定
    if (ADC_ACQ_BLOCK_HALF == block)
    {
        // ① 检查所有 ADC 的前半缓冲是否都就绪
        if ((0U == s_adc1_half_ready) ||
            (0U == s_adc2_half_ready) ||
            (0U == s_adc3_half_ready))
        {
            return 0U; // 至少一个 ADC 未就绪，无法锁定
        }

        // ② 清除就绪标志（表示已锁定）
        s_adc1_half_ready = 0U;
        s_adc2_half_ready = 0U;
        s_adc3_half_ready = 0U;

        // ③ 设置读取块和下次优先块
        s_read_block = ADC_ACQ_BLOCK_HALF;
        s_next_read_block = ADC_ACQ_BLOCK_FULL;
    }
    // ② 后半块锁定
    else
    {
        // ① 检查所有 ADC 的后半缓冲是否都就绪
        if ((0U == s_adc1_full_ready) ||
            (0U == s_adc2_full_ready) ||
            (0U == s_adc3_full_ready))
        {
            return 0U; // 至少一个 ADC 未就绪，无法锁定
        }

        // ② 清除就绪标志（表示已锁定）
        s_adc1_full_ready = 0U;
        s_adc2_full_ready = 0U;
        s_adc3_full_ready = 0U;

        // ③ 设置读取块和下次优先块
        s_read_block = ADC_ACQ_BLOCK_FULL;
        s_next_read_block = ADC_ACQ_BLOCK_HALF;
    }

    // ③ 重置读取索引和就绪标志
    s_read_group_index = 0U;
    s_block_ready = 1U;

    return 1U; // 锁定成功
}

/**
 * @brief 锁定下一个可用的 DMA 块
 * @details 使用"乒乓"策略：
 *          ① 优先尝试锁定 s_next_read_block（从上次读完的块的另一个）
 *          ② 如果优先块未就绪，尝试备用块
 *          ③ 如果两个块都未就绪，返回 0
 * @retval uint8_t
 *         1 = 成功锁定（优先块或备用块之一就绪）
 *         0 = 两个块都未就绪
 *
 * @note "乒乓"策略的优势：
 *       假设优先读前半，读完后优先读后半：
 *       - 当读前半时，DMA 在向后半写数据
 *       - 当读后半时，DMA 在向前半写数据
 *       - 这样可以最大化地避免冲突和等待
 */
static uint8_t adc_acq_service_lock_next_block(void)
{
    uint8_t fallback_block;

    // ① 优先尝试 s_next_read_block
    if (0U != adc_acq_service_try_lock_block(s_next_read_block))
    {
        return 1U; // 优先块就绪，锁定成功
    }

    // ② 计算备用块（与优先块相反）
    fallback_block = (ADC_ACQ_BLOCK_HALF == s_next_read_block)
                         ? ADC_ACQ_BLOCK_FULL
                         : ADC_ACQ_BLOCK_HALF;

    // ③ 尝试锁定备用块
    return adc_acq_service_try_lock_block(fallback_block);
}
