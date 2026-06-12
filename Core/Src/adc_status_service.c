#include "adc_status_service.h"
#include <string.h>

#define ADC_STATUS_AVG_WINDOW_SAMPLES 1024U
#define ADC_STATUS_MAX_SAMPLES_PER_PROCESS 8U

static uint64_t s_adc_sum[ADC_ACQ_CHANNEL_COUNT];
static float s_adc_avg[ADC_ACQ_CHANNEL_COUNT];
static uint32_t s_adc_sum_count;
static uint32_t s_last_avg_count;
static uint32_t s_state_flags;
static uint32_t s_alarm_flags;

void adc_status_service_init(void)
{
    memset(s_adc_sum, 0, sizeof(s_adc_sum));
    memset(s_adc_avg, 0, sizeof(s_adc_avg));
    s_state_flags = 0U;
    s_alarm_flags = 0U;
    s_adc_sum_count = 0U;
    s_last_avg_count = 0U;
}

void adc_status_service_process(void)
{
    adc_acq_sample_t sample;
    uint8_t count;

    /*
     * 后台状态采样器：
     * 当 TCP 流和 DAC 级联都没有消费 ADC 时，主循环会调用此函数。
     * 每轮最多取少量样本，避免为了心跳统计长时间占用 CPU。
     */
    for (count = 0U;
         count < ADC_STATUS_MAX_SAMPLES_PER_PROCESS;
         count++)
    {
        if (0U == adc_acq_service_get_sample(&sample))
        {
            return;
        }
        /*
         * 注意：adc_acq_service_get_sample() 内部已经会调用
         * adc_status_service_update_sample()。
         * 所以这里不需要再次 update，否则会重复累计。
         */
    }
}

void adc_status_service_update_sample(const adc_acq_sample_t *sample)
{
    uint8_t ch;

    if (NULL == sample)
    {
        return;
    }

    for (ch = 0U; ch < ADC_ACQ_CHANNEL_COUNT; ch++)
    {
        s_adc_sum[ch] += sample->raw[ch];
    }

    s_adc_sum_count++;

    if (s_adc_sum_count >= ADC_STATUS_AVG_WINDOW_SAMPLES)
    {
        for (ch = 0U; ch < ADC_ACQ_CHANNEL_COUNT; ch++)
        {
            s_adc_avg[ch] = (float)s_adc_sum[ch] / (float)s_adc_sum_count;
            s_adc_sum[ch] = 0U;
        }

        s_last_avg_count = s_adc_sum_count;
        s_adc_sum_count = 0U;
    }
}

void adc_status_service_set_runtime_flags(uint32_t state_flags,
                                          uint32_t alarm_flags)
{
    s_state_flags = state_flags;
    s_alarm_flags = alarm_flags;
}

void adc_status_service_get_snapshot(adc_status_snapshot_t *snapshot)
{
    if (NULL == snapshot)
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->alarm_flags = s_alarm_flags;
    snapshot->state_flags = s_state_flags;
    snapshot->avg_sample_count = s_last_avg_count;
    memcpy(snapshot->adc_avg, s_adc_avg, sizeof(s_adc_avg));
}
