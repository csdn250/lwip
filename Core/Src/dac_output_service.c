#include "dac_output_service.h"

#include "dac_tpc112s4.h"
#include "device_config.h"

#include <string.h>

#define DAC_OUTPUT_MAX_CASCADE_SAMPLES_PER_PROCESS 8U
static device_dac_config_t s_last_config;
static uint16_t s_current_code[DEVICE_CONFIG_DAC_CHANNEL_COUNT];

static uint8_t dac_output_service_config_changed(const device_dac_config_t
                                                     *dac_config);
static uint16_t dac_output_service_limit_code(int32_t value);
static uint16_t dac_output_service_manual_to_code(const device_dac_channel_config_t
                                                      *dac_ch_config);
static uint16_t dac_output_service_cascade_to_code(uint16_t adc_raw,
                                                   const device_adc_cal_channel_t
                                                       *adc_cal,
                                                   const device_dac_channel_config_t
                                                       *dac_config);

static int32_t dac_output_service_round_float_to_i32(float value);
static uint8_t dac_output_service_has_adc_cascade(const device_dac_config_t
                                                      *dac_config);

void dac_output_service_init(void)
{
    memset(&s_last_config, 0xFF, sizeof(s_last_config));
    memset(s_current_code, 0, sizeof(s_current_code));
}

void dac_output_service_process(void)
{
    const device_dac_config_t *dac_config;
    uint8_t ch;
    uint16_t code;

    dac_config = device_config_get_dac_config();

    if (0U == dac_output_service_config_changed(dac_config))
    {
        return;
    }

    for (ch = 0U; ch < DEVICE_CONFIG_DAC_CHANNEL_COUNT; ch++)
    {
        if (DEVICE_CONFIG_DAC_MODE_MANUAL == dac_config->ch[ch].mode)
        {
            code = dac_output_service_manual_to_code(&dac_config->ch[ch]);
            s_current_code[ch] = code;
            dac_tpc112s4_write_channel(ch, code);
        }
    }

    memcpy(&s_last_config, dac_config, sizeof(s_last_config));
}

void dac_output_service_process_adc_cascade(void)
{
    const device_dac_config_t *dac_config;
    adc_acq_sample_t sample;
    uint8_t count;

    dac_config = device_config_get_dac_config();

    if (0U == dac_output_service_has_adc_cascade(dac_config))
    {
        return;
    }

    /*
    如果没有任何 DA 通道是级联模式：
    什么都不做
    如果有 DA 通道是级联模式：
    从 ADC 采集服务最多取 8 组样本
    每取到一组，就执行一次 DAC 级联输出
    */
    for (count = 0U;
         count < DAC_OUTPUT_MAX_CASCADE_SAMPLES_PER_PROCESS;
         count++)
    {
        if (0U == adc_acq_service_get_sample(&sample))
        {
            return;
        }

        dac_output_service_apply_adc_sample(&sample);
    }
}

void dac_output_service_apply_adc_sample(const adc_acq_sample_t *sample)
{
    const device_dac_config_t *dac_config;
    const device_adc_cal_config_t *adc_cal;

    uint8_t da_ch;
    uint8_t adc_ch;
    uint16_t code;

    if (NULL == sample)
    {
        return;
    }

    dac_config = device_config_get_dac_config();
    adc_cal = device_config_get_adc_calibration();

    for (da_ch = 0U; da_ch < DEVICE_CONFIG_DAC_CHANNEL_COUNT; da_ch++)
    {
        if (DEVICE_CONFIG_DAC_MODE_ADC_CASCADE != dac_config->ch[da_ch].mode)
        {
            continue;
        }

        adc_ch = dac_config->ch[da_ch].adc_channel;
        if (adc_ch >= DEVICE_CONFIG_ADC_CHANNEL_COUNT)
        {
            continue;
        }

        code = dac_output_service_cascade_to_code(sample->raw[adc_ch],
                                                  &adc_cal->ch[adc_ch],
                                                  &dac_config->ch[da_ch]);

        s_current_code[da_ch] = code;
        dac_tpc112s4_write_channel(da_ch, code);
    }
}

void dac_output_service_get_current_codes(uint16_t *codes,
                                          uint8_t max_count)
{
    uint8_t ch;
    uint8_t count;

    if (NULL == codes)
    {
        return;
    }

    count = max_count;
    if (count > DEVICE_CONFIG_DAC_CHANNEL_COUNT)
    {
        count = DEVICE_CONFIG_DAC_CHANNEL_COUNT;
    }

    for (ch = 0U; ch < count; ch++)
    {
        codes[ch] = s_current_code[ch];
    }
}

static uint8_t dac_output_service_config_changed(const device_dac_config_t *dac_config)
{
    if (NULL == dac_config)
    {
        return 0U;
    }

    return (0 == memcmp(&s_last_config, dac_config, sizeof(s_last_config))
                ? 0U
                : 1U);
}

static uint8_t dac_output_service_has_adc_cascade(const device_dac_config_t *dac_config)
{
    uint8_t ch;

    if (NULL == dac_config)
    {
        return 0U;
    }

    for (ch = 0U; ch < DEVICE_CONFIG_DAC_CHANNEL_COUNT; ch++)
    {
        if (DEVICE_CONFIG_DAC_MODE_ADC_CASCADE == dac_config->ch[ch].mode)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint16_t dac_output_service_limit_code(int32_t value)
{
    if (value < 0L)
    {
        return 0U;
    }

    if (value > 4095L)
    {
        return 4095U;
    }

    return (uint16_t)value;
}

static uint16_t dac_output_service_manual_to_code(const device_dac_channel_config_t
                                                      *dac_ch_config)
{
    float dac_value;
    int32_t code;

    if (NULL == dac_ch_config)
    {
        return 0U;
    }

    dac_value = (dac_ch_config->manual_voltage * dac_ch_config->k) + dac_ch_config->b;

    code = dac_output_service_round_float_to_i32(dac_value);

    return dac_output_service_limit_code(code);
}

static uint16_t dac_output_service_cascade_to_code(uint16_t adc_raw,
                                                   const device_adc_cal_channel_t
                                                       *adc_cal,
                                                   const device_dac_channel_config_t
                                                       *dac_config)
{
    float adc_value;
    float dac_value;
    int32_t code;

    adc_value = ((float)adc_raw * adc_cal->k) + adc_cal->b;

    dac_value = (adc_value * dac_config->k) + dac_config->b;

    code = dac_output_service_round_float_to_i32(dac_value);

    return dac_output_service_limit_code(code);
}

static int32_t dac_output_service_round_float_to_i32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}
