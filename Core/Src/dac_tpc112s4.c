#include "dac_tpc112s4.h"
#include "spi.h"

#define DAC_TPC112S4_CH_A 4U
#define DAC_TPC112S4_CH_B 5U
#define DAC_TPC112S4_CH_C 6U
#define DAC_TPC112S4_CH_D 7U

#define DAC_SYNC_LOW() (DAC_SYNC_GPIO_Port->BSRR = ((uint32_t)DAC_SYNC_Pin << 16U))
#define DAC_SYNC_HIGH() (DAC_SYNC_GPIO_Port->BSRR = (uint32_t)DAC_SYNC_Pin)

#define DAC_LOAD_LOW() (DAC_LOAD_GPIO_Port->BSRR = ((uint32_t)DAC_LOAD_Pin << 16U))
#define DAC_LOAD_HIGH() (DAC_LOAD_GPIO_Port->BSRR = (uint32_t)DAC_LOAD_Pin)

#define DAC_TIMING_DELAY_CYCLES 4U

static uint16_t dac_tpc112s4_make_frame(uint8_t channel,
                                        uint16_t code)
{
    code &= 0x0FFFU;
    return (uint16_t)(((uint16_t)channel << 13) | code);
}

static void dac_tpc112s4_delay_short(void)
{
    volatile uint32_t i;

    for (i = 0U; i < DAC_TIMING_DELAY_CYCLES; ++i)
    {
        __NOP();
    }
}

static void dac_tpc112s4_write_frame(uint16_t frame)
{
    DAC_SYNC_LOW();

    /* 给 DAC 一个 SYNC 拉低后的建立时间，保证第一位不会被吃掉。 */
    dac_tpc112s4_delay_short();

    HAL_SPI_Transmit(&hspi1,
                     (uint8_t *)&frame,
                     1U,
                     100U);

    /* 等最后一个时钟结束后，再释放 SYNC。 */
    dac_tpc112s4_delay_short();

    DAC_SYNC_HIGH();

    /* 给下一帧留出 SYNC 高电平间隔。 */
    dac_tpc112s4_delay_short();
}

static void dac_tpc112s4_load_update(void)
{
    DAC_LOAD_LOW();

    /* LOAD 低脉冲宽度，不能用 HAL_Delay，太慢。 */
    dac_tpc112s4_delay_short();

    DAC_LOAD_HIGH();
}

void dac_tpc112s4_write_channel(uint8_t channel,
                                uint16_t code)
{
    uint8_t hw_channel;

    if (DAC_TPC112S4_CHANNEL_COUNT <= channel)
    {
        return;
    }

    hw_channel = (uint8_t)(DAC_TPC112S4_CH_A + channel);

    dac_tpc112s4_write_frame(dac_tpc112s4_make_frame(hw_channel,
                                                     code));
    dac_tpc112s4_load_update();
}

void dac_tpc112s4_write_all(uint16_t ch1,
                            uint16_t ch2,
                            uint16_t ch3,
                            uint16_t ch4)
{
    dac_tpc112s4_write_frame(dac_tpc112s4_make_frame(DAC_TPC112S4_CH_A, ch1));
    dac_tpc112s4_write_frame(dac_tpc112s4_make_frame(DAC_TPC112S4_CH_B, ch2));
    dac_tpc112s4_write_frame(dac_tpc112s4_make_frame(DAC_TPC112S4_CH_C, ch3));
    dac_tpc112s4_write_frame(dac_tpc112s4_make_frame(DAC_TPC112S4_CH_D, ch4));

    dac_tpc112s4_load_update();
}
