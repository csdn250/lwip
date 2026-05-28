# 2026-05-28 DAC SPI 调试记录

## 今日目标

验证 STM32H743 通过 SPI1 控制 TPC112S4 四通道 DAC 的基础通信时序，确认逻辑分析仪能稳定解析出 MCU 发出的 16 bit 控制帧，并为后续 DAC 输出闭环控制做准备。

## 当前硬件连接

- MCU: STM32H743IITx
- DAC: TPC112S4, 12 bit, 4 通道
- SPI: SPI1, 只发送模式
- SCK: PG11
- MOSI: PD7
- SYNC: PB12, GPIO 输出, 低有效
- LOAD: PB13, GPIO 输出, 低脉冲更新输出

## DAC 帧格式

TPC112S4 每次写入 16 bit:

```text
D15..D13: 通道选择
D12     : PD, 掉电控制位
D11..D0 : 12 bit DAC 数据
```

当前测试帧:

```text
通道 A: 0x8000
通道 B: 0xA555
通道 C: 0xCAAA
通道 D: 0xEFFF
```

## 出现的问题

### 1. Saleae 解码结果和发送数据不一致

现象:

逻辑分析仪解码时出现过 `0xD2AA`、`0xE555`、`0x0005`、`0x000B` 等异常值，不符合软件发送的测试帧。

判断:

问题不是 DAC 帧组包错误。当前组包方式为:

```c
frame = (channel << 13) | (code & 0x0FFF);
```

该方式能生成预期测试帧。异常更可能来自 SPI 时钟模式、SCK 空闲态、SYNC 建立时间或逻辑分析仪采样配置。

### 2. Saleae 提示 SCK 空闲态不匹配

现象:

逻辑分析仪出现提示:

```text
The initial (idle) state of the CLK line does not match the settings.
```

判断:

当分析仪配置为 CPOL=0 时，它期望片选有效前后 SCK 空闲为低电平。如果实际 SCK 空闲为高电平或不稳定，后续按 CPOL=0/CPHA=1 解码就不可信。

处理方向:

- STM32 SPI 配置保持 `CLKPolarity = SPI_POLARITY_LOW`
- `CLKPhase = SPI_PHASE_2EDGE`
- 启用 `MasterKeepIOState`
- SCK GPIO 加下拉，保证空闲低电平稳定

### 3. SYNC 拉低后第一位建立时间不足

现象:

SYNC 拉低后马上启动 SPI，第一组 SCK 和第一位 MOSI 之间时间太紧，可能导致第一位被 DAC 或分析仪错误采样。

处理方向:

在 `SYNC` 拉低后增加很短的软件延迟:

```c
DAC_SYNC_LOW();
dac_tpc112s4_delay_short();
HAL_SPI_Transmit(...);
```

注意:

该延迟用于补足纳秒到微秒级建立时间，不能使用 `HAL_Delay(1)`。`HAL_Delay(1)` 是毫秒级，会把 DAC 刷新率直接拖慢到几百 Hz。

### 4. LOAD 更新脉冲影响刷新频率

现象:

早期使用较大的软件延迟或主循环 `HAL_Delay()` 时，四通道更新周期被拖到毫秒级。

判断:

LOAD 只需要满足芯片手册要求的最小脉冲宽度，不应该使用毫秒级延迟。后续如果要做高速、确定性的 DAC 同步更新，可以考虑由硬件定时器/PWM 产生 LOAD 脉冲。

## 当前排查方法

1. 先用低 SPI 速率测试，例如 `SPI_BAUDRATEPRESCALER_64` 或 `16`，确认协议相位和帧值正确。
2. 逻辑分析仪配置为 16 bit、MSB first、Enable active low。
3. 根据手册“下降沿移入 DIN”，优先验证 `CPOL=0, CPHA=1`。
4. 如果 Saleae 提示 SCK idle mismatch，先处理 SCK 空闲态，不盲目相信解码值。
5. 先增大 `DAC_TIMING_DELAY_CYCLES` 验证稳定性，再逐步缩小，找到可靠边界。
6. 确认四个测试帧是否稳定为:

```text
0x8000, 0xA555, 0xCAAA, 0xEFFF
```

## 当前代码状态

已新增 DAC 测试驱动:

- `Core/Inc/dac_tpc112s4.h`
- `Core/Src/dac_tpc112s4.c`

已新增 ADC 采集服务雏形:

- `Core/Inc/adc_acq_service.h`
- `Core/Src/adc_acq_service.c`

SPI 当前关键配置:

```c
hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
hspi1.Init.NSS = SPI_NSS_SOFT;
hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
```

SCK GPIO 当前建议:

```c
GPIO_InitStruct.Pull = GPIO_PULLDOWN;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
```

## 后续计划

1. 继续用逻辑分析仪确认 `CPOL=0, CPHA=1` 下是否能稳定解析四个测试帧。
2. 如果稳定，逐步提高 SPI 速率，记录不同分频下的稳定性。
3. 将测试用的连续 `dac_tpc112s4_test_pattern()` 改为可控测试接口，避免正式业务中无条件循环发送。
4. 进一步评估 LOAD 是否改为定时器/PWM 输出，提高四通道同步刷新确定性。
5. DAC 芯片到货后，用示波器验证四路模拟输出是否与码值匹配。

