# adc_collect

STM32H743 + lwIP based AD/DA data collection firmware.

本工程当前目标是实现多通道 ADC 采集、四通道 DAC 输出，以及基于以太网的上位机参数配置和数据上传。设备启动后提供 TCP 服务；没有 TCP 客户端连接时，通过 UDP 周期广播设备信息，方便上位机发现设备。

## 当前状态

- MCU: STM32H743
- TCP Server 端口: `8080`
- UDP 发现广播端口: `8081`
- 默认设备 IP: `192.168.1.21`
- 默认上位机 IP: `192.168.1.20`
- 默认 MAC: `02:00:00:00:00:21`
- 默认设备名: `ADDA_COLLECT_1`
- ADC: 12 个逻辑通道，ADC1/ADC2/ADC3 由 TIM6 统一触发
- DAC: 4 个输出通道，底层驱动为 `dac_tpc112s4`
- 参数存储: EEPROM 框架已预留，目标器件为 `24C64`

## 主循环

主循环当前按下面顺序运行：

```c
MX_LWIP_Process();
adc_tcp_server_process();
dac_output_service_process();

if (0U == adc_tcp_server_is_streaming())
{
  dac_output_service_process_adc_cascade();
}

udp_discovery_process();
```

含义：

- `MX_LWIP_Process()` 必须高频轮询，驱动 lwIP raw API 收发。
- `adc_tcp_server_process()` 处理 TCP 命令和 ADC 数据流。
- `dac_output_service_process()` 处理 DAC 手动输出配置变化。
- 当 TCP ADC 数据流未开启时，DAC 级联服务可独立消费 ADC 样本。
- 当 TCP ADC 数据流开启时，TCP 数据流取样后顺便喂给 DAC，避免两个模块同时抢 ADC 样本。
- 没有 TCP 客户端连接时，UDP 发现服务每 5 秒广播一次设备信息。

## 协议帧格式

TCP 命令和 UDP 发现广播使用统一外层帧结构：

```text
12 34 CMD LEN_H LEN_L PAYLOAD CRC32_3 CRC32_2 CRC32_1 CRC32_0 56 78
```

字段说明：

- `12 34`: 帧头
- `CMD`: 命令字
- `LEN_H LEN_L`: payload 长度，大端
- `PAYLOAD`: 数据区
- `CRC32`: 对 `帧头 + CMD + 长度 + PAYLOAD` 计算 CRC32/IEEE，发送时大端
- `56 78`: 帧尾

TCP 是字节流，不是一包一条命令。固件会先把收到的数据复制到接收缓冲区，释放 lwIP `pbuf`，再从缓冲区中按帧头、长度、CRC 和帧尾切出完整命令并顺序处理。

### 固定 150 字节命令帧

当前上位机下发的普通 TCP 命令使用固定 150 字节帧，适用于 `0x01` 写参数、`0x02` 读参数、`0x07` 心跳：

```text
12 34 CMD LEN_H LEN_L BLOCK_ID_H BLOCK_ID_L DATA... PAD... CRC32 56 78
```

字段说明：

- 固定帧总长度为 150 字节。
- `LEN` 表示 `DATA` 的有效长度，不包含 `BLOCK_ID`。
- `BLOCK_ID` 固定占 2 字节，位于第 5~6 字节。
- `DATA` 从第 7 字节开始，最大 137 字节，不足部分补 0。
- `CRC32` 位于第 144~147 字节，对第 0~143 字节计算，发送时大端。
- 帧尾 `56 78` 位于第 148~149 字节。

ADC 数据流 `0x81/0x82` 仍使用可变长度帧，用于提高连续采集数据的发送效率。

## 命令字

| 命令 | 方向 | 说明 |
| --- | --- | --- |
| `0x01` | PC -> MCU / MCU -> PC | 写参数块，以及写结果响应 |
| `0x02` | PC -> MCU / MCU -> PC | 读参数块，以及读结果响应 |
| `0x07` | PC -> MCU / MCU -> PC | 心跳 |
| `0x71` | MCU -> UDP broadcast | 设备发现/状态广播 |
| `0x81` | MCU -> PC | ADC raw 数据流 |
| `0x82` | MCU -> PC | ADC converted 数据流 |
| `0x83` | MCU -> PC | 调试状态帧，受 `ADC_TCP_DEBUG_STATUS_ENABLE` 控制 |

## 参数块

写参数 payload 格式：

```text
BLOCK_ID_H BLOCK_ID_L DATA...
```

读参数 payload 格式：

```text
BLOCK_ID_H BLOCK_ID_L
```

| Block ID | 说明 |
| --- | --- |
| `0x0001` | ADC 标定参数，12 通道，每通道 `k_raw` + `b_raw` |
| `0x0002` | 控制参数，启动/停止 ADC 数据上传 |
| `0x0003` | 配置参数预留 |
| `0x0004` | 设备 IP / 上位机 IP |
| `0x0005` | 设备 MAC |
| `0x0006` | 设备端口 / 上位机端口 |
| `0x0007` | 子网掩码 |
| `0x0008` | 网关 |
| `0x0009` | DA1 配置 |
| `0x000A` | DA2 配置 |
| `0x000B` | DA3 配置 |
| `0x000C` | DA4 配置 |
| `0x000D` | RAM 日志快照，只读；写入 `01` 可清空 RAM 日志 |

### ADC 标定参数

`0x0001` 数据区长度为 96 字节：

```text
CH1 k_raw int32_be
CH1 b_raw int32_be
...
CH12 k_raw int32_be
CH12 b_raw int32_be
```

换算规则：

```text
adc_value = adc_raw * (k_raw * 0.00000001) + b_raw
```

### 控制参数

`0x0002` 当前使用前两个字节：

```text
enable stream_type
```

示例：

```text
01 81   启动 raw ADC 数据上传
01 82   启动 converted ADC 数据上传
00 81   停止 raw ADC 数据上传
00 82   停止 converted ADC 数据上传
```

### DA 通道配置

`0x0009` - `0x000C` 每个 DA 通道参数块长度为 14 字节：

```text
mode        1 byte
manual_raw  4 bytes, int32_be
adc_channel 1 byte
k_raw       4 bytes, int32_be
b_raw       4 bytes, int32_be
```

字段含义：

- `mode = 0`: 手动输出，上位机下发 `manual_raw`
- `mode = 1`: ADC 级联输出，选择 `adc_channel`
- `manual_raw`: 手动模式下暂按 DAC 整数码值使用，范围最终限幅到 `0..4095`
- `adc_channel`: 级联模式下选择 ADC 逻辑通道 `0..11`
- `k_raw / b_raw`: DAC 标定参数

协议决策：DAC 手动输出后续统一改为上位机下发 `float32_be` 电压值，单片机根据 DAC 标定参数换算为 DAC 整数码值。当前 `manual_raw` 是过渡字段，等待 DAC 参数块格式升级后替换。

ADC 到 DAC 级联计算：

```text
adc_value = adc_raw * adc_k + adc_b
dac_value = adc_value * dac_k + dac_b
dac_code  = round(dac_value), then limit to 0..4095
```

### RAM 日志快照

`0x000D` 是动态只读参数块，不写入 EEPROM。它从 RAM 环形日志中返回最近事件，便于现场排查。当前 RAM 日志容量为 64 条，单次固定帧最多返回最近 8 条。

读 `0x000D` 返回的数据格式：

```text
version      1 byte, 当前为 0x01
count        1 byte, 本帧日志条数
record_size  1 byte, 当前为 16
reserved     1 byte

records...
```

每条日志 16 字节：

```text
tick_ms  uint32_be
event    uint16_be
arg0     uint16_be
arg1     uint32_be
arg2     uint32_be
```

清空 RAM 日志：

```text
写 block 0x000D，DATA = 01
```

清空成功后，固件会重新记录一条 `LOG_CLEARED`，避免日志快照完全为空。

调试版支持 IWDG 看门狗复位测试：

```text
写 block 0x000D，DATA = A5
```

该命令会停止主循环喂狗，约 4 秒后由 IWDG 触发复位。复位后读取日志，`RESET_CAUSE` 的 `arg1` 应包含 `0x10`。该命令受固件宏 `ADC_TCP_WATCHDOG_TEST_ENABLE` 控制，正式发布版本可关闭。

## ADC 数据流

ADC 数据流 payload 内部还有一层数据头，所有多字节字段当前统一使用大端：

```text
magic          uint32  0x41444331, ASCII "ADC1"
seq            uint32  样本序号
timestamp_us   uint32  当前暂用样本序号/时间戳字段
channel_mask   uint16
channel_count  uint16
sample_format  uint16  0x81 raw, 0x82 converted float
payload_bytes  uint16
sample_data
```

raw 数据：

```text
uint16_be, 按样本组排列，每组 12 通道
```

converted 数据：

```text
float32_be, 按样本组排列，每组 12 通道
```

当前批量发送策略：

- raw: 每帧最多 32 组样本
- converted: 每帧最多 16 组样本

## UDP 发现广播

当没有 TCP 客户端连接时，设备每 5 秒向 `255.255.255.255:8081` 发送一次发现广播。建立 TCP 连接后广播停止。

UDP payload 当前内容：

```text
"ADDA" + version + status + mac + ip + netmask + gateway + tcp_port + device_name
```

Wireshark 过滤器：

```text
udp.port == 8081
```

## 关键源码

| 文件 | 说明 |
| --- | --- |
| `Core/Src/main.c` | 外设初始化和主循环调度 |
| `Core/Src/adc_acq_service.c` | ADC DMA 半/全缓冲采集服务 |
| `Core/Src/adc_tcp_server.c` | TCP 监听、收包缓存、拆帧、CRC 校验、命令分发、ADC 数据流 |
| `Core/Src/adc_frame_builder.c` | ADC raw/converted 数据帧构建 |
| `Core/Src/device_config.c` | 网络、ADC 标定、DAC 配置的运行时镜像 |
| `Core/Src/dac_tpc112s4.c` | DAC 控制芯片底层 SPI 输出 |
| `Core/Src/dac_output_service.c` | DAC 手动输出和 ADC 级联输出服务 |
| `Core/Src/udp_discovery.c` | UDP 发现广播 |
| `Core/Src/eeprom_storage.c` | 24C64 EEPROM 读写框架 |
| `Core/Src/app_log.c` | RTT 日志和 RAM 环形事件日志 |
| `tools/make_fixed_tcp_frame.py` | 生成 NetAssist 可发送的固定 150 字节 HEX 命令帧 |
| `tools/read_log_snapshot.py` | 读取并解析 `0x000D` RAM 日志快照 |
| `tools/tcp_stream_monitor.py` | 接收并解析 ADC 数据流，统计速率、gap、overlap、bad |

## 常用测试命令

生成心跳帧：

```powershell
python tools\make_fixed_tcp_frame.py 07 00 00
```

读取 MAC 参数块：

```powershell
python tools\make_fixed_tcp_frame.py 02 00 05
```

配置 DA1 手动输出 2048：

```powershell
python tools\make_fixed_tcp_frame.py 01 00 09 00 00 00 08 00 FF 05 F5 E1 00 00 00 00 00
```

配置 DA1 级联 ADC0：

```powershell
python tools\make_fixed_tcp_frame.py 01 00 09 01 00 00 00 00 00 05 F5 E1 00 00 00 00 00
```

启动 raw ADC 数据上传：

```powershell
python tools\make_fixed_tcp_frame.py 01 00 02 01 81
```

停止 raw ADC 数据上传：

```powershell
python tools\make_fixed_tcp_frame.py 01 00 02 00 81
```

读取 RAM 日志快照：

```powershell
cd C:\Users\myw29\Desktop\ADDA_collect\adc_collect\tools
python .\read_log_snapshot.py --host 192.168.1.21 --bind 192.168.1.20
```

生成清空 RAM 日志命令：

```powershell
python tools\make_fixed_tcp_frame.py 01 00 0D 01
```

生成 IWDG 看门狗复位测试命令：

```powershell
python tools\make_fixed_tcp_frame.py 01 00 0D A5
```

Python 监控 raw 数据流：

```powershell
cd C:\Users\myw29\Desktop\ADDA_collect\adc_collect\tools
python .\tcp_stream_monitor.py --host 192.168.1.21 --bind 192.168.1.20 --type raw --duration 5
```

Python 监控 converted 数据流：

```powershell
cd C:\Users\myw29\Desktop\ADDA_collect\adc_collect\tools
python .\tcp_stream_monitor.py --host 192.168.1.21 --bind 192.168.1.20 --type converted --duration 5
```

指定源 IP 测试 ping：

```powershell
ping -S 192.168.1.20 192.168.1.21
```

## 已验证功能

- `ping -S 192.168.1.20 192.168.1.21` 正常
- TCP 连接、心跳、读参数、写参数正常
- TCP 字节流拆帧和 CRC32 校验正常
- UDP 发现广播可被 Wireshark 捕获
- ADC raw 数据上传可被 Python 正常解析
- ADC converted 数据上传可被 Python 正常解析
- ADC 标定参数可通过 TCP 下发
- IP/MAC/端口/子网掩码/网关可通过 TCP 下发并同步到 lwIP 当前配置
- DAC 手动输出参数可通过 TCP 下发和读回
- DAC ADC 级联软件链路已跑通，等待实际 DAC 控制芯片到货后验证电气输出
- RAM 日志快照 `0x000D` 可通过 TCP 读取，写入 `01` 可清空日志
- IWDG 看门狗已启用，正常主循环会定期喂狗；调试命令 `0x000D/A5` 已验证可触发 IWDG 复位，复位原因日志包含 `0x10`

最近一次 ADC raw + DAC 级联软件测试结果：

```text
gap=0
overlap=0
bad=0
avg_per_ch≈70 kSa/s
```

## 后续计划

- 用实际 DAC 控制芯片验证 DA1~DA4 输出电压
- 将 DAC 手动输出参数从过渡期 `manual_raw` 码值升级为 `float32_be` 电压值
- 根据实际 DAC 性能决定是否降低级联更新频率或优化 SPI 发送
- 把 ADC 标定、DAC 配置、网络参数统一纳入可靠 EEPROM 参数区
- 整理正式上位机协议文档
- 继续清理早期调试遗留注释和编码乱码
- 预留 RS485 服务层：收发方向控制、协议解析、联调测试

## 构建

工程使用 Keil MDK-ARM 构建：

```text
MDK-ARM/adc_collect.uvprojx
```

当前目标使用 ARMCC V5.06。
