# ADDA_collect 项目交接文档

生成时间：2026-06-01

## 1. 项目目标

项目基于 STM32H743IITx，目标是做一个 12 路 ADC 采集 + 以太网传输 + 四路 DAC 控制的采集控制板。

核心功能包括：

- 使用 ADC1、ADC2、ADC3 共 12 个采集点进行同步采集。
- ADC 采集数据经过 DMA 搬运到内存，由软件按固定顺序整理成统一帧格式。
- 上位机通过以太网获取采集数据。
- 上位机可选择接收原始数据，或接收经过标定公式换算后的模拟量数据。
- 标定参数通过 I2C 从 EEPROM 读取，上位机后续也要支持写入、更新 EEPROM 标定参数。
- 上位机可通过 TCP 下发 DAC 输出值，单片机通过 SPI 控制 TPC112S4 四通道 DAC。
- 后续还要支持 ADC 到 DAC 的内部级联控制：`adc_value -> k_ad/b_ad -> k_da/b_da -> dac_code`。
- 后续还要恢复设备发现、修改 IP、子网掩码、网关、MAC、设备名等网络管理功能。

## 2. 当前进度

当前工程路径：

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect
```

## 11. 2026-06-01 TCP/LwIP 排查记录

### 今天已经确认的现象

- 单片机 IP 为 `192.168.1.21`，电脑网卡 IP 为 `192.168.1.20`。
- 普通 `ping 192.168.1.21` 曾经失败，并出现来自 `192.168.1.2` 的“无法访问目标主机”。后来确认这会受 Windows 路由/网卡选择影响。
- 使用指定源地址的命令可以正常 ping 通：

```text
ping -S 192.168.1.20 192.168.1.21
```

- TCP 服务端端口 `8080` 已经可以被 NetAssist 连接。
- NetAssist 发送原始 HEX 心跳帧后，RTT 已经能看到：

```text
tcp accept
proto heartbeat
```

### 今天排查过程

1. 先用 RTT 确认主循环仍在跑，`link=1 up=1` 只能说明 PHY/netif 状态起来了，不等于 IP/ARP/TCP 一定正常。
2. 用 Wireshark 看 ARP 和 ICMP，发现电脑侧曾持续询问 `Who has 192.168.1.21? Tell 192.168.1.20`。
3. 对照稳定工程：

```text
C:\Users\myw29\Desktop\lwip\lwip_tcp\lwip_demo
```

4. 将当前工程的 lwIP 内存池、TCP 窗口、pbuf、heap 地址、内存对齐等配置向稳定工程靠齐。
5. 增加以太网收发计数 RTT 日志，确认底层 Rx/Tx 计数确实在增长。
6. 网络链路稳定后，关闭 `eth dbg ...` 周期日志，避免影响观察 TCP 协议日志。
7. 用 NetAssist 测试协议帧时发现一个容易误判的点：接收区选 HEX 不代表发送区也是 HEX。若发送区仍是 ASCII，则 `12 34 07 ...` 会按字符发送，不会被解析成真实字节 `0x12 0x34 0x07 ...`。

### 今天已经做过的关键修改

- `LWIP/Target/lwipopts.h`
  - 调整 `TCP_MSS`、`TCP_SND_BUF`、`TCP_WND`、`MEMP_NUM_TCP_SEG`、`MEM_SIZE`、`TCP_SND_QUEUELEN`、`PBUF_POOL_SIZE`、`PBUF_POOL_BUFSIZE`。
  - 将 `MEM_ALIGNMENT` 调整为 `32`。
  - 将 `LWIP_RAM_HEAP_POINTER` 调整到 `0x30020000`。
- `Core/Inc/stm32h7xx_hal_conf.h`
  - 增加 ETH TX/RX descriptor 数量。
- `LWIP/Target/ethernetif.c`
  - 增加 Rx/Tx cache 维护和调试计数。
  - 将 `ETH_RX_BUFFER_CNT` 调整为 `16`。
  - 当前 `ETH_DEBUG_RTT` 为 `0U`，底层以太网调试日志默认不输出。
- `Core/Src/adc_tcp_server.c`
  - 重构为更适合阅读的结构：协议定义、模块状态、函数声明、public API、lwIP 回调、连接管理、接收缓存、协议解析、命令分发。
  - 当前已经支持接收 TCP 字节流、缓存 pbuf 数据、释放 pbuf、按 `12 34 ... 56 78` 协议切帧、校验简单累加和、识别 `0x07/0x01/0x02` 命令。

### 当前协议理解

协议帧格式：

```text
SOF(2) + CMD(1) + LEN(2) + PAYLOAD(n) + SUM(2) + EOF(2)
```

示例心跳帧：

```text
12 34 07 00 00 00 4D 56 78
```

含义：

```text
12 34  -> 帧头
07     -> 心跳命令
00 00  -> payload 长度为 0
00 4D  -> 16 位累加和，高字节在前
56 78  -> 帧尾
```

校验范围：

```text
帧头 + 命令 + 长度 + payload
```

不包含：

```text
校验和字段自身
帧尾
```

### 明天继续的位置

明天不要直接上大功能，先继续带着理解 TCP 发送逻辑。

下一步要写的函数：

```c
static err_t adc_tcp_server_send_frame(struct tcp_pcb *tpcb,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payload_len);
```

发送函数的伪代码已经讲到这里：

```text
frame_len = payload_len + 9
frame[0] = 0x12
frame[1] = 0x34
frame[2] = cmd
frame[3] = payload_len 高字节
frame[4] = payload_len 低字节
payload 从 frame[5] 开始
checksum = frame[0] 到 payload 结束的简单累加和
checksumH 放在 frame[5 + payload_len]
checksumL 放在 frame[6 + payload_len]
EOF0 放在 frame[7 + payload_len]
EOF1 放在 frame[8 + payload_len]
最后 tcp_write() + tcp_output()
```

明天建议先实现最小闭环：

1. 新增 `adc_tcp_server_send_frame()`。
2. 收到 `0x07` 心跳后回复同样的心跳帧。
3. 用 NetAssist 的 HEX 发送模式发送：

```text
12 34 07 00 00 00 4D 56 78
```

4. 验证 NetAssist 能收到 MCU 回包。

### 学习进度记录

今天已经讲清楚并练习过：

- TCP 是字节流，不是一条 recv 等于一条命令。
- `pbuf` 要先复制到自己的缓存，再 `tcp_recved()`，最后 `pbuf_free()`。
- `s_rx_buf[2]` 是命令字。
- `payload_len = ((uint16_t)frame[3] << 8) | frame[4]`。
- `frame_len = payload_len + 9`。
- `checksum_len = 5 + payload_len`。
- `checksum` 是 16 位累加结果，例如 `0x004D` 拆成 `00 4D`。
- `memmove()` 用来在接收缓存中丢弃脏字节或删除已处理帧，因为源和目标可能重叠。

当前阶段还不是完整业务功能阶段，而是“硬件外设逐项打通 + 基础软件框架搭建”阶段。

已经确认：

- RTT 和 EasyLogger 可以正常输出。
- 以太网 PHY 链路可以起来，`netif_is_link_up()` 和 `netif_is_up()` 都能为 1。
- 在未启用当前 TCP 测试代码时，电脑可以 ping 通 `192.168.1.21`。
- ADC 采集服务可以读取 12 路 DMA 数据，并通过 RTT 打印采样值。
- ADC 数据帧构建器已经能按 `channel_mask` 打包成固定帧。
- DAC SPI 波形已经用逻辑分析仪确认过基本可解析，当前四通道测试帧能被 Saleae 按 16 bit 解析。

当前阻塞点：

- 只要 `adc_tcp_server_init()` 中执行 `tcp_new()`，即使不 `tcp_bind()`、不 `tcp_listen()`、不 `tcp_close()`，ping/ARP 都会异常。
- 这个现象说明问题大概率不在 TCP accept/recv 业务代码，而在 LwIP 内存池、堆位置、对齐、Cache/MPU 或 CubeMX 默认 LwIP 配置上。

## 3. 已完成修改

### RTT / EasyLogger

- 已移植 SEGGER RTT。
- 已移植 EasyLogger。
- 已确认 Keil 勾选 MicroLIB 后 RTT 输出正常。
- 已有 `app_log_init()`、`app_log_key_event()` 基础接口。
- 当前启动日志可看到：

```text
I/eLog EasyLogger V2.2.99 is initialize success.
I/BOOT logger started
I/BOOT event=1 logger started
I/BOOT event=2 peripherals init done
```

### 以太网 LAN8720 兼容处理

实际 PHY 是 LAN8720，但 CubeMX 生成的是 LAN8742 组件。

在 `LWIP/Target/ethernetif.c` 的链路检测逻辑中增加过兼容处理：

- 如果 `LAN8742_GetLinkState()` 返回 `LAN8742_STATUS_AUTONEGO_NOTDONE`。
- 再直接读取 PHY 的 BSR 寄存器两次。
- 如果第二次 BSR 表示 link up，则强制认为是 `100M full duplex`。

这个修改解决了 PHY 链路状态识别问题。

### DCache / Ethernet 基础处理

当前 `main.c` 中已开启：

```c
SCB_EnableICache();
SCB_EnableDCache();
```

`ethernetif.c` 里已经有 Tx clean、Rx invalidate 的 Cache 维护函数。

当前 MPU 只配置了 `0x30000000` 附近的 ETH 描述符 1KB 区域，不一定覆盖 LwIP heap / RX pool。

### DAC SPI

当前 DAC 芯片目标：TPC112S4，四通道 12 bit DAC。

当前 SPI1 关键配置：

```c
hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
```

当前 DAC 引脚：

```text
PD7  -> SPI1_MOSI
PG11 -> SPI1_SCK
PB12 -> DAC_SYNC
PB13 -> DAC_LOAD
```

当前测试帧：

```text
CH A: 0x8000
CH B: 0xA555
CH C: 0xCAAA
CH D: 0xEFFF
```

Saleae 逻辑分析仪在 16 bit、MSB first、Enable active low 下可以解析出这些帧。

当前四帧加一次 LOAD 大约 15.8 us。SPI 优化先作为 TODO，不继续在当前阶段纠缠。

### ADC 采集

当前 ADC 规划：

```text
ADC1: 4 个转换
ADC2: 1 个转换
ADC3: 7 个转换
合计: 12 个采集点
```

全部使用 TIM6 TRGO 触发，DMA Circular 模式。

当前 ADC 数据整理顺序：

```text
raw[0..3]   来自 ADC1 rank1..rank4
raw[4]      来自 ADC2 rank1
raw[5..11]  来自 ADC3 rank1..rank7
```

当前已经能通过 RTT 打印：

```text
adc seq=0 count=12 ch=...
```

注意：测试时 ADC 引脚是悬空的，所以数值跳动是正常现象，不能用于判断精度。

### ADC 帧格式

新增了 ADC 数据帧构建模块。

帧头当前包含：

```c
uint32_t magic;
uint32_t seq;
uint32_t timestamp_us;
uint16_t channel_mask;
uint16_t channel_count;
uint16_t sample_format;
uint16_t payload_bytes;
```

当前 `magic = 0x31434441`，按小端发送时内存字节顺序是 `41 44 43 31`，ASCII 看起来是 `ADC1`。

当前测试：

```text
frame len=68 magic=0x31434441 count=12 payload=48 seq=...
```

计算关系：

```text
header = 20 bytes
payload = channel_count * sizeof(float)
12 路 float = 12 * 4 = 48 bytes
总长度 = 20 + 48 = 68 bytes
```

## 4. 关键文件

### 主程序

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Src\main.c
```

当前 main 里 ADC、TCP、DAC 测试代码大多是注释状态。当前主要用于确认 LwIP 主循环和网络状态。

重点调用：

```c
MX_LWIP_Process();
```

这是裸机 LwIP 必须在主循环持续调用的处理函数。

### LwIP 配置

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\LWIP\Target\lwipopts.h
```

当前风险最大。当前仍是 CubeMX 默认风格：

```c
#define MEM_ALIGNMENT 4
#define LWIP_RAM_HEAP_POINTER 0x30044000
```

和之前稳定的 `lwip_demo` 工程不同。已知稳定工程中用过：

```c
#define TCP_SND_BUF             (20 * TCP_MSS)
#define TCP_WND                 (32 * TCP_MSS)
#define MEMP_NUM_TCP_SEG        96
#define MEM_SIZE                (64 * 1024)
#define PBUF_POOL_SIZE          40
#define MEM_ALIGNMENT           32
#define LWIP_RAM_HEAP_POINTER   0x30020000
```

下一步优先考虑把当前工程 LwIP 内存配置向这个稳定基线靠齐。

### 以太网底层

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\LWIP\Target\ethernetif.c
```

包含：

- LAN8720 兼容链路判断。
- ETH Rx/Tx Cache 维护。
- `ETH_RX_BUFFER_CNT` 当前为 `12U`，稳定工程曾使用 `16U`。

### TCP 服务

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Inc\adc_tcp_server.h
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Src\adc_tcp_server.c
```

当前 `adc_tcp_server_init()` 是诊断状态，只做 `tcp_new()`，没有完整开启 bind/listen。

当前结论：

- 不是 accept/recv 导致 ping 异常。
- `tcp_new()` 本身触发后，ARP/ping 就异常。

### ADC 采集服务

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Inc\adc_acq_service.h
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Src\adc_acq_service.c
```

负责：

- 启动 ADC DMA。
- 接收 half/full complete 回调标志。
- 从 ADC1/ADC2/ADC3 DMA buffer 中整理 12 路采样值。

### ADC 帧构建

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Inc\adc_frame_builder.h
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Src\adc_frame_builder.c
```

负责：

- 按 `channel_mask` 选择上传通道。
- 按固定帧头 + payload 的方式打包数据。
- 当前 payload 是 float，每通道 4 字节。

### DAC 驱动

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Inc\dac_tpc112s4.h
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Src\dac_tpc112s4.c
```

负责：

- 生成 TPC112S4 16 bit 控制帧。
- 控制 SYNC 和 LOAD。
- 当前使用 HAL_SPI_Transmit，后续可优化为 LL、寄存器或 DMA。

### SPI 配置

```text
C:\Users\myw29\Desktop\ADDA_collect\adc_collect\Core\Src\spi.c
```

CubeMX 会覆盖本文件非 USER CODE 区域，修改时要谨慎。

## 5. 不能动的边界

- 不要随便改 ADC 通道顺序。上位机解析依赖固定通道顺序。
- 不要随便改 ADC 帧头字段顺序和大小。上位机后续会按这个二进制协议解析。
- 不要把结构体裸发出去但不做 `#pragma pack(1)` 或显式 `memcpy`，否则编译器对齐可能造成协议错位。
- 不要随便关闭 `MX_LWIP_Process()`，裸机 LwIP 必须持续调用。
- 不要把耗时打印放进 ADC DMA 中断回调。中断里只做标志位更新。
- 不要用 `HAL_Delay()` 做 DAC 的微秒级 SYNC/LOAD 时序。
- 不要在 `tcp_err()` 回调里继续操作或关闭同一个 PCB。进入 err 回调时 PCB 已经由 LwIP 管理释放流程。
- 不要把 ADC 高速采集、TCP 大量发送、日志刷屏同时打开做定位测试。先单项验证。
- CubeMX 重新生成后，要重点检查：
  - `main.c` USER CODE 是否保留。
  - `ethernetif.c` LAN8720 兼容修改是否还在。
  - `spi.c` SPI 极性、相位、分频、SCK pull-down 是否还在。
  - Keil 工程是否还包含新增的 `.c` 文件。

## 6. 已经否掉的方案

- 不继续用 ADC 单通道验证项目架构，当前项目已经转为 3 个 ADC 共 12 路。
- DAC 当前不继续追求极限 SPI 性能，先保留 HAL_SPI_Transmit 版本，把 SPI 优化列为 TODO。
- 不再用 `HAL_Delay()` 做 DAC LOAD/SYNC 微小延时。
- 不把当前 `tcp_new()` 异常归因到 accept/recv，因为只执行 `tcp_new()` 也会触发 ping/ARP 异常。
- 不在当前阶段做 EEPROM 标定参数完整协议，先打通 ADC 帧和 TCP 数据链路。
- 不在当前阶段做 UDP 设备发现和网络配置，等 TCP/LwIP 基础稳定后再移植之前验证工程的成熟逻辑。

## 7. 当前风险点

### 最高优先级：tcp_new 后 ping/ARP 异常

当前现象：

- 注释掉 `adc_tcp_server_init()` 时，ping `192.168.1.21` 正常。
- 只执行 `tcp_new()` 后，ping/ARP 异常。
- `netif_is_link_up()` 和 `netif_is_up()` 仍为 1。

判断：

- PHY 链路不等于 IP 层正常。
- Windows ARP 表里没有 `192.168.1.21`，说明 ARP 请求/响应路径有问题。
- `tcp_new()` 会申请 TCP PCB 内存，如果申请后 ARP 都异常，重点怀疑 LwIP 内存池、heap 地址、内存对齐、Cache/MPU 配置。

建议下一步优先排查：

1. 对比稳定工程 `C:\Users\myw29\Desktop\lwip\lwip_tcp\lwip_demo` 的 `lwipopts.h`。
2. 把当前工程的 LwIP 内存参数先改到稳定基线。
3. 重新编译后，仍然只保留 `tcp_new()` 诊断版本，再测 ping。
4. 如果 ping 恢复，再逐步恢复 `tcp_bind()`、`tcp_listen()`、`tcp_accept()`。
5. 如果仍失败，再加 RX 计数和 ARP 包计数，确认 ARP 包是否进到 `ethernetif_input()`。

### CubeMX 覆盖风险

当前工程还在频繁用 CubeMX 生成代码，生成后容易覆盖：

- `main.c` 测试代码。
- `spi.c` SPI 参数。
- `ethernetif.c` LAN8720 兼容处理。
- Keil 工程文件新增源文件列表。

### 编码风险

部分中文注释已经出现乱码，后续建议：

- Keil 工程和源码统一 UTF-8 或 GBK，不要混用。
- 如果 Keil 中文注释显示不稳定，关键注释可先用简短英文或 ASCII 拼音，避免源码注释损坏。

### ADC 同步风险

当前 ADC1/ADC2/ADC3 由 TIM6 TRGO 同步触发，但三路 ADC 的转换序列长度不同：

```text
ADC1: 4 ranks
ADC2: 1 rank
ADC3: 7 ranks
```

这意味着“一次触发”后，每个 ADC 内部 rank 是按顺序转换的，不是 12 路完全同一瞬间采样。后续需要在文档和协议里明确“采样点顺序”和“触发批次”的定义。

## 8. 已经跑过的测试

### RTT / EasyLogger 测试

结果：通过。

现象：

```text
I/eLog EasyLogger V2.2.99 is initialize success.
I/BOOT logger started
```

### 以太网链路测试

结果：链路识别通过。

现象：

```text
net ip=192.168.1.21 link=1 up=1
```

### ping 测试

结果：条件通过。

通过条件：

- 不调用当前 TCP 测试初始化时，电脑 `192.168.1.20` 可以 ping 通单片机 `192.168.1.21`。

失败条件：

- 一旦执行 `tcp_new()`，ping/ARP 异常。

### DAC SPI 逻辑分析仪测试

结果：基本通过。

测试帧：

```text
0x8000
0xA555
0xCAAA
0xEFFF
```

结论：

- SPI 16 bit、MSB first 可解析。
- 四帧加一次 LOAD 当前约 15.8 us。
- SPI 分频 4 可以稳定解析。

### ADC 数据读取测试

结果：通过基础打印。

现象：

```text
adc seq=0 count=12 ch=...
adc seq=1 count=12 ch=...
```

注意：输入悬空，数据不代表真实测量性能。

### ADC 帧构建测试

结果：通过。

现象：

```text
frame len=68 magic=0x31434441 count=12 payload=48 seq=...
```

## 9. 下一步计划

### 第一步：修复 tcp_new 后 ping/ARP 异常

先不要继续写 ADC TCP 发送业务，先让下面这个最小测试成立：

```text
MX_LWIP_Init()
adc_tcp_server_init() 只执行 tcp_new()
主循环持续 MX_LWIP_Process()
ping 192.168.1.21 正常
```

建议优先修改 `LWIP/Target/lwipopts.h`，向稳定工程靠齐：

```c
#define TCP_SND_BUF             (20 * TCP_MSS)
#define TCP_WND                 (32 * TCP_MSS)
#define MEMP_NUM_TCP_SEG        96
#define MEM_SIZE                (64 * 1024)
#define PBUF_POOL_SIZE          40
#define MEM_ALIGNMENT           32
#define LWIP_RAM_HEAP_POINTER   0x30020000
```

也可以把 `ethernetif.c` 的：

```c
#define ETH_RX_BUFFER_CNT 12U
```

临时改成：

```c
#define ETH_RX_BUFFER_CNT 16U
```

用于对齐之前稳定工程的配置。

### 第二步：恢复完整 TCP listen

按顺序恢复：

1. `tcp_new()`
2. `tcp_bind()`
3. `tcp_listen()`
4. `tcp_accept()`
5. 客户端连接
6. `PING -> OK,PING`

每恢复一步都测一次 ping，避免一次恢复太多导致定位困难。

### 第三步：接入 ADC 帧发送

在 `adc_tcp_server_process()` 里做：

1. 判断是否有客户端。
2. 判断是否开启 ADC stream。
3. 从 `adc_acq_service_get_sample()` 取样。
4. 用 `adc_frame_builder_build_raw_float()` 打包。
5. 判断 `tcp_sndbuf()` 是否足够。
6. `tcp_write(..., TCP_WRITE_FLAG_COPY)`。
7. `tcp_output()`。

先低频发送，确认上位机能解析，再逐步提高速率。

### 第四步：恢复网络管理命令

等 TCP 稳定后再做：

- GET_NET
- SET_NET
- SET_MAC
- SET_NAME
- ADC_ON / ADC_OFF
- DAC_SET
- EEPROM 标定参数读写

### 第五步：EEPROM 标定参数

后续要定义 EEPROM 参数布局：

- ADC 每通道 `k_ad`、`b_ad`
- DAC 每通道 `k_da`、`b_da`
- 参数版本号
- CRC
- 默认参数区
- 生效参数区

### 第六步：ADC 到 DAC 内部级联

公式：

```text
analog = adc_raw * k_ad + b_ad
dac_value = analog * k_da + b_da
```

再把 `dac_value` 限幅并换算成 TPC112S4 12 bit code。

## 10. 新窗口启动提示词

可以把下面这段直接发给新窗口：

```text
我们继续 C:\Users\myw29\Desktop\ADDA_collect\adc_collect 这个 STM32H743 项目。

请先阅读项目根目录的 handoff.md，然后按里面的当前风险点继续指导我。

当前最重要的问题是：不调用 adc_tcp_server_init() 时，单片机 192.168.1.21 可以 ping 通；但是 adc_tcp_server_init() 里只要执行 tcp_new()，即使不 bind/listen/close，ping 和 ARP 就异常。netif 显示 link=1 up=1。

请你继续按照之前的方式指导我：先解释为什么这么做，再给我具体要改的代码位置和代码片段，最后问我关键技术问题确认我真正掌握。写代码主要由我自己来，你负责一步步带我排查和审查。

优先任务：对比之前稳定工程 C:\Users\myw29\Desktop\lwip\lwip_tcp\lwip_demo 的 lwipopts.h / ethernetif.c，把当前工程 LwIP 内存、对齐、heap、pbuf、TCP pool 配置修到 tcp_new 后仍能 ping 通。
```
