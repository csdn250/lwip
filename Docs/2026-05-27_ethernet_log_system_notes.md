# 2026-05-27 以太网与日志系统调试记录

## 今日目标

1. 在新项目 `adc_collect` 中确认以太网基础链路可用。
2. 将 RTT + EasyLogger 日志系统重新搭建起来。
3. 为后续 ADC/DAC/EEPROM/RS485 调试建立可复用的问题定位方式。

## 一、RTT 与 EasyLogger 恢复

### 现象

最开始 RTT Viewer 找不到 RTT control block，窗口中只显示：

```text
Failed to find RTT control block in RAM
```

程序中即使调用 `SEGGER_RTT_WriteString()`，RTT Viewer 也没有显示。

### 处理

确认 Keil 工程启用了 MicroLIB 后，RTT 输出恢复正常。

当前日志链路：

```text
EasyLogger -> elog_port_output() -> SEGGER_RTT_Write()
```

当前已建立的应用日志接口：

```c
void app_log_init(void);
void app_log_key_event(app_log_event_t event, const char *message);
```

### 原理

`app_log_key_event()` 是关键事件统一入口。后续如果要把关键事件写入 Flash，只需要在这个函数内部扩展，不需要到各业务模块里逐个修改。

## 二、LwIP 主循环处理

### 现象

以太网初始化完成后，程序能进入主循环，但 ping 不通。

### 根因

裸机 LwIP 工程中，`MX_LWIP_Init()` 只负责初始化协议栈，不能反复调用，也不能代替运行时处理。

主循环必须持续调用：

```c
MX_LWIP_Process();
```

它负责：

```text
接收以太网数据包
处理 ARP/IP/ICMP/TCP/UDP
处理 lwIP 定时器
检查 PHY 链路状态
```

### 处理

在 `while (1)` 中保留：

```c
MX_LWIP_Process();
```

## 三、LAN8720 与 LAN8742 驱动兼容问题

### 现象

RTT 打印：

```text
phy addr=0
lwip ip=192.168.1.21 link=0 up=0
```

进一步读取 PHY 寄存器：

```text
state=6 bcr=0x1140 bsr=0x796D physcsr=0x067F link=0 up=0
```

### 证据

`phy addr=0` 说明 STM32 可以通过 MDC/MDIO 找到 PHY。

`bsr=0x796D` 中：

```text
BSR bit2 = 1，Link 已建立
BSR bit5 = 1，自动协商完成
```

但 `LAN8742_GetLinkState()` 返回：

```text
state=6 = LAN8742_STATUS_AUTONEGO_NOTDONE
```

这与标准 BSR 状态矛盾。

### 根因

实际硬件使用 LAN8720，但 CubeMX 默认生成 LAN8742 驱动。两者标准寄存器兼容，但扩展状态寄存器 `PHYSCSR` 位定义不完全一致。

LAN8742 驱动读取 LAN8720 扩展寄存器后误判自动协商未完成，导致：

```c
netif_set_up(netif);
netif_set_link_up(netif);
```

没有执行。

### 临时兼容处理

当 LAN8742 驱动返回 `AUTONEGO_NOTDONE`，但 BSR 已经确认 link 建立时，按 100M full duplex 先拉起 MAC 和 netif。

处理后 RTT 打印：

```text
lwip ip=192.168.1.21 link=1 up=1
```

## 四、ETH GPIO Speed 问题

### 现象

`link=1 up=1` 后，电脑仍然 ping 不通。

电脑端：

```text
来自 192.168.1.20 的回复: 无法访问目标主机
```

`arp -a` 中也没有 `192.168.1.21` 的动态 ARP 项。

### 对比发现

旧验证工程中 ETH GPIO 配置为：

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
```

新工程 CubeMX 生成的是：

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
```

### 根因

RMII 工作在 50 MHz 相关时序下，TX_EN、TXD0、TXD1 等信号如果 GPIO 输出速度过低，PHY 可能无法可靠识别 MAC 发出的数据。

### 处理

将 ETH 相关 GPIO 全部改为 `GPIO_SPEED_FREQ_VERY_HIGH`：

```text
PC1  ETH_MDC
PA1  ETH_REF_CLK
PA2  ETH_MDIO
PA7  ETH_CRS_DV
PC4  ETH_RXD0
PC5  ETH_RXD1
PB11 ETH_TX_EN
PG13 ETH_TXD0
PG14 ETH_TXD1
```

修改后，不开 D-Cache 时 ping 成功。

## 五、D-Cache 与 ETH DMA 问题

### 现象

重新开启 D-Cache 后，ping 再次失败：

```text
请求超时
```

### 根因

STM32H7 Cortex-M7 开启 D-Cache 后，CPU 和 ETH DMA 看到的内存可能不一致。

以太网涉及两类内存：

```text
1. ETH DMA 描述符
2. ETH 数据 buffer
```

描述符区在：

```text
DMARxDscrTab = 0x30000000
DMATxDscrTab = 0x30000200
```

如果描述符区被 D-Cache 缓存，CPU 和 DMA 对描述符状态的理解可能不一致。

发送数据时，如果 CPU 写了 ARP/ICMP 回复，但没有 clean D-Cache，ETH DMA 可能读到旧数据。

接收数据时，如果 DMA 写入了新包，但 CPU 没有 invalidate D-Cache，CPU 可能读到旧缓存。

### 正式处理

1. MPU 保留默认 4GB 区域。
2. 增加 `0x30000000` 的 non-cacheable MPU region。
3. 发送前对 pbuf payload 执行 cache clean。
4. 接收后对 DMA 写入的 buffer 执行 cache invalidate。
5. clean/invalidate 地址和长度按 32 字节 cache line 对齐。

最终 D-Cache 打开后 ping 成功。

## 六、最终验证结果

电脑端指定本地网卡：

```bat
ping -S 192.168.1.20 192.168.1.21
```

成功结果：

```text
来自 192.168.1.21 的回复: 字节=32 时间<1ms TTL=255
```

说明：

```text
PHY link 正常
LwIP netif up 正常
ARP 正常
ICMP 正常
D-Cache 打开后 ETH DMA 收发正常
```

## 七、必须长期保留的关键点

1. `while (1)` 中必须持续调用 `MX_LWIP_Process()`。
2. LAN8720 不能完全照搬 LAN8742 扩展状态寄存器解析。
3. ETH GPIO speed 必须是 `GPIO_SPEED_FREQ_VERY_HIGH`。
4. D-Cache 打开时，ETH DMA 描述符区必须 non-cacheable。
5. ETH TX 数据发送前要 clean D-Cache。
6. ETH RX 数据使用前要 invalidate D-Cache。
7. CubeMX 重新生成后，必须复查 ETH GPIO speed 和 LAN8720 兼容补丁是否被覆盖。

## 八、后续计划

1. 将 LAN8720 兼容逻辑整理成正式 PHY 适配层。
2. 清理 main.c 中临时调试代码。
3. 完善 `app_log_key_event()`，形成关键事件日志规范。
4. 后续增加 Flash 环形日志区，只存关键事件和严重错误。

