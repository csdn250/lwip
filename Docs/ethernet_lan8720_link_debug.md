# LAN8720 以太网链路调试记录

## 背景

当前项目使用 STM32H743IITx + LAN8720 PHY，CubeMX 默认生成的 LwIP PHY 组件是 LAN8742。两者基础 MDIO 标准寄存器兼容度较高，但扩展状态寄存器并不完全等价。

## 现象

程序初始化后，LwIP 打印：

```text
lwip ip=192.168.1.21 link=0 up=0
```

电脑端网卡显示已连接，速率为 100.0 Mbps，但单片机侧 `netif_is_link_up()` 和 `netif_is_up()` 一直为 0。

## 排查证据

读取 PHY 地址：

```text
phy addr=0
```

说明 STM32 可以通过 MDC/MDIO 正常访问 PHY，PHY 管理接口可用。

读取 PHY 标准状态寄存器 BSR：

```text
phy bsr1=0x796D bsr2=0x796D link=0 up=0
```

`BSR` 是 IEEE 标准寄存器：

```text
bit2 = Link Status
bit5 = Auto Negotiation Complete
```

`0x796D` 中 bit2 和 bit5 均为 1，说明：

```text
PHY 物理链路已建立
自动协商已完成
```

继续打印 LAN8742 驱动返回值：

```text
state=6 bcr=0x1140 bsr=0x796D physcsr=0x067F link=0 up=0
```

其中：

```text
state=6 = LAN8742_STATUS_AUTONEGO_NOTDONE
```

这与 BSR 标准寄存器的自动协商完成状态矛盾。

## 根因判断

LAN8720 的基础标准寄存器可以正常读取，且物理链路已经建立。但 CubeMX 生成的 LAN8742 驱动在 `LAN8742_GetLinkState()` 中会继续读取 `PHYSCSR` 扩展状态寄存器，并按照 LAN8742 的位定义判断自动协商和速率双工。

LAN8720 与 LAN8742 的扩展状态寄存器位定义不完全一致，导致驱动误判为：

```text
LAN8742_STATUS_AUTONEGO_NOTDONE
```

于是 `ethernet_link_check_state()` 不会执行：

```c
HAL_ETH_Start(&heth);
netif_set_up(netif);
netif_set_link_up(netif);
```

最终表现为物理链路已建立，但 LwIP 仍显示：

```text
link=0 up=0
```

## 当前验证处理

在 `ethernet_link_check_state()` 中增加 LAN8720 兼容处理：

```c
PHYLinkState = LAN8742_GetLinkState(&LAN8742);

if (LAN8742_STATUS_AUTONEGO_NOTDONE == PHYLinkState)
{
    uint32_t bsr1 = 0;
    uint32_t bsr2 = 0;

    if ((ETH_PHY_IO_ReadReg(LAN8742.DevAddr, LAN8742_BSR, &bsr1) >= 0) &&
        (ETH_PHY_IO_ReadReg(LAN8742.DevAddr, LAN8742_BSR, &bsr2) >= 0))
    {
        if (0U != (bsr2 & LAN8742_BSR_LINK_STATUS))
        {
            PHYLinkState = LAN8742_STATUS_100MBITS_FULLDUPLEX;
        }
    }
}
```

处理后 RTT 打印：

```text
lwip ip=192.168.1.21 link=1 up=1
state=6 bcr=0x1140 bsr=0x796D physcsr=0x067F link=1 up=1
phy bsr1=0x796D bsr2=0x796D link=1 up=1
```

说明 LwIP 网络接口已经被正确拉起。

## 后续正式化建议

当前处理适合作为验证补丁。产品交付阶段建议改成正式 LAN8720 PHY 适配层：

1. 新增或重命名 PHY 驱动为 `lan8720`，避免代码命名与实际硬件不一致。
2. 链路判断优先使用标准寄存器 BSR 的 link/autoneg 位。
3. 速率和双工状态按 LAN8720 手册中的专用状态寄存器解析。
4. 启动日志保留 PHY 地址、BSR、最终 link/up 状态，方便现场定位。
5. 临时调试打印在交付版本中降级或关闭，避免 RTT/日志刷屏。

## 关键结论

```text
phy addr=0 只能证明 MDIO/MDC 管理接口通。
BSR bit2=1 才证明 PHY 物理链路已建立。
link=1 up=1 才证明 LwIP netif 已经可以参与协议栈收发。
```

## 后续现象：link/up 已经为 1，但 ping 仍不通

Windows ping 显示：

```text
来自 192.168.1.2 的回复: 无法访问目标主机。
```

这不是单片机回复，而是电脑本机报告 ARP 无法解析目标主机。此时说明 PHY link 已经不是第一嫌疑，下一步应检查 MAC/DMA 收发链路。

当前新工程已开启 D-Cache，ETH DMA 描述符放在：

```text
DMARxDscrTab: 0x30000000
DMATxDscrTab: 0x30000200
```

但 MPU 目前只配置了默认 4GB 区域，没有像旧验证工程一样，把 `0x30000000` 附近的 ETH DMA 描述符区域单独设置为 non-cacheable。STM32H7 的 ETH DMA 与 Cortex-M7 D-Cache 不一致时，可能导致 PHY link 正常但 ARP/ping 收发失败。

下一步优先验证：

1. 临时关闭 D-Cache，如果 ping 恢复，说明是 cache/MPU 问题。
2. 正式修复是在 `MPU_Config()` 中为 `0x30000000` 增加 non-cacheable MPU region。

## 最终连通性根因：ETH GPIO 速度过低

继续对比旧验证工程后发现，新工程 CubeMX 生成的 ETH GPIO 初始化为：

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
```

旧工程中可正常通信的 ETH GPIO 初始化为：

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
```

RMII 使用 50 MHz REF_CLK，TX_EN、TXD0、TXD1 等信号边沿速度不足时，可能出现 PHY link 已经建立、LwIP netif 也已 up，但实际 ARP/ICMP 无法正常收发的现象。

本次在 `HAL_ETH_MspInit()` 中将 ETH 相关 GPIO 全部改为 Very High：

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

修改后电脑端指定网卡 ping：

```text
ping -S 192.168.1.20 192.168.1.21
```

结果：

```text
来自 192.168.1.21 的回复: 字节=32 时间<1ms TTL=255
```

说明 ARP 和 ICMP 已经正常，STM32 以太网链路打通。

## 当前必须保留的两个修复点

1. LAN8720 兼容处理：避免 LAN8742 驱动误判 `AUTONEGO_NOTDONE`。
2. ETH GPIO speed 必须为 `GPIO_SPEED_FREQ_VERY_HIGH`。

后续如果重新用 CubeMX 生成代码，必须检查这两个点是否被覆盖。
