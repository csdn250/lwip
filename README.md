# adc_collect

STM32H743 + lwIP based AD/DA data collection firmware.

本项目当前重点是以太网通讯链路：设备启动后使用 lwIP 提供 TCP 服务，同时在未建立 TCP 连接时周期性发送 UDP 发现广播，方便上位机识别设备并连接配置。

## 当前状态

- TCP Server 端口：`8080`
- UDP 发现广播端口：`8081`
- 默认设备 IP：`192.168.1.21`
- 默认上位机 IP：`192.168.1.20`
- 默认 MAC：`02:00:00:00:00:21`
- 默认设备名：`ADDA_COLLECT_1`
- EEPROM 参数存储框架已预留，目标器件为 `24C64`

## 协议帧格式

TCP 命令和 UDP 发现广播当前使用统一帧结构：

```text
12 34 CMD LEN_H LEN_L PAYLOAD CRC32_3 CRC32_2 CRC32_1 CRC32_0 56 78
```

字段说明：

- `12 34`：帧头
- `CMD`：命令字
- `LEN_H LEN_L`：payload 长度，高字节在前
- `PAYLOAD`：数据区
- `CRC32`：对 `帧头 + CMD + 长度 + PAYLOAD` 计算 CRC32/IEEE，发送时高字节在前
- `56 78`：帧尾

已实现命令：

| 命令 | 方向 | 说明 |
| --- | --- | --- |
| `0x01` | TCP | 写参数块 |
| `0x02` | TCP | 读参数块 |
| `0x07` | TCP | 心跳 |
| `0x71` | UDP | 设备发现/状态广播 |
| `0x81` | TCP | 原始采集数据上报预留 |
| `0x82` | TCP | 换算后数据上报预留 |

## 参数块

当前已按上位机协议预留参数块：

| Block ID | 说明 |
| --- | --- |
| `0x0001` | AD/DA 标定参数 |
| `0x0002` | 控制参数 |
| `0x0003` | 配置参数 |
| `0x0004` | 设备 IP / 上位机 IP |
| `0x0005` | 设备 MAC |
| `0x0006` | 设备端口 / 上位机端口 |
| `0x0007` | 子网掩码 |
| `0x0008` | 网关 |
| `0x0009` - `0x000C` | DA 通道参数 |

注意：后续还需要把 TCP 写入的网络参数同步到 `device_config`，并进一步决定是否立即重启网卡配置或要求设备重启后生效。

## UDP 发现广播

当没有 TCP 客户端连接时，设备每 5 秒向 `255.255.255.255:8081` 发送一次发现广播。建立 TCP 连接后广播停止。

UDP payload 当前内容：

```text
"ADDA" + version + mac + ip + netmask + gateway + tcp_port + device_name
```

Wireshark 可使用过滤器查看：

```text
udp.port == 8081
```

## 关键源码

- `Core/Src/adc_tcp_server.c`：TCP 监听、接收缓存、拆帧、CRC32 校验、命令分发、响应发送
- `Core/Src/udp_discovery.c`：UDP 发现广播
- `Core/Src/device_config.c`：设备网络配置默认值
- `Core/Src/eeprom_storage.c`：24C64 EEPROM 读写框架
- `tools/make_tcp_frame.py`：生成 NetAssist 测试用 HEX 协议帧

## 测试辅助

生成心跳帧：

```powershell
python tools\make_tcp_frame.py 07
```

生成读取 MAC 参数块帧：

```powershell
python tools\make_tcp_frame.py 02 00 05
```

使用指定源 IP 测试 ping：

```powershell
ping -S 192.168.1.20 192.168.1.21
```

## 构建

工程使用 Keil MDK-ARM 构建：

```text
MDK-ARM/adc_collect.uvprojx
```

当前代码已在测试板上验证：

- `ping -S 192.168.1.20 192.168.1.21` 正常
- TCP 心跳、读参数、写参数基础流程正常
- CRC32 校验流程正常
- UDP 发现广播可被 Wireshark 抓到

## 后续计划

- TCP 写入 IP/MAC/子网掩码/网关/端口后，同步更新 `device_config`
- 明确网络参数修改后是立即生效还是重启后生效
- 将可靠参数区落到 24C64 EEPROM
- 增加结构化现场快照日志，用于问题追踪
- 接入 `0x81` 原始数据和 `0x82` 换算数据上报
