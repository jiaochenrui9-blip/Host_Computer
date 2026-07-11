# Host_Computer 二进制串口协议 v0.1

## 帧格式

所有多字节整数和 `float` 均采用小端序。

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| 帧头 | 2 字节 | 固定为 `AA 55` |
| 设备地址 | 1 字节 | 当前设备 `01`，广播地址 `FF` |
| 命令 | 1 字节 | 见命令表 |
| 数据长度 | 2 字节 | 数据区字节数，小端序，最大 64 |
| 数据 | N 字节 | 命令参数 |
| CRC16 | 2 字节 | CRC-16/MODBUS，小端序 |

CRC 的计算范围从“设备地址”开始，到“数据区最后一个字节”结束，不包含帧头和 CRC 本身。CRC 初值为 `0xFFFF`，多项式为 `0xA001`。

## 请求命令

| 命令 | 编号 | 数据内容 |
| --- | ---: | --- |
| START | `01` | 无 |
| STOP | `02` | 无 |
| RESET | `03` | 无 |
| GET_STATUS | `04` | 无 |
| SET_SPEED | `10` | `float32 speed` |
| SET_POSITION | `11` | `int32 position` |
| SET_PID | `12` | `float32 kp, ki, kd` |
| SET_MODE | `13` | `uint8 mode`，0~4 |
| GET_SPEED | `20` | 无 |
| GET_POSITION | `21` | 无 |

模式编号：`0=MANUAL`、`1=AUTO`、`2=AVOID`、`3=SPEED`、`4=POSITION`。

## 响应命令

| 命令 | 编号 | 数据内容 |
| --- | ---: | --- |
| ACK | `80` | 原请求命令 1 字节 + `00` |
| ERROR | `81` | 原请求命令 1 字节 + 错误码 1 字节 |
| STATUS | `84` | running 1 字节 + mode 1 字节 + run_ms 4 字节 |
| SPEED | `A0` | left float32 + right float32 |
| POSITION | `A1` | left int32 + right int32 |

错误码：`01=长度错误`、`02=CRC错误`、`03=未知命令`、`04=参数错误`、`05=接收队列已满`。

广播地址 `FF` 可以执行控制和设置命令，但设备不会回复，避免多设备同时回包冲突。

## HEX 测试帧

在串口助手的 HEX 发送模式下可以直接测试：

```text
START 请求:  AA 55 01 01 00 00 50 18
START ACK:   AA 55 01 80 02 00 01 00 00 3C
STATUS 请求: AA 55 01 04 00 00 40 19
```

发送 HEX 帧时不要自动追加 `\r\n`。

## 接收方式

当前实现使用 UART 单字节接收中断和 4 帧接收队列，不使用 DMA。中断只完成组帧、CRC 校验和入队，命令在 `BinaryProtocol_Task()` 中处理。连续两个字节为 `AA 55` 时进入二进制解析，其余数据继续交给原文本协议。
