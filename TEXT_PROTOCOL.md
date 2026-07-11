# Host_Computer 文本串口协议 v0.2

串口参数默认为 `115200, 8N1`，每条命令以 `\r\n`、`\n` 或 `\r` 结束。命令使用 ASCII 文本，不区分大小写；首尾空格、Tab 和参数之间的连续空格会被自动整理。

## 控制与设置

| 命令 | 示例 | 成功响应 |
| --- | --- | --- |
| 启动 | `START` | `OK` |
| 停止 | `STOP` | `OK` |
| 复位状态 | `RESET` | `OK` |
| 设置速度 | `SET_SPEED 20.5` | `OK` |
| 设置位置 | `SET_POS 2000` | `OK` |
| 设置位置别名 | `SET_POSITION 2000` | `OK` |
| 设置 PID | `SET_PID 3.0 0.2 0.1` | `OK` |
| 设置模式 | `SET_MODE SPEED` | `OK` |

模式可选：`MANUAL`、`AUTO`、`AVOID`、`SPEED`、`POSITION`。

## 查询

| 命令 | 响应示例 |
| --- | --- |
| `PING` | `PONG` |
| `STATUS` / `GET_STATUS` | `STATUS MODE=SPEED RUN=12345` |
| `GET_SPEED` | `SPEED L=18.50 R=18.20` |
| `GET_POS` / `GET_POSITION` | `POS L=1024 R=1018` |
| `GET_PID` | `PID KP=3.000 KI=0.200 KD=0.100` |
| `GET_TARGET` | `TARGET SPEED=20.500 POS=2000` |

`GET_SPEED` 和 `GET_POS` 通过弱函数 `App_GetSpeed()`、`App_GetPosition()` 读取真实数据。项目未实现这两个接口时默认返回 0。

## 错误响应

| 响应 | 含义 |
| --- | --- |
| `ERR UNKNOWN_CMD` | 命令不存在 |
| `ERR BAD_FORMAT` | 无参数命令后出现了多余内容 |
| `ERR BAD_VALUE` | 数字缺失、溢出或包含 NaN/Inf |
| `ERR BAD_MODE` | 模式名称不合法 |
| `ERR LINE_TOO_LONG` | 一行超过 95 字节 |
| `ERR RX_QUEUE_FULL` | 主循环处理过慢，4 条命令队列已满 |
| `ERR INVALID_CHAR` | 文本命令包含非 ASCII 控制字符 |
| `ERR UART_*` | 串口发生溢出、帧、噪声或校验错误 |

超长行或非法字符出现后，接收器会丢弃到下一个换行符再恢复，避免把错误行的尾部当成新命令执行。文本终端输入时支持 Backspace 和 Delete 删除字符。
