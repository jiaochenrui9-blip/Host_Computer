# ReadyForApp

STM32F103 通用二进制协议层工程，使用 USART1，可直接在其上添加应用层。

## 协议

- [二进制协议 v0.1](BINARY_PROTOCOL.md)

二进制帧格式：

```text
帧头(AA 55) | 设备地址 | 命令 | 数据长度(uint16 LE) | Payload | CRC16/MODBUS
```

当前默认串口参数为 `115200, 8N1`。接收使用 UART 单字节中断和帧队列，不使用 DMA。

## 构建

使用项目已有的 STM32 CMake 预设，或在 CLion 中选择 STM32 Debug 配置。命令行验证：

```powershell
cmake --build cmake-build-debug-stm32 --target Host_Computer -j 8
```

当前工程不包含电机、PID、舵机、状态机或具体命令定义；这些内容由后续应用层自行接入。
