# Host_Computer

STM32F103 上位机通信协议工程，支持文本协议与二进制协议共用 USART1。

## 协议

- [文本协议 v0.2](TEXT_PROTOCOL.md)
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

协议命令最终通过 `App_*` 弱函数连接到电机、PID、舵机或传感器等实际业务代码。
