#ifndef HOST_COMPUTER_BINARY_PROTOCOL_H
#define HOST_COMPUTER_BINARY_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BINARY_PROTOCOL_HEADER_1 0xAAU
#define BINARY_PROTOCOL_HEADER_2 0x55U
#define BINARY_PROTOCOL_DEVICE_ADDRESS 0x01U
#define BINARY_PROTOCOL_BROADCAST_ADDRESS 0xFFU
#define BINARY_PROTOCOL_MAX_PAYLOAD 64U

typedef enum
{
    BINARY_CMD_START = 0x01,
    BINARY_CMD_STOP = 0x02,
    BINARY_CMD_RESET = 0x03,
    BINARY_CMD_GET_STATUS = 0x04,
    BINARY_CMD_SET_SPEED = 0x10,
    BINARY_CMD_SET_POSITION = 0x11,
    BINARY_CMD_SET_PID = 0x12,
    BINARY_CMD_SET_MODE = 0x13,
    BINARY_CMD_GET_SPEED = 0x20,
    BINARY_CMD_GET_POSITION = 0x21,
    BINARY_CMD_SET_SERVO_POSITION = 0x22,

    BINARY_RSP_ACK = 0x80,
    BINARY_RSP_ERROR = 0x81,
    BINARY_RSP_STATUS = 0x84,
    BINARY_RSP_SPEED = 0xA0,
    BINARY_RSP_POSITION = 0xA1
} BinaryProtocolCommand;

typedef enum
{
    BINARY_ERROR_NONE = 0x00,
    BINARY_ERROR_BAD_LENGTH = 0x01,
    BINARY_ERROR_BAD_CRC = 0x02,
    BINARY_ERROR_UNKNOWN_COMMAND = 0x03,
    BINARY_ERROR_BAD_VALUE = 0x04,
    BINARY_ERROR_QUEUE_FULL = 0x05
} BinaryProtocolError;

void BinaryProtocol_Init(UART_HandleTypeDef *huart);
void BinaryProtocol_Task(void);
void BinaryProtocol_ResetReceiver(void);

/* Returns 1 when the byte belongs to a binary frame, otherwise 0. */
uint8_t BinaryProtocol_InputByte(uint8_t byte);

uint16_t BinaryProtocol_CRC16(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef BinaryProtocol_SendFrame(uint8_t command,
                                           const uint8_t *payload,
                                           uint16_t payload_length);
void BinaryProtocol_SendSpeed(float left_speed, float right_speed);
void BinaryProtocol_SendPosition(int32_t left_position, int32_t right_position);

#ifdef __cplusplus
}
#endif

#endif
