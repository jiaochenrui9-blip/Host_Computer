#include "Binary_Protocol.h"
#include "UART_Protocol.h"

#include <string.h>

typedef union
{
    uint32_t u32;
    float f32;
} BinaryFloat32;
//Protocol example : 帧头1 帧头2 地址 命令 长度低 长度高 数据... CRC低 CRC高
_Static_assert(sizeof(float) == 4, "Binary protocol requires 32-bit float");
_Static_assert(sizeof(uint32_t) == 4, "Binary protocol requires 32-bit uint32_t");
_Static_assert(sizeof(BinaryFloat32) == 4, "Binary protocol float union must be 4 bytes");

enum
{
    BINARY_FRAME_QUEUE_SIZE = 4,
    BINARY_INTER_BYTE_TIMEOUT_MS = 100
};

typedef struct
{
    uint8_t address;
    uint8_t command;
    uint16_t length;
    uint8_t payload[BINARY_PROTOCOL_MAX_PAYLOAD];
} BinaryFrame;

typedef enum
{
    RX_WAIT_HEADER_1 = 0,
    RX_WAIT_HEADER_2,
    RX_ADDRESS,
    RX_COMMAND,
    RX_LENGTH_LOW,
    RX_LENGTH_HIGH,
    RX_PAYLOAD,
    RX_CRC_LOW,
    RX_CRC_HIGH
} BinaryRxState;

static UART_HandleTypeDef *binary_uart = NULL;
static volatile BinaryRxState rx_state = RX_WAIT_HEADER_1;
static BinaryFrame rx_working_frame;
static BinaryFrame rx_frame_queue[BINARY_FRAME_QUEUE_SIZE];
static volatile uint16_t rx_payload_index = 0;
static volatile uint16_t rx_calculated_crc = 0xFFFFU;
static volatile uint16_t rx_received_crc = 0;
static volatile uint8_t rx_queue_read = 0;
static volatile uint8_t rx_queue_write = 0;
static volatile uint8_t rx_queue_count = 0;
static volatile uint32_t rx_last_byte_tick = 0;
static volatile uint8_t rx_error_pending = 0;
static volatile uint8_t rx_error_command = 0;
static volatile BinaryProtocolError rx_error_code = BINARY_ERROR_NONE;
//初始crc为0xFFFF，多项式为0xA001
static uint16_t BinaryProtocol_CRC16Update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= byte;
    for (bit = 0; bit < 8U; bit++)
    {
        if ((crc & 0x0001U) != 0U)
        {
            crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
        }
        else
        {
            crc >>= 1U;
        }
    }
    return crc;
}

static void BinaryProtocol_ResetParser(void)
{
    rx_state = RX_WAIT_HEADER_1;
    rx_payload_index = 0;
    rx_calculated_crc = 0xFFFFU;
    rx_received_crc = 0;
}

static uint8_t BinaryProtocol_IsLocalAddress(uint8_t address)
{
    return (address == BINARY_PROTOCOL_DEVICE_ADDRESS) ||
           (address == BINARY_PROTOCOL_BROADCAST_ADDRESS);
}

static void BinaryProtocol_SetPendingError(uint8_t command, BinaryProtocolError error)
{
    if (rx_error_pending == 0U)
    {
        rx_error_command = command;
        rx_error_code = error;
        rx_error_pending = 1U;
    }
}

static uint32_t BinaryProtocol_ReadU32LE(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static int32_t BinaryProtocol_ReadI32LE(const uint8_t *data)
{
    return (int32_t)BinaryProtocol_ReadU32LE(data);
}

static uint8_t BinaryProtocol_ReadFloatLE(const uint8_t *data, float *value)
{
    BinaryFloat32 converter;

    converter.u32 = BinaryProtocol_ReadU32LE(data);

    if ((converter.u32 & 0x7F800000U) == 0x7F800000U)
    {
        return 0;
    }

    *value = converter.f32;
    return 1;
}

static void BinaryProtocol_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void BinaryProtocol_WriteI32LE(uint8_t *data, int32_t value)
{
    BinaryProtocol_WriteU32LE(data, (uint32_t)value);
}

static void BinaryProtocol_WriteFloatLE(uint8_t *data, float value)
{
    BinaryFloat32 converter;

    converter.f32 = value;
    BinaryProtocol_WriteU32LE(data, converter.u32);
}

static void BinaryProtocol_SendAck(uint8_t request_command)
{
    uint8_t payload[2] = {request_command, BINARY_ERROR_NONE};
    BinaryProtocol_SendFrame(BINARY_RSP_ACK, payload, sizeof(payload));
}

static void BinaryProtocol_SendError(uint8_t request_command, BinaryProtocolError error)
{
    uint8_t payload[2] = {request_command, (uint8_t)error};
    BinaryProtocol_SendFrame(BINARY_RSP_ERROR, payload, sizeof(payload));
}

static void BinaryProtocol_SendStatus(void)
{
    const ProtocolState *state = Protocol_GetState();
    uint8_t payload[6];
    uint32_t run_time = 0;

    if (state->running != 0U)
    {
        run_time = HAL_GetTick() - state->start_tick;
    }

    payload[0] = state->running;
    payload[1] = (uint8_t)state->mode;
    BinaryProtocol_WriteU32LE(&payload[2], run_time);
    BinaryProtocol_SendFrame(BINARY_RSP_STATUS, payload, sizeof(payload));
}

static void BinaryProtocol_ProcessFrame(const BinaryFrame *frame)
{
    uint8_t should_respond;
    float speed;
    float kp;
    float ki;
    float kd;
    float left_speed;
    float right_speed;
    int32_t left_position;
    int32_t right_position;
    ProtocolMode mode;

    if (!BinaryProtocol_IsLocalAddress(frame->address))
    {
        return;
    }

    should_respond = (frame->address != BINARY_PROTOCOL_BROADCAST_ADDRESS);

    switch (frame->command)
    {
    case BINARY_CMD_START:
        if (frame->length != 0U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        App_Start();
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;

    case BINARY_CMD_STOP:
        if (frame->length != 0U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        App_Stop();
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;

    case BINARY_CMD_RESET:
        if (frame->length != 0U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        App_Reset();
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;

    case BINARY_CMD_GET_STATUS:
        if (frame->length != 0U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        if (should_respond) BinaryProtocol_SendStatus();
        break;

    case BINARY_CMD_SET_SPEED:
        if (frame->length != 4U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        if (!BinaryProtocol_ReadFloatLE(frame->payload, &speed))
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_VALUE);
            break;
        }
        App_SetTargetSpeed(speed);
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;

    case BINARY_CMD_SET_POSITION:
        if (frame->length != 4U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        App_SetTargetPosition(BinaryProtocol_ReadI32LE(frame->payload));
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;

    case BINARY_CMD_SET_PID:
        if (frame->length != 12U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        if (!BinaryProtocol_ReadFloatLE(&frame->payload[0], &kp) ||
            !BinaryProtocol_ReadFloatLE(&frame->payload[4], &ki) ||
            !BinaryProtocol_ReadFloatLE(&frame->payload[8], &kd))
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_VALUE);
            break;
        }
        App_SetPID(kp, ki, kd);
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;

    case BINARY_CMD_SET_MODE:
        if (frame->length != 1U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        if (frame->payload[0] > (uint8_t)PROTOCOL_MODE_POSITION)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_VALUE);
            break;
        }
        mode = (ProtocolMode)frame->payload[0];
        App_SetMode(mode);
        if (should_respond) BinaryProtocol_SendAck(frame->command);
        break;
        
    case BINARY_CMD_GET_SPEED:
        if (frame->length != 0U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        App_GetSpeed(&left_speed, &right_speed);
        if (should_respond) BinaryProtocol_SendSpeed(left_speed, right_speed);
        break;

    case BINARY_CMD_GET_POSITION:
        if (frame->length != 0U)
        {
            if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_BAD_LENGTH);
            break;
        }
        App_GetPosition(&left_position, &right_position);
        if (should_respond) BinaryProtocol_SendPosition(left_position, right_position);
        break;

    default:
        if (should_respond) BinaryProtocol_SendError(frame->command, BINARY_ERROR_UNKNOWN_COMMAND);
        break;
    }
}

void BinaryProtocol_Init(UART_HandleTypeDef *huart)
{
    binary_uart = huart;
    rx_queue_read = 0;
    rx_queue_write = 0;
    rx_queue_count = 0;
    rx_error_pending = 0;
    rx_error_command = 0;
    rx_error_code = BINARY_ERROR_NONE;
    memset(&rx_working_frame, 0, sizeof(rx_working_frame));
    memset(rx_frame_queue, 0, sizeof(rx_frame_queue));
    BinaryProtocol_ResetParser();
}

void BinaryProtocol_Task(void)
{
    BinaryFrame frame;
    uint8_t error_pending;
    uint8_t error_command;
    BinaryProtocolError error_code;

    __disable_irq();
    if ((rx_state != RX_WAIT_HEADER_1) &&
        ((HAL_GetTick() - rx_last_byte_tick) > BINARY_INTER_BYTE_TIMEOUT_MS))
    {
        BinaryProtocol_ResetParser();
    }
    __enable_irq();

    __disable_irq();
    error_pending = rx_error_pending;
    error_command = rx_error_command;
    error_code = rx_error_code;
    rx_error_pending = 0;
    __enable_irq();

    if (error_pending != 0U)
    {
        BinaryProtocol_SendError(error_command, error_code);
    }

    while (rx_queue_count > 0U)
    {
        __disable_irq();
        frame = rx_frame_queue[rx_queue_read];
        rx_queue_read = (uint8_t)((rx_queue_read + 1U) % BINARY_FRAME_QUEUE_SIZE);
        rx_queue_count--;
        __enable_irq();

        BinaryProtocol_ProcessFrame(&frame);
    }
}

void BinaryProtocol_ResetReceiver(void)
{
    BinaryProtocol_ResetParser();
}

uint8_t BinaryProtocol_InputByte(uint8_t byte)
{
    if (binary_uart == NULL)
    {
        return 0;
    }

    if (rx_state == RX_WAIT_HEADER_1)
    {
        if (byte != BINARY_PROTOCOL_HEADER_1)
        {
            return 0;
        }
        rx_state = RX_WAIT_HEADER_2;
        rx_last_byte_tick = HAL_GetTick();
        return 1;
    }

    if (rx_state == RX_WAIT_HEADER_2)
    {
        rx_last_byte_tick = HAL_GetTick();
        if (byte == BINARY_PROTOCOL_HEADER_2)
        {
            rx_state = RX_ADDRESS;
            return 1;
        }

        BinaryProtocol_ResetParser();
        if (byte == BINARY_PROTOCOL_HEADER_1)
        {
            rx_state = RX_WAIT_HEADER_2;
            return 1;
        }
        return 0;
    }

    rx_last_byte_tick = HAL_GetTick();

    switch (rx_state)
    {
    case RX_ADDRESS:
        rx_working_frame.address = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(0xFFFFU, byte);
        rx_state = RX_COMMAND;
        break;

    case RX_COMMAND:
        rx_working_frame.command = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        rx_state = RX_LENGTH_LOW;
        break;

    case RX_LENGTH_LOW:
        rx_working_frame.length = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        rx_state = RX_LENGTH_HIGH;
        break;

    case RX_LENGTH_HIGH:
        rx_working_frame.length |= (uint16_t)((uint16_t)byte << 8U);
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        if (rx_working_frame.length > BINARY_PROTOCOL_MAX_PAYLOAD)
        {
            if (BinaryProtocol_IsLocalAddress(rx_working_frame.address) &&
                (rx_working_frame.address != BINARY_PROTOCOL_BROADCAST_ADDRESS))
            {
                BinaryProtocol_SetPendingError(rx_working_frame.command, BINARY_ERROR_BAD_LENGTH);
            }
            BinaryProtocol_ResetParser();
        }
        else if (rx_working_frame.length == 0U)
        {
            rx_state = RX_CRC_LOW;
        }
        else
        {
            rx_payload_index = 0;
            rx_state = RX_PAYLOAD;
        }
        break;

    case RX_PAYLOAD:
        rx_working_frame.payload[rx_payload_index++] = byte;
        rx_calculated_crc = BinaryProtocol_CRC16Update(rx_calculated_crc, byte);
        if (rx_payload_index >= rx_working_frame.length)
        {
            rx_state = RX_CRC_LOW;
        }
        break;

    case RX_CRC_LOW:
        rx_received_crc = byte;
        rx_state = RX_CRC_HIGH;
        break;

    case RX_CRC_HIGH:
        rx_received_crc |= (uint16_t)((uint16_t)byte << 8U);
        if (rx_received_crc == rx_calculated_crc)
        {
            if (BinaryProtocol_IsLocalAddress(rx_working_frame.address))
            {
                if (rx_queue_count < BINARY_FRAME_QUEUE_SIZE)
                {
                    rx_frame_queue[rx_queue_write] = rx_working_frame;
                    rx_queue_write = (uint8_t)((rx_queue_write + 1U) % BINARY_FRAME_QUEUE_SIZE);
                    rx_queue_count++;
                }
                else if (rx_working_frame.address != BINARY_PROTOCOL_BROADCAST_ADDRESS)
                {
                    BinaryProtocol_SetPendingError(rx_working_frame.command, BINARY_ERROR_QUEUE_FULL);
                }
            }
        }
        else if (BinaryProtocol_IsLocalAddress(rx_working_frame.address) &&
                 (rx_working_frame.address != BINARY_PROTOCOL_BROADCAST_ADDRESS))
        {
            BinaryProtocol_SetPendingError(rx_working_frame.command, BINARY_ERROR_BAD_CRC);
        }
        BinaryProtocol_ResetParser();
        break;

    default:
        BinaryProtocol_ResetParser();
        break;
    }

    return 1;
}

uint16_t BinaryProtocol_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    if ((data == NULL) && (length > 0U))
    {
        return 0;
    }

    for (index = 0; index < length; index++)
    {
        crc = BinaryProtocol_CRC16Update(crc, data[index]);
    }
    return crc;
}

HAL_StatusTypeDef BinaryProtocol_SendFrame(uint8_t command,
                                           const uint8_t *payload,
                                           uint16_t payload_length)
{
    uint8_t frame[2U + 1U + 1U + 2U + BINARY_PROTOCOL_MAX_PAYLOAD + 2U];
    uint16_t crc;
    uint16_t frame_length;

    if ((binary_uart == NULL) ||
        (payload_length > BINARY_PROTOCOL_MAX_PAYLOAD) ||
        ((payload == NULL) && (payload_length > 0U)))
    {
        return HAL_ERROR;
    }

    frame[0] = BINARY_PROTOCOL_HEADER_1;
    frame[1] = BINARY_PROTOCOL_HEADER_2;
    frame[2] = BINARY_PROTOCOL_DEVICE_ADDRESS;
    frame[3] = command;
    frame[4] = (uint8_t)(payload_length & 0xFFU);
    frame[5] = (uint8_t)((payload_length >> 8U) & 0xFFU);
    if (payload_length > 0U)
    {
        memcpy(&frame[6], payload, payload_length);
    }

    crc = BinaryProtocol_CRC16(&frame[2], (uint16_t)(4U + payload_length));
    frame[6U + payload_length] = (uint8_t)(crc & 0xFFU);
    frame[7U + payload_length] = (uint8_t)((crc >> 8U) & 0xFFU);
    frame_length = (uint16_t)(8U + payload_length);

    return HAL_UART_Transmit(binary_uart, frame, frame_length, 100U);
}

void BinaryProtocol_SendSpeed(float left_speed, float right_speed)
{
    uint8_t payload[8];

    BinaryProtocol_WriteFloatLE(&payload[0], left_speed);
    BinaryProtocol_WriteFloatLE(&payload[4], right_speed);
    BinaryProtocol_SendFrame(BINARY_RSP_SPEED, payload, sizeof(payload));
}

void BinaryProtocol_SendPosition(int32_t left_position, int32_t right_position)
{
    uint8_t payload[8];

    BinaryProtocol_WriteI32LE(&payload[0], left_position);
    BinaryProtocol_WriteI32LE(&payload[4], right_position);
    BinaryProtocol_SendFrame(BINARY_RSP_POSITION, payload, sizeof(payload));
}
