#include "UART_Protocol.h"
#include "Binary_Protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *protocol_uart = NULL;
static ProtocolState protocol_state = {
    .running = 0,
    .target_speed = 0.0f,
    .target_position = 0,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .mode = PROTOCOL_MODE_MANUAL,
    .start_tick = 0,
    .last_error = "NONE",
};

enum
{
    PROTOCOL_LINE_QUEUE_SIZE = 4,
    PROTOCOL_RX_ERROR_LINE_TOO_LONG = (1U << 0),
    PROTOCOL_RX_ERROR_QUEUE_FULL = (1U << 1),
    PROTOCOL_RX_ERROR_INVALID_CHAR = (1U << 2),
    PROTOCOL_RX_ERROR_UART = (1U << 3)
};

static uint8_t rx_byte = 0;
static char rx_current_line[PROTOCOL_LINE_MAX_LEN];
static char rx_line_queue[PROTOCOL_LINE_QUEUE_SIZE][PROTOCOL_LINE_MAX_LEN];
static volatile uint16_t rx_index = 0;
static volatile uint8_t rx_queue_read = 0;
static volatile uint8_t rx_queue_write = 0;
static volatile uint8_t rx_queue_count = 0;
static volatile uint8_t rx_discard_until_eol = 0;
static volatile uint8_t rx_error_flags = 0;
static volatile uint32_t uart_error_code = HAL_UART_ERROR_NONE;

static char process_line[PROTOCOL_LINE_MAX_LEN];

static const char *Protocol_ModeToString(ProtocolMode mode)
{
    switch (mode)
    {
    case PROTOCOL_MODE_MANUAL:
        return "MANUAL";
    case PROTOCOL_MODE_AUTO:
        return "AUTO";
    case PROTOCOL_MODE_AVOID:
        return "AVOID";
    case PROTOCOL_MODE_SPEED:
        return "SPEED";
    case PROTOCOL_MODE_POSITION:
        return "POSITION";
    default:
        return "UNKNOWN";
    }
}

static uint8_t Protocol_ParseMode(const char *text, ProtocolMode *mode)
{
    if (strcmp(text, "MANUAL") == 0)
    {
        *mode = PROTOCOL_MODE_MANUAL;
        return 1;
    }
    if (strcmp(text, "AUTO") == 0)
    {
        *mode = PROTOCOL_MODE_AUTO;
        return 1;
    }
    if (strcmp(text, "AVOID") == 0)
    {
        *mode = PROTOCOL_MODE_AVOID;
        return 1;
    }
    if (strcmp(text, "SPEED") == 0)
    {
        *mode = PROTOCOL_MODE_SPEED;
        return 1;
    }
    if (strcmp(text, "POSITION") == 0)
    {
        *mode = PROTOCOL_MODE_POSITION;
        return 1;
    }
    return 0;
}

static uint8_t Protocol_IsEnd(const char *text)
{
    return (text == NULL) || (*text == '\0') || (*text == '\r') || (*text == '\n');
}

static uint8_t Protocol_IsFiniteFloat(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return ((bits & 0x7F800000U) != 0x7F800000U);
}

static void Protocol_NormalizeLine(char *line)
{
    char *source;
    char *destination;
    uint8_t pending_space = 0;

    if (line == NULL)
    {
        return;
    }

    source = line;
    destination = line;
    while ((*source == ' ') || (*source == '\t'))
    {
        source++;
    }

    while (*source != '\0')
    {
        if ((*source == ' ') || (*source == '\t'))
        {
            if (destination != line)
            {
                pending_space = 1;
            }
        }
        else
        {
            if (pending_space != 0U)
            {
                *destination++ = ' ';
                pending_space = 0;
            }

            if ((*source >= 'a') && (*source <= 'z'))
            {
                *destination++ = (char)(*source - ('a' - 'A'));
            }
            else
            {
                *destination++ = *source;
            }
        }
        source++;
    }
    *destination = '\0';
}

static const char *Protocol_SkipSpaces(const char *text)
{
    while ((text != NULL) && ((*text == ' ') || (*text == '\t')))
    {
        text++;
    }
    return text;
}

static uint8_t Protocol_ParseFloat1(const char *text, float *value)
{
    char *end_ptr;
    float parsed;

    if ((text == NULL) || (value == NULL))
    {
        return 0;
    }

    text = Protocol_SkipSpaces(text);
    parsed = strtof(text, &end_ptr);
    end_ptr = (char *)Protocol_SkipSpaces(end_ptr);
    if ((end_ptr == text) || !Protocol_IsEnd(end_ptr) || !Protocol_IsFiniteFloat(parsed))
    {
        return 0;
    }

    *value = parsed;
    return 1;
}

static uint8_t Protocol_ParseInt1(const char *text, int32_t *value)
{
    uint32_t magnitude = 0;
    uint32_t limit;
    uint8_t negative = 0;
    uint8_t has_digit = 0;

    if ((text == NULL) || (value == NULL))
    {
        return 0;
    }

    text = Protocol_SkipSpaces(text);
    if ((*text == '+') || (*text == '-'))
    {
        negative = (*text == '-');
        text++;
    }

    limit = (negative != 0U) ? 2147483648U : 2147483647U;
    while ((*text >= '0') && (*text <= '9'))
    {
        uint32_t digit = (uint32_t)(*text - '0');
        has_digit = 1;
        if ((magnitude > (limit / 10U)) ||
            ((magnitude == (limit / 10U)) && (digit > (limit % 10U))))
        {
            return 0;
        }
        magnitude = (magnitude * 10U) + digit;
        text++;
    }

    text = Protocol_SkipSpaces(text);
    if ((has_digit == 0U) || !Protocol_IsEnd(text))
    {
        return 0;
    }

    if (negative != 0U)
    {
        *value = (magnitude == 2147483648U) ? INT32_MIN : -(int32_t)magnitude;
    }
    else
    {
        *value = (int32_t)magnitude;
    }
    return 1;
}

static uint8_t Protocol_ParseFloat3(const char *text, float *a, float *b, float *c)
{
    char *end_ptr;
    const char *start_ptr;
    float first;
    float second;
    float third;

    if ((text == NULL) || (a == NULL) || (b == NULL) || (c == NULL))
    {
        return 0;
    }

    start_ptr = Protocol_SkipSpaces(text);
    first = strtof(start_ptr, &end_ptr);
    if (end_ptr == start_ptr)
    {
        return 0;
    }

    start_ptr = Protocol_SkipSpaces(end_ptr);
    second = strtof(start_ptr, &end_ptr);
    if (end_ptr == start_ptr)
    {
        return 0;
    }

    start_ptr = Protocol_SkipSpaces(end_ptr);
    third = strtof(start_ptr, &end_ptr);
    if (end_ptr == start_ptr)
    {
        return 0;
    }
    end_ptr = (char *)Protocol_SkipSpaces(end_ptr);
    if (!Protocol_IsEnd(end_ptr) ||
        !Protocol_IsFiniteFloat(first) ||
        !Protocol_IsFiniteFloat(second) ||
        !Protocol_IsFiniteFloat(third))
    {
        return 0;
    }

    *a = first;
    *b = second;
    *c = third;
    return 1;
}

static void Protocol_SendText(const char *text)
{
    if ((protocol_uart == NULL) || (text == NULL))
    {
        return;
    }
    HAL_UART_Transmit(protocol_uart, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void Protocol_RestartReceive(void)
{
    if (protocol_uart != NULL)
    {
        HAL_UART_Receive_IT(protocol_uart, &rx_byte, 1);
    }
}

void Protocol_Init(UART_HandleTypeDef *huart)
{
    protocol_uart = huart;
    BinaryProtocol_Init(huart);
    rx_index = 0;
    rx_queue_read = 0;
    rx_queue_write = 0;
    rx_queue_count = 0;
    rx_discard_until_eol = 0;
    rx_error_flags = 0;
    uart_error_code = HAL_UART_ERROR_NONE;
    memset(rx_current_line, 0, sizeof(rx_current_line));
    memset(rx_line_queue, 0, sizeof(rx_line_queue));
    memset(process_line, 0, sizeof(process_line));
    Protocol_RestartReceive();
    Protocol_SendText("OK HOST_PROTOCOL_READY\r\n");
}

void Protocol_Task(void)
{
    uint8_t error_flags;
    uint32_t uart_errors;

    BinaryProtocol_Task();

    __disable_irq();
    error_flags = rx_error_flags;
    uart_errors = uart_error_code;
    rx_error_flags = 0;
    uart_error_code = HAL_UART_ERROR_NONE;
    __enable_irq();

    if ((error_flags & PROTOCOL_RX_ERROR_LINE_TOO_LONG) != 0U)
    {
        Protocol_SendError("LINE_TOO_LONG");
    }
    if ((error_flags & PROTOCOL_RX_ERROR_QUEUE_FULL) != 0U)
    {
        Protocol_SendError("RX_QUEUE_FULL");
    }
    if ((error_flags & PROTOCOL_RX_ERROR_INVALID_CHAR) != 0U)
    {
        Protocol_SendError("INVALID_CHAR");
    }
    if ((error_flags & PROTOCOL_RX_ERROR_UART) != 0U)
    {
        if ((uart_errors & HAL_UART_ERROR_ORE) != 0U) Protocol_SendError("UART_OVERRUN");
        if ((uart_errors & HAL_UART_ERROR_FE) != 0U) Protocol_SendError("UART_FRAME");
        if ((uart_errors & HAL_UART_ERROR_NE) != 0U) Protocol_SendError("UART_NOISE");
        if ((uart_errors & HAL_UART_ERROR_PE) != 0U) Protocol_SendError("UART_PARITY");
        if (uart_errors == HAL_UART_ERROR_NONE) Protocol_SendError("UART_RX");
    }

    while (rx_queue_count > 0)
    {
        __disable_irq();
        strncpy(process_line, rx_line_queue[rx_queue_read], sizeof(process_line) - 1);
        process_line[sizeof(process_line) - 1] = '\0';
        rx_queue_read = (uint8_t)((rx_queue_read + 1) % PROTOCOL_LINE_QUEUE_SIZE);
        rx_queue_count--;
        __enable_irq();

        Protocol_ProcessLine(process_line);
    }
}

void Protocol_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((protocol_uart == NULL) || (huart != protocol_uart))
    {
        return;
    }

    if (BinaryProtocol_InputByte(rx_byte) != 0U)
    {
        Protocol_RestartReceive();
        return;
    }

    if ((rx_byte == '\n') || (rx_byte == '\r'))
    {
        if (rx_discard_until_eol != 0U)
        {
            rx_discard_until_eol = 0;
            rx_index = 0;
        }
        else if (rx_index > 0)
        {
            rx_current_line[rx_index] = '\0';
            if (rx_queue_count < PROTOCOL_LINE_QUEUE_SIZE)
            {
                strncpy(rx_line_queue[rx_queue_write], rx_current_line, PROTOCOL_LINE_MAX_LEN);
                rx_line_queue[rx_queue_write][PROTOCOL_LINE_MAX_LEN - 1] = '\0';
                rx_queue_write = (uint8_t)((rx_queue_write + 1) % PROTOCOL_LINE_QUEUE_SIZE);
                rx_queue_count++;
            }
            else
            {
                rx_error_flags |= PROTOCOL_RX_ERROR_QUEUE_FULL;
            }
            rx_index = 0;
        }
    }
    else if (rx_discard_until_eol != 0U)
    {
        /* Wait for the next line after a malformed or oversized command. */
    }
    else if ((rx_byte == 0x08U) || (rx_byte == 0x7FU))
    {
        if (rx_index > 0U)
        {
            rx_index--;
        }
    }
    else if (((rx_byte >= 0x20U) && (rx_byte <= 0x7EU)) || (rx_byte == '\t'))
    {
        if (rx_index < (PROTOCOL_LINE_MAX_LEN - 1U))
        {
            rx_current_line[rx_index++] = (char)rx_byte;
        }
        else
        {
            rx_index = 0;
            rx_discard_until_eol = 1;
            rx_error_flags |= PROTOCOL_RX_ERROR_LINE_TOO_LONG;
        }
    }
    else
    {
        rx_index = 0;
        rx_discard_until_eol = 1;
        rx_error_flags |= PROTOCOL_RX_ERROR_INVALID_CHAR;
    }

    Protocol_RestartReceive();
}

void Protocol_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((protocol_uart == NULL) || (huart != protocol_uart))
    {
        return;
    }

    uart_error_code |= huart->ErrorCode;
    rx_error_flags |= PROTOCOL_RX_ERROR_UART;
    rx_index = 0;
    rx_discard_until_eol = 0;
    BinaryProtocol_ResetReceiver();

    if (huart->RxState == HAL_UART_STATE_READY)
    {
        Protocol_RestartReceive();
    }
}

void Protocol_ProcessLine(char *line)
{
    char *arguments;
    float value = 0.0f;
    int32_t pos = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float left_speed = 0.0f;
    float right_speed = 0.0f;
    int32_t left_pos = 0;
    int32_t right_pos = 0;
    ProtocolMode mode;

    if (line == NULL)
    {
        return;
    }

    Protocol_NormalizeLine(line);
    if (line[0] == '\0')
    {
        return;
    }

    arguments = strchr(line, ' ');
    if (arguments != NULL)
    {
        *arguments++ = '\0';
    }
    else
    {
        arguments = line + strlen(line);
    }

    if (strcmp(line, "PING") == 0)
    {
        if (arguments[0] == '\0') Protocol_SendText("PONG\r\n");
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "START") == 0)
    {
        if (arguments[0] == '\0')
        {
            App_Start();
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "STOP") == 0)
    {
        if (arguments[0] == '\0')
        {
            App_Stop();
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "RESET") == 0)
    {
        if (arguments[0] == '\0')
        {
            App_Reset();
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_FORMAT");
    }
    else if ((strcmp(line, "STATUS") == 0) || (strcmp(line, "GET_STATUS") == 0))
    {
        if (arguments[0] == '\0') Protocol_SendStatus();
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "GET_SPEED") == 0)
    {
        if (arguments[0] == '\0')
        {
            App_GetSpeed(&left_speed, &right_speed);
            Protocol_SendSpeed(left_speed, right_speed);
        }
        else Protocol_SendError("BAD_FORMAT");
    }
    else if ((strcmp(line, "GET_POS") == 0) || (strcmp(line, "GET_POSITION") == 0))
    {
        if (arguments[0] == '\0')
        {
            App_GetPosition(&left_pos, &right_pos);
            Protocol_SendPosition(left_pos, right_pos);
        }
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "GET_PID") == 0)
    {
        if (arguments[0] == '\0') Protocol_SendPID();
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "GET_TARGET") == 0)
    {
        if (arguments[0] == '\0') Protocol_SendTarget();
        else Protocol_SendError("BAD_FORMAT");
    }
    else if (strcmp(line, "SET_SPEED") == 0)
    {
        if (Protocol_ParseFloat1(arguments, &value))
        {
            App_SetTargetSpeed(value);
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_VALUE");
    }
    else if ((strcmp(line, "SET_POS") == 0) || (strcmp(line, "SET_POSITION") == 0))
    {
        if (Protocol_ParseInt1(arguments, &pos))
        {
            App_SetTargetPosition(pos);
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_VALUE");
    }
    else if (strcmp(line, "SET_PID") == 0)
    {
        if (Protocol_ParseFloat3(arguments, &kp, &ki, &kd))
        {
            App_SetPID(kp, ki, kd);
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_VALUE");
    }
    else if (strcmp(line, "SET_MODE") == 0)
    {
        if (Protocol_ParseMode(arguments, &mode))
        {
            App_SetMode(mode);
            Protocol_SendOK();
        }
        else Protocol_SendError("BAD_MODE");
    }
    else
    {
        Protocol_SendError("UNKNOWN_CMD");
    }
}

void Protocol_SendOK(void)
{
    Protocol_SendText("OK\r\n");
}

void Protocol_SendError(const char *message)
{
    char tx_buffer[64];
    if ((message == NULL) || (message[0] == '\0'))
    {
        message = "UNKNOWN";
    }
    snprintf(tx_buffer, sizeof(tx_buffer), "ERR %s\r\n", message);
    Protocol_SendText(tx_buffer);
}

void Protocol_SendStatus(void)
{
    char tx_buffer[96];
    uint32_t run_time = 0;

    if (protocol_state.running)
    {
        run_time = HAL_GetTick() - protocol_state.start_tick;
    }

    snprintf(tx_buffer,
             sizeof(tx_buffer),
             "STATUS MODE=%s RUN=%lu\r\n",
             Protocol_ModeToString(protocol_state.mode),
             (unsigned long)run_time);
    Protocol_SendText(tx_buffer);
}

void Protocol_SendSpeed(float left_speed, float right_speed)
{
    char tx_buffer[64];
    snprintf(tx_buffer, sizeof(tx_buffer), "SPEED L=%.2f R=%.2f\r\n", left_speed, right_speed);
    Protocol_SendText(tx_buffer);
}

void Protocol_SendPosition(int32_t left_pos, int32_t right_pos)
{
    char tx_buffer[64];
    snprintf(tx_buffer, sizeof(tx_buffer), "POS L=%ld R=%ld\r\n", (long)left_pos, (long)right_pos);
    Protocol_SendText(tx_buffer);
}

void Protocol_SendSensor(float distance, float voltage, float temperature)
{
    char tx_buffer[96];
    snprintf(tx_buffer,
             sizeof(tx_buffer),
             "SENSOR DIST=%.2f VOLT=%.2f TEMP=%.2f\r\n",
             distance,
             voltage,
             temperature);
    Protocol_SendText(tx_buffer);
}

void Protocol_SendPID(void)
{
    char tx_buffer[96];
    snprintf(tx_buffer,
             sizeof(tx_buffer),
             "PID KP=%.3f KI=%.3f KD=%.3f\r\n",
             protocol_state.kp,
             protocol_state.ki,
             protocol_state.kd);
    Protocol_SendText(tx_buffer);
}

void Protocol_SendTarget(void)
{
    char tx_buffer[96];
    snprintf(tx_buffer,
             sizeof(tx_buffer),
             "TARGET SPEED=%.3f POS=%ld\r\n",
             protocol_state.target_speed,
             (long)protocol_state.target_position);
    Protocol_SendText(tx_buffer);
}

const ProtocolState *Protocol_GetState(void)
{
    return &protocol_state;
}

__attribute__((weak)) void App_Start(void)
{
    protocol_state.running = 1;
    protocol_state.start_tick = HAL_GetTick();
    strcpy(protocol_state.last_error, "NONE");
}

__attribute__((weak)) void App_Stop(void)
{
    protocol_state.running = 0;
}

__attribute__((weak)) void App_Reset(void)
{
    protocol_state.running = 0;
    protocol_state.target_speed = 0.0f;
    protocol_state.target_position = 0;
    protocol_state.kp = 0.0f;
    protocol_state.ki = 0.0f;
    protocol_state.kd = 0.0f;
    protocol_state.mode = PROTOCOL_MODE_MANUAL;
    protocol_state.start_tick = 0;
    strcpy(protocol_state.last_error, "NONE");
}

__attribute__((weak)) void App_SetTargetSpeed(float speed)
{
    protocol_state.target_speed = speed;
}

__attribute__((weak)) void App_SetTargetPosition(int32_t position)
{
    protocol_state.target_position = position;
}

__attribute__((weak)) void App_SetPID(float kp, float ki, float kd)
{
    protocol_state.kp = kp;
    protocol_state.ki = ki;
    protocol_state.kd = kd;
}

__attribute__((weak)) void App_SetMode(ProtocolMode mode)
{
    protocol_state.mode = mode;
}

__attribute__((weak)) void App_GetSpeed(float *left_speed, float *right_speed)
{
    if (left_speed != NULL) *left_speed = 0.0f;
    if (right_speed != NULL) *right_speed = 0.0f;
}

__attribute__((weak)) void App_GetPosition(int32_t *left_pos, int32_t *right_pos)
{
    if (left_pos != NULL) *left_pos = 0;
    if (right_pos != NULL) *right_pos = 0;
}
