#ifndef HOST_COMPUTER_UART_PROTOCOL_H
#define HOST_COMPUTER_UART_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_LINE_MAX_LEN 96

typedef enum
{
    PROTOCOL_MODE_MANUAL = 0,
    PROTOCOL_MODE_AUTO,
    PROTOCOL_MODE_AVOID,
    PROTOCOL_MODE_SPEED,
    PROTOCOL_MODE_POSITION
} ProtocolMode;

typedef struct
{
    uint8_t running;
    float target_speed;
    int32_t target_position;
    float kp;
    float ki;
    float kd;
    ProtocolMode mode;
    uint32_t start_tick;
    char last_error[32];
} ProtocolState;

void Protocol_Init(UART_HandleTypeDef *huart);
void Protocol_Task(void);
void Protocol_ProcessLine(char *line);
void Protocol_RxCpltCallback(UART_HandleTypeDef *huart);
void Protocol_ErrorCallback(UART_HandleTypeDef *huart);

void Protocol_SendOK(void);
void Protocol_SendError(const char *message);
void Protocol_SendStatus(void);
void Protocol_SendSpeed(float left_speed, float right_speed);
void Protocol_SendPosition(int32_t left_pos, int32_t right_pos);
void Protocol_SendSensor(float distance, float voltage, float temperature);
void Protocol_SendPID(void);
void Protocol_SendTarget(void);

const ProtocolState *Protocol_GetState(void);

void App_Start(void);
void App_Stop(void);
void App_Reset(void);
void App_SetTargetSpeed(float speed);
void App_SetTargetPosition(int32_t position);
void App_SetPID(float kp, float ki, float kd);
void App_SetMode(ProtocolMode mode);
void App_GetSpeed(float *left_speed, float *right_speed);
void App_GetPosition(int32_t *left_pos, int32_t *right_pos);

#ifdef __cplusplus
}
#endif

#endif
