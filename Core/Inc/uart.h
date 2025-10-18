#include "stm32f1xx_hal.h"  // HAL_UART 포함
#include <string.h>   // memset, strlen
#include <stdlib.h>   // atoi

#include "motor.h"
#include "encoder.h"

extern uint8_t rx_data;
extern UART_HandleTypeDef huart2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// ------------------- 명령 파싱 함수 -------------------
void parse_command(char *cmd);

// ------------------- 명령 실행 함수 -------------------
void execute_command(char motor, char dir, int speed);

