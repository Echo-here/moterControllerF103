#include "uart.h"
#include "motor.h"


#define PWM_DUTY_SCALE 4.9f

uint8_t rx_buffer[20]; // UART 수신 데이터를 임시 저장할 버퍼
uint8_t rx_index = 0;    // 수신 버퍼의 현재 위치
uint8_t rx_data;         // UART로 수신한 1바이트 데이터

UART_HandleTypeDef huart2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {

        if (rx_data == '\n' || rx_data == '\r')
        {
            rx_buffer[rx_index] = '\0';

            parse_command((char*)rx_buffer);   // 파싱 전담 함수 호출
            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }
        else
        {
            if (rx_index < sizeof(rx_buffer) - 1)
                rx_buffer[rx_index++] = rx_data;
        }

        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

// ------------------- 명령 파싱 함수 -------------------
void parse_command(char *cmd)
{
    if (strlen(cmd) == 1)
    {
        if (cmd[0] == 's')
        {
        	Motor_StopAll();
            printf("Stop command received\n");
        }
        else if (cmd[0] == 'e')
        {
        	Motor_SetBrake(true);
            printf("Brake command received\n");
        }
        else
        {
            printf("Unknown single command: %s\n", cmd);
        }
    }
    else if (strlen(cmd) >= 5)
    {
        char motor = cmd[0];   // 'l' or 'r'
        char dir   = cmd[2];   // 'f' or 'b'
        int  speed = atoi(&cmd[4]);

        if ((motor == 'l' || motor == 'r') &&
            (dir == 'f' || dir == 'b') &&
            (speed >= 0 && speed <= 9))
        {
        	Motor_SetBrake(false);
            execute_command(motor, dir, speed);
        }
        else
        {
            printf("Invalid command: %s\n", cmd);
        }
    }
    else
    {
        printf("Unknown command format: %s\n", cmd);
    }
}

// ------------------- 명령 실행 함수 -------------------
void execute_command(char motor, char dir, int speed)
{
    uint16_t duty = speed * PWM_DUTY_SCALE;

    if (motor == 'l')  // Left Motor
    {
        left_motor.target = duty;
        HAL_GPIO_WritePin(left_motor.dir_port, left_motor.dir_pin,
                          (dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else if (motor == 'r') // Right Motor
    {
        right_motor.target = duty;
        HAL_GPIO_WritePin(right_motor.dir_port, right_motor.dir_pin,
                          (dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    printf("CMD -> Motor:%c Dir:%c Speed:%d (duty=%d)\n",
           motor, dir, speed, duty);
}
