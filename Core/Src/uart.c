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

        char left_dir   = cmd[0];
		int  left_speed = atoi(&cmd[2]);
        char right_dir   = cmd[4];
        int  right_speed = atoi(&cmd[6]);

        if ((left_dir == 'f' || left_dir == 'b') &&
            (right_dir == 'f' || right_dir == 'b') &&
            (left_speed >= 0 && left_speed <= 9)&&
			(right_speed >= 0 && right_speed <= 9))
        {
        	Motor_SetBrake(false);
            execute_command(left_dir, left_speed, right_dir, right_speed);
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
void execute_command(char left_dir, int left_speed, char right_dir, int right_speed)
{
    uint16_t left_duty = left_speed * PWM_DUTY_SCALE;
	uint16_t right_duty = right_speed * PWM_DUTY_SCALE;

    left_motor.target = left_duty;
    HAL_GPIO_WritePin(left_motor.dir_port, left_motor.dir_pin,
                          (left_dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);

    right_motor.target = right_duty;
    HAL_GPIO_WritePin(right_motor.dir_port, right_motor.dir_pin,
                          (right_dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);

    printf("CMD -> Left_Motor: Dir:%c Speed:%d (duty=%d)\n CMD -> Right_Motor: Dir:%c Speed:%d (duty=%d)\n",
           left_dir, left_speed, left_duty, right_dir, right_speed, right_duty);
}
