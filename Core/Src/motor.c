#include "motor.h"

extern TIM_HandleTypeDef htim2;

Motor_t left_motor = {
    .htim = &htim2,
    .channel = TIM_CHANNEL_1,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_11,
    .brk_port = GPIOB,
    .brk_pin = GPIO_PIN_1,
    .target = 0,
    .current = 0
};

Motor_t right_motor = {
    .htim = &htim2,
    .channel = TIM_CHANNEL_2,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_10,
    .brk_port = GPIOB,
    .brk_pin = GPIO_PIN_2,
    .target = 0,
    .current = 0
};

//점진적 모터 속도 제어
void Motor_UpdateAllPWM(){
	Motor_UpdatePWM(&left_motor);
	Motor_UpdatePWM(&right_motor);
}

void Motor_UpdatePWM(Motor_t *motor) {
    if (motor->current < motor->target) motor->current++;
    else if (motor->current > motor->target) motor->current--;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->current);
}

//점진적 모터 정지
void Motor_StopAll(void)
{
  // 양쪽 모터의 PWM을 0으로 설정하여 정지
	left_motor.target  = 0;
	right_motor.target = 0;

  // 필요 시 후속 처리 (예: 방향핀 LOW, 브레이크)
}

//모터드라이버 브레이크
void Motor_SetBrake(bool on) {
    if(on) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    	left_motor.target = 0;
    	right_motor.target = 0;
    	left_motor.current = 0;
    	right_motor.current = 0;

    }
    else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    }
}
