#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"   // TIM_HandleTypeDef, GPIO_TypeDef, HAL 매크로 등
#include <stdint.h>          // uint16_t, uint32_t
#include <stdbool.h>         // bool
#include <stdio.h>           // printf



//모터 구조
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
    uint16_t target;
    uint16_t current;
    GPIO_TypeDef *dir_port;    // 방향 제어 포트
    uint16_t dir_pin;          // 방향 제어 핀
    GPIO_TypeDef *brk_port;    // 브레이크 포트
    uint16_t brk_pin;          // 브레이크 핀


} Motor_t;


extern Motor_t left_motor;   // 다른 파일에서 접근 가능하도록 extern 선언
extern Motor_t right_motor;

//점진적 모터 속도 제어
void Motor_UpdateAllPWM();

// 모터속도 점진젓증가
void Motor_UpdatePWM(Motor_t *motor);

//점진적 모터 정지
void Motor_StopAll(void);

//모터드라이버 브레이크
void Motor_SetBrake(bool on);


#endif
