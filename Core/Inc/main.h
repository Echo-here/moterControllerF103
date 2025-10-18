/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>     // atoi 함수 선언
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t prev_cnt;
    uint32_t total_count;
} Encoder_t;

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
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//printf로 uart 출력
int __io_putchar(int ch);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// ------------------- 명령 파싱 함수 -------------------
void parse_command(char *cmd);

// ------------------- 명령 실행 함수 -------------------
void execute_command(char motor, char dir, int speed);

//점진적 모터 속도 제어
void Transform_PWM();

// 모터속도 점진젓증가
void Motor_UpdatePWM(Motor_t *motor);

//점진적 모터 정지
void Stop_Motor(void);

//모터드라이버 브레이크
void set_brake(bool on);

//적외선 센서 감지 인터럽트
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

// TIM4,1 인터럽트 핸들러
void TIM4_IRQHandler(void);
void TIM1_IRQHandler(void);

// HAL 콜백 함수 (Update Event 등 처리)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//엔코더 값 가져오기
int32_t Encoder_GetPosition(Encoder_t *encoder);

// 카운터 값 리셋
void Encoder_Reset(Encoder_t *encoder);

void encoder_process_once(Encoder_t *encoder);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define moter_left_Pin GPIO_PIN_0
#define moter_left_GPIO_Port GPIOA
#define moter_right_Pin GPIO_PIN_1
#define moter_right_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define moter_right_dir_Pin GPIO_PIN_10
#define moter_right_dir_GPIO_Port GPIOB
#define moter_left_dir_Pin GPIO_PIN_11
#define moter_left_dir_GPIO_Port GPIOB
#define left_encoder_A_Pin GPIO_PIN_8
#define left_encoder_A_GPIO_Port GPIOA
#define left_encoder_B_Pin GPIO_PIN_9
#define left_encoder_B_GPIO_Port GPIOA
#define right_encoder_A_Pin GPIO_PIN_6
#define right_encoder_A_GPIO_Port GPIOB
#define right_encoder_B_Pin GPIO_PIN_7
#define right_encoder_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
