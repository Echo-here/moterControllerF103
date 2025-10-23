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


//적외선 센서 감지 인터럽트
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

// TIM4,1 인터럽트 핸들러
void TIM4_IRQHandler(void);
void TIM1_IRQHandler(void);

// HAL 콜백 함수 (Update Event 등 처리)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
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
