/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>     // atoi 함수 선언
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_DUTY_SCALE 4.9f
#define IR_THRESHOLD 4000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//엔코더 구조체
typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t total;
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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[20]; // UART 수신 데이터를 임시 저장할 버퍼
uint8_t rx_index = 0;    // 수신 버퍼의 현재 위치
uint8_t rx_data;         // UART로 수신한 1바이트 데이터

Encoder_t encoder1 = { .htim = &htim1, .total = 0 };
Encoder_t encoder4 = { .htim = &htim4, .total = 0 };

Motor_t left_motor = {
    .htim = &htim2,
    .pwm_channel = TIM_CHANNEL_1,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_11,
    .brk_port = GPIOB,
    .brk_pin = GPIO_PIN_1,
    .target = 0,
    .current = 0
};

Motor_t right_motor = {
    .htim = &htim2,
    .pwm_channel = TIM_CHANNEL_2,
    .dir_port = GPIOB,
    .dir_pin = GPIO_PIN_10,
    .brk_port = GPIOB,
    .brk_pin = GPIO_PIN_2,
    .target = 0,
    .current = 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
volatile int32_t encoder_count = 0;  // 인코더 카운터
volatile int32_t cycle = 18450;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//printf로 uart 출력
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 1000);
  if (ch == '\n')
    HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 1000);
  return ch;
}

/**
  * @brief  UART 수신 완료 시 호출되는 콜백 함수
  * @param  huart: UART 핸들
  * @retval None
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART2)
//    {
//        if (rx_data == '\n' || rx_data == '\r')
//        {
//            rx_buffer[rx_index] = '\0';
//
//            // --- 단일 명령 (예: "s") ---
//            if (rx_index == 1 && rx_buffer[0] == 's')
//            {
//                Stop_Motor();
//                printf("stop CMD received");
//            }
//            else if (rx_index == 1 && rx_buffer[0] == 'e')
//            {
//                set_brake(true);
//                printf("brake CMD received");
//            }
//            // --- 일반 모터 명령 (예: "l f 3") ---
//            else if (rx_index >= 5)
//            {
//                set_brake(false);
//                char motor = rx_buffer[0];   // 'l' or 'r'
//                char dir   = rx_buffer[2];   // 'f' or 'b'
//                int speed  = atoi(&rx_buffer[4]); // duty (0~9)
//
//                if ((motor == 'l' || motor == 'r') &&
//                    (dir == 'f' || dir == 'b') &&
//                    (speed >= 0 && speed <= 9))
//                {
//                    uint16_t duty = speed * PWM_DUTY_SCALE;
//
//                    if (motor == 'l')  // Left Motor
//                    {
//                    	left_motor.target= duty;
//                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,
//                            (dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);
//
//                    }
//                    else if (motor == 'r') // Right Motor
//                    {
//                       right_motor.target = duty;
//                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,
//                            (dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);
//                    }
//                }
//                else
//              {
//                    HAL_UART_Transmit(&huart2,
//                        (uint8_t*)"Invalid CMD\r\n", 13, HAL_MAX_DELAY);
//                }
//            }
//
//            // 버퍼 초기화
//            rx_index = 0;
//            memset(rx_buffer, 0, sizeof(rx_buffer));
//        }
//        else
//        {
//            if (rx_index < sizeof(rx_buffer) - 1)
//                rx_buffer[rx_index++] = rx_data;
//        }
//
//        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
//    }
//}
// ------------------- UART 콜백 -------------------
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
            Stop_Motor();
            printf("Stop command received\n");
        }
        else if (cmd[0] == 'e')
        {
            set_brake(true);
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
            set_brake(false);
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
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,
                          (dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    else if (motor == 'r') // Right Motor
    {
        right_motor.target = duty;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,
                          (dir == 'f') ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    printf("CMD -> Motor:%c Dir:%c Speed:%d (duty=%d)\n",
           motor, dir, speed, duty);
}




//점진적 모터 속도 제어
void Transform_PWM(){
	Motor_UpdatePWM(&left_motor);
	Motor_UpdatePWM(&right_moter);
}

void Motor_UpdatePWM(Motor_t *motor) {
    if (motor->current < motor->target) motor->current++;
    else if (motor->current > motor->target) motor->current--;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->current);
}

//점진적 모터 정
void Stop_Motor(void)
{
  // 양쪽 모터의 PWM을 0으로 설정하여 정지
	left_motor.target  = 0;
	right_motor.target = 0;

  // 필요 시 후속 처리 (예: 방향핀 LOW, 브레이크)
}
//모터드라이버 브레이크
void set_brake(bool on) {
    if(on) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }
    else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    }
}

//적외선 센서 감지 인터럽트
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        uint32_t adc_value = HAL_ADC_GetValue(hadc);
        if(adc_value < IR_THRESHOLD){
        	Stop_Motor();
        }
    }
}

// TIM4,1 인터럽트 핸들러
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}
void TIM1_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

// HAL 콜백 함수 (Update Event 등 처리)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    volatile int32_t *encoder_total = NULL;

    if(htim->Instance == TIM1) encoder_total = &encoder_total_left;
    else if(htim->Instance == TIM4) encoder_total = &encoder_total_right;
    else return;

    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) *encoder_total -= 0x10000;
    else *encoder_total += 0x10000;
}

//엔코더 값 가져오
int32_t Encoder_GetPosition(Encoder_t *encoder)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(encoder->htim) + encoder->total;
}

// 카운터 값 리셋
void Encoder_Reset(Encoder_t *encoder)
{
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->total = 0;  // 누적값도 같이 리셋해주면 안전함
}

// 타이머 콜백
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)  // TIM3 = 1ms 주기 타이머
    {
        Transform_PWM();   // 모터 점진 제어
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // 왼쪽, 오른쪽 모터 PWM 채널 시작
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Left Motor PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Right Motor PWM

  // UART 수신 인터럽트 시작 (1바이트씩 수신)
  HAL_UART_Receive_IT(&huart2, &rx_data, 1); // UART Receive 1 byte
  HAL_ADC_Start_IT(&hadc1);
  // TIM3 시작 필요
  HAL_TIM_Base_Start(&htim3);

  // 엔코더 입력을 위한 타이머 입력 캡처 시작

  // 초기 PWM 값을 0으로 설정
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|moter_right_dir_Pin|moter_left_dir_Pin
                          |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 moter_right_dir_Pin moter_left_dir_Pin
                           PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|moter_right_dir_Pin|moter_left_dir_Pin
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
