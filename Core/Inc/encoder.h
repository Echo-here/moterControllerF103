#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"   // TIM_HandleTypeDef, GPIO_TypeDef, HAL 매크로 등
#include <stdint.h>          // uint16_t, uint32_t
#include <stdbool.h>         // bool
#include <stdio.h>           // printf

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t prev_cnt;
    uint32_t total_count;
} Encoder_t;

extern Encoder_t encoder1;
extern Encoder_t encoder4;

void Encoder_SendTotalCount();
void Encoder_Update(Encoder_t *encoder);


#endif // ENCODER_H
