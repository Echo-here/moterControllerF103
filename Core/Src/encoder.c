#include "encoder.h"

Encoder_t encoder1 = { .htim = &htim1, .prev_cnt = 0, .total_count = 0 };
Encoder_t encoder4 = { .htim = &htim4, .prev_cnt = 0, .total_count = 0 };

void Encoder_SendTotalCount() {
	 printf("%ld, %ld\n", encoder1.total_count, encoder4.total_count);

}

void Encoder_Update(Encoder_t *encoder){
	 uint16_t curr_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(encoder->htim);

	 // 16비트 카운터 wrap 처리
	 int32_t delta = (int32_t)((int16_t)(curr_cnt - encoder->prev_cnt));
	 encoder->prev_cnt = curr_cnt;
	 // 무한 누적
	 encoder->total_count += delta;


}
