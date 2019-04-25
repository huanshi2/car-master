#ifndef __EC_TIMER_H
#define __EC_TIMER_H

#include "stm32f10x.h"
#include "stdbool.h"

void TIM2_NVIC_Configuration(void);
void Motor_TIM_Config( uint8_t idx, uint16_t steps);
void Motor_Speed(uint8_t idx, uint16_t steps);
void Motor_TIM_Enable(uint8_t idx, bool en);
void TIM2_Mode_Config(void);
void TIM2_IRQHandler(void);

#endif
