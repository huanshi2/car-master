#ifndef __EC_MOTOR_H
#define __EC_MOTOR_H

#include "stdbool.h"
#include "misc.h"


//T_ 开头的函数用于测试


void T_Motor_Initialize(void);
void T_Motor_Delay(volatile unsigned ms);
void T_Motor_Rotate(bool direction, unsigned delay);
void T_Motor_Move(unsigned idx_a, unsigned idx_b, bool direction, unsigned delay);
void T_Motor_Plus(unsigned idx, bool level);

void Motor_Initialize(void);
void Motor_EnableDriver(bool bEnable);
void Motor_Config(uint8_t idx, uint16_t steps, bool start);
void Motor_Config1(uint16_t Va, uint16_t Vb,uint16_t Vc);
void Motor_Start(uint8_t idx);
void Motor_Stop(uint8_t idx);
void Motor_Speed(uint8_t idx, uint16_t steps);
void Motor_SetCCW(uint8_t idx, bool ccw);
bool Motor_IsCCW( uint8_t idx );



#endif
