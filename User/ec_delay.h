
#ifndef __EC_DELAY_H
#define __EC_DELAY_H

#include "ec_common.h"

void Delay_Initialize(void);
void Delay_us(uint32_t nus);
void Delay_ms(u16 nms);

//以下两个函数为移植 DMP 函数库而定义的
void MPU6050_delay_ms(unsigned long num_ms);
void MPU6050_get_ms(unsigned long *count);

#endif
