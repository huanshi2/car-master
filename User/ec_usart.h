
#ifndef __EC_USART_H
#define __EC_USART_H

#include "stdio.h"
#include "stdbool.h"

//定义最大接收字节数 200
#define USART_REC_LEN 200
void Uart1_Initialize(uint32_t bound);

#endif
