#ifndef __CONTROL_H
#define __CONTROL_H

#include "stdint.h"

int absolute(int a);
void delay1(uint32_t time);

void go_anywhere(float angel, int speed);
void set_direction(uint16_t angel,uint16_t speed);
void move_time(uint16_t angel, uint16_t speed, uint16_t time);
void move_distance(uint16_t angel, uint16_t speed, uint16_t distance);
void go_left(uint16_t angel, uint32_t sudu);
void go_right(uint16_t angel, uint32_t speed);
uint16_t test_distance(void);
int get_distance(int num);
int gengsui(uint16_t);
int gengsui2(uint16_t);
void move_ahead(uint16_t direction ,uint8_t changedirection,uint16_t speed,uint16_t distance);
void move_ahead1(uint16_t direction ,uint8_t changedirection,uint16_t speed,uint16_t distance);
void move_a(uint16_t direction ,uint16_t speed ,uint16_t distance);

#endif
