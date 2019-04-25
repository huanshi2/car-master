#include "vl53l0x.h"
#include "vl53l0x_gen.h"
#include "stack.h"
#include "stm32f10x_exti.h"
#include <math.h>
#include "ec_motor.h"
#include "ec_led.h"
#include "stdint.h"
#include "ec_delay.h"
#include "ec_motor.h"
#include "control.h"
#include "stdint.h"

//计算轮子速度所用到的常量
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)   

//三个轮子的速度和传感器角度
int Va,Vb,Vc;
float Pitch,Roll,Yaw;



/******************************************************************************/
extern VL53L0X_Dev_t vl53l0x_dev,vl53l0x_dev2,vl53l0x_dev3;//设备I2C数据参数
/******************************************************************************/

int absolute(int a)
{
	if (a<0)  return -a;
	else return a;
}

void delay1(u32 time)
{
	for(;0<time;time--)
	{
	   Delay_ms(10);
	}
}

void go_anywhere(float angel, int speed)
{
	Va = -speed*cos(angel/180*3.141592658);
	Vb = X_PARAMETER*speed*cos(angel/180*3.1415926) - Y_PARAMETER*speed*sin(angel/180*3.1415926);
	Vc = X_PARAMETER*speed*cos(angel/180*3.1415926) + Y_PARAMETER*speed*sin(angel/180*3.1415926);	
}


//其他函数算得三个轮子转速，此函数只是驱动轮子转动。
void set_direction(u16 angel,u16 speed)
{ 
	go_anywhere(angel, speed);
	if (Va>0) Motor_SetCCW(1, true);else  Motor_SetCCW(1, false);
	if (Vb>0) Motor_SetCCW(2, true);else  Motor_SetCCW(2, false);
	if (Vc>0) Motor_SetCCW(3, true);else  Motor_SetCCW(3, false);
	if (Va==0) Motor_Config(1, absolute(Va), false); else Motor_Config(1, absolute(Va), true);
	if (Vb==0) Motor_Config(2, absolute(Vb), false); else Motor_Config(2, absolute(Vb), true);
	if (Vc==0) Motor_Config(3, absolute(Vc), false); else Motor_Config(3, absolute(Vc), true);
}


void move_time(u16 angel, u16 speed, u16 time)
{ 
	Led_On(true);
  set_direction(angel,speed);  //speed典型值600，1200
	for(;time>0;time--)
	{
		 Delay_ms(10);
	}
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
	Led_On(false);
}

void move_distance(u16 angel, u16 speed, u16 distance)
{ 
	u32 time;
	Led_On(true);
  set_direction(angel,speed);
	time=100*distance*12000/(78*speed);  //speed典型值600，   speed=1200  速度为8cm/s 左右
	for(;time>0;time--)
	{
		 Delay_ms(10);
	}
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
	Led_On(false);
}

void go_left(u16 angel, u32 sudu)
{ 
	//设置初始转动方向
	Motor_SetCCW(1, true);  //初始方向设定为正转
	Motor_SetCCW(2, true);     
	Motor_SetCCW(3, true);

	Motor_Config(1, sudu, true);  //初始speed=600
	Motor_Config(2, sudu, true);    
  Motor_Config(3, sudu, true);
	
	Led_On(false);
	delay1(5*angel/(sudu/600));

	Led_On(true);
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
}

void go_right(u16 angel, u32 speed)
{ 
	//设置初始转动方向
	Motor_SetCCW(1, false);  //初始方向设定为正转
	Motor_SetCCW(2, false);     
	Motor_SetCCW(3, false);

	Motor_Config(1, speed, true);  //初始speed=600
	Motor_Config(2, speed, true);    
  Motor_Config(3, speed, true);
	
	Led_On(true);
	delay1(5*angel/(speed/600));

	Led_On(false);
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
}




u16 test_distance()
{  
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区

	vl53l0x_start_single_test(&vl53l0x_dev,&vl53l0x_data,buf);//执行一次测量
	return Distance_data;
}


int get_distance(int num)
{
	int distance;
	switch(num)
   {
   case 1 :
	 vl53l0x_set_mode(&vl53l0x_dev,0);
	VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev,&vl53l0x_data);
    distance = vl53l0x_data.RangeMilliMeter;
      break;
   case 2 :
	vl53l0x_set_mode(&vl53l0x_dev2,0);
    VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev2,&vl53l0x_data2);
    distance = vl53l0x_data2.RangeMilliMeter;
      break; 
    case 3 :
	vl53l0x_set_mode(&vl53l0x_dev3,0);
    VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev3,&vl53l0x_data3);
    distance = vl53l0x_data3.RangeMilliMeter;
      break;
   default :
      printf("无效的传感器编号\n" );
   }
   return distance;

}

//返回第一次右移的秒数
int gengsui(u16 speed)
	{ 
	int i=0;
	int Distance;
	go_anywhere(0, speed);
	Distance=get_distance(1);
	
	if (Va>0) Motor_SetCCW(1, true);else  Motor_SetCCW(1, false);
	if (Vb>0) Motor_SetCCW(2, true);else  Motor_SetCCW(2, false);
	if (Vc>0) Motor_SetCCW(3, true);else  Motor_SetCCW(3, false);
	
	
		
	while(1)
		{
		if((Distance>100)&&(Distance<200))
			{   
			i++;
	    Motor_Config1(absolute(Va),absolute(Vb),absolute(Vc));
			Distance=get_distance(1);
			}
			else
				{
				printf("2");
				move_time(0, speed,400);
	      Led_On(false);
        move_time(90,speed,400);
	      Led_On(true);
        return i;
			  }
	  }
  }

//返回第二次前进的描秒数
int gengsui2(u16 speed)
	{ 
	int i=0;
	int Distance;
		
	Distance=get_distance(2);
	go_anywhere(90, speed);
	
	if (Va>0) Motor_SetCCW(1, true);else  Motor_SetCCW(1, false);
	if (Vb>0) Motor_SetCCW(2, true);else  Motor_SetCCW(2, false);
	if (Vc>0) Motor_SetCCW(3, true);else  Motor_SetCCW(3, false);
		
	while(1){
		if((Distance>50)&&(Distance<250))
			{   
			i++;
			Motor_Config1(absolute(Va),absolute(Vb),absolute(Vc));
			Distance=get_distance(2);	
			}
			else{
	      move_time(90, speed,400);
	      Led_On(false);
				move_time(180,speed,400);
        return i;
			}
	}
}


void move_ahead(u16 direction ,u8 changedirection,u16 speed,u16 distance)
{	
	int time;
	int Distance;
	int t1,t2;
	if(changedirection==1)
  {
		go_right(direction,600);
		//go_left(direction,600);
	}
	else{		
	//设置初始转动方向
	go_anywhere(direction, speed);
	
	if (Va>0) Motor_SetCCW(1, true);else  Motor_SetCCW(1, false);
	if (Vb>0) Motor_SetCCW(2, true);else  Motor_SetCCW(2, false);
	if (Vc>0) Motor_SetCCW(3, true);else  Motor_SetCCW(3, false);
		
	time=distance*12000/(78*speed);
	Distance=get_distance(1);
		
	for(;time>0;time--)
		{ 
			Motor_Config1(absolute(Va),absolute(Vb),absolute(Vc));
			Distance=get_distance(1);
			while(Distance<200)
			{ 
				Led_On(false);
				printf("%4i \r \n",vl53l0x_data.RangeMilliMeter);
		    t1=gengsui(speed);
				t2=gengsui2(speed);
				
				t1=t1+300;
				move_time(180,1000,t1);

				go_anywhere(direction, speed);
	
	     if (Va>0) Motor_SetCCW(1, true);else  Motor_SetCCW(1, false);
	     if (Vb>0) Motor_SetCCW(2, true);else  Motor_SetCCW(2, false);
	     if (Vc>0) Motor_SetCCW(3, true);else  Motor_SetCCW(3, false);
				
				time=time-t2-8;
				for(;time>0;time--)
        {
				Motor_Config1(absolute(Va),absolute(Vb),absolute(Vc));
				}
				break;
			 }
    }
	Led_On(true);
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
	Led_On(false);
	}
}

void move_ahead1(u16 direction ,u8 changedirection,u16 speed,u16 distance)
{	
	int time;
	int t1,t2;
	if(changedirection==1)
  {
		//go_right(direction,600);
		go_left(direction,600);
	}
	else{		
	//设置初始转动方向
	set_direction(direction,speed);
	
	time=100*distance*12000/(78*speed);
  //time=time/3;                 //速度固定为1200时进行的简单的校正
	for(;time>0;time--)
		{ 
			Delay_ms(10);
			test_distance();
			while(Distance_data<200)
			{ 
				Led_On(false);
				printf("%4i \r \n",Distance_data);
		    t1=gengsui(speed);
				printf("%d \r \n",t1);
				go_left(90,600);
				t2=gengsui2(speed);
				printf("%d \r \n",t2);
				t1=t1+170;
				move_time(90,1200,t1);

				Motor_Stop(1);
	      Motor_Stop(2);
	      Motor_Stop(3);
				
				go_right(90,600);
		
				time=time-t2-660;
				set_direction(90,1200);

				for(;time>0;time--)
        {
					Delay_ms(10);
				}
				Motor_Stop(1);
	      Motor_Stop(2);
	      Motor_Stop(3);
				break;
			}
    }
	Led_On(true);
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
	Led_On(false);
	}
}



void move_a(u16 direction ,u16 speed ,u16 distance)
{ 

	u32 time;
	Led_On(true);
	
	time=distance*12000/(78*speed);
	
  go_anywhere(direction, speed);
	
	if (Va>0) Motor_SetCCW(1, true);else  Motor_SetCCW(1, false);
	if (Vb>0) Motor_SetCCW(2, true);else  Motor_SetCCW(2, false);
	if (Vc>0) Motor_SetCCW(3, true);else  Motor_SetCCW(3, false);
	
	printf("%d \r\n",Va);
	printf("%d \r\n",Vb);
	printf("%d \r\n",Vc);
	
	for(;time;time--)
   { 
		 get_distance(0);
		 Motor_Config1(absolute(Va),absolute(Vb),absolute(Vc));
		 /*
		 while(Distance_data<200)
			{ 
				Led_On(false);
				printf("%4i \r \n",Distance_data);
		    t1=gengsui1();
				printf("%d \r \n",t1);
				go_left(90,600);
				t2=gengsui1();
				printf("%d \r \n",t2);
				t1=t1+170;
				move_time(90,1200,t1);

				Motor_Stop(1);
	      Motor_Stop(2);
	      Motor_Stop(3);
				
				go_right(90,600);
		
				time=time-t2-700;
				set_direction(90,1200);

				for(;time>0;time--)
        {
					Delay_ms(10);
				}
				Motor_Stop(1);
	      Motor_Stop(2);
	      Motor_Stop(3);
				break;
			}
			*/
	 }
	Motor_Stop(1);
	Motor_Stop(2);
	Motor_Stop(3);
}









