#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"
#include "system_stm32f10x.h"
#include "ec_led.h"
#include "ec_motor.h"
#include "ec_timer.h"
#include "ec_key.h"
#include "ec_usart.h"
#include "ec_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "vl53l0x.h"
#include "vl53l0x_gen.h"
#include "stm32f10x_exti.h"
#include <math.h>
#include "stmflash.h"
#include "stack.h"
#include "control.h"
#include "sys.h"
#include "exti.h"



//要写入STM32 FLASH的字符串数组
const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};

#define SIZE sizeof(TEXT_Buffer)	 	//数组长度
#define FLASH_SAVE_ADDR  0X08020000 	
//设置FLASH保存地址（必须为偶数，且其值要大与本代码所占用的大小+0X08000000) 0x20000就是128k


/******************************************************************************/
extern VL53L0X_Dev_t vl53l0x_dev,vl53l0x_dev2,vl53l0x_dev3;//设备I2C数据参数
/******************************************************************************/

int main(void)
{
	
	u8 mode=0;
	int d1,d2,d3;
  int t=0;
	u8 datatemp[SIZE];
	
	Stack* directionVec = CreateStack();
  Stack* changedirectionVec=CreateStack();
	Stack* speed=CreateStack();
	Stack* distanceVec=CreateStack();
	
	
	Delay_Initialize();               //=====延时初始化
	Uart1_Initialize(9600);
	Led_Initialize();
	IIC_Init();                     //=====IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP     
	Delay_ms(1000);
	Motor_Initialize();             //电机初始化
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//分组2
	Delay_ms(200);
	

	if(vl53l0x_init(&vl53l0x_dev))     //vl53l0x初始化
	 {
		printf("VL53L0X_Init Error!!!\r\n");
		Delay_ms(200);
	 }
	 else
		 printf("VL53L0X_Init OK\r\n");
	
	if(vl53l0x_set_mode(&vl53l0x_dev,mode))   //配置测量模式
		{
			printf("Mode Set Error!!!\r\n");
		}
	else
		printf("Mode Set OK!!!\r\n");
	
	One_measurement(mode);
	
	
	if(vl53l0x_init_2(&vl53l0x_dev2))     //第二个vl53l0x初始化
	 {
		printf("VL53L0X_Init Error!!!\r\n");
		Delay_ms(200);
	 }
	 else
		 printf("VL53L0X_Init OK\r\n");
	
	if(vl53l0x_set_mode(&vl53l0x_dev2,mode))   //配置测量模式
		{
			printf("Mode Set Error!!!\r\n");
		}
	else
		printf("Mode1 Set OK!!!\r\n");
	
		One_measurement2(mode);

	/*
	Delay_ms(1000);	
	
	
	if(vl53l0x_init_3(&vl53l0x_dev3))     //第三个vl53l0x初始化
	 {
		printf("VL53L0X_Init Error!!!\r\n");
		Delay_ms(200);
	 }
	 else
		 printf("VL53L0X_Init OK\r\n");
	
	if(vl53l0x_set_mode(&vl53l0x_dev3,mode))   //配置测量模式
		{
			printf("Mode Set Error!!!\r\n");
		}
	else
		printf("Mode2 Set OK!!!\r\n");
	
		One_measurement3(mode);
	
	*/
	vl53l0x_info();
	vl53l0x_info2();
	//vl53l0x_info3();

	Motor_EnableDriver(true);
	
	
	printf("1\r\n");
	Delay_ms(1000);                 //=====延时等待初始化稳定
  //EXTI_init();                   //=====MPU6050 5ms定时中断初始化
	printf("2\r\n");
  
	
	Motor_EnableDriver(true);

  //move_ahead(90,0,1000,60);
	
	
		Delay_ms(500);
	One_measurement(mode);
	One_measurement2(mode);
	
	Read_DMP();
	Delay_ms(1000);
	while(1)
		{
	  Read_DMP();
		printf("%8d ,%8d ,%8d \r\n",accel[0],accel[1],accel[2]);
		printf("%8d ,%8d ,%8d \r\n",gyro[0],gyro[1],gyro[2]);
    printf("%f ,%f ,%f \r\n",Pitch,Roll,Yaw);
	  } 	
	

//  move_ahead(90,0,1200,150);
//	move_ahead(90,1,600,110);
//	move_ahead(90,0,1200,50);
//	
//  //move_ahead(90,1,600,210);
//	//move_ahead(90,0,1200,220);
//	
//  PushStack(directionVec,90); 
//  PushStack(directionVec,90);
//  PushStack(directionVec,90);
//	//PushStack(directionVec,90);
//  //PushStack(directionVec,90);
//	
//	PushStack(changedirectionVec,0); 
//	PushStack(changedirectionVec,1); 
//	PushStack(changedirectionVec,0); 
//	//PushStack(changedirectionVec,1);
//	//PushStack(changedirectionVec,0);
//	
//	PushStack(speed,1200); 
//	PushStack(speed,600); 
//	PushStack(speed,1200);
//	//PushStack(speed,600); 
//	//PushStack(speed,1200);
//	
//	PushStack(distanceVec,150); 
//	PushStack(distanceVec,110); 
//	PushStack(distanceVec,50);
//	//PushStack(distanceVec,210); 
//	//PushStack(distanceVec,220); 
//	
//	go_right(180,600);
//  
// for(;t<3;t++)
//  { 
//		printf("33");
//		move_ahead1(GetTopElement(directionVec),GetTopElement(changedirectionVec),GetTopElement(speed),GetTopElement(distanceVec));
//	  
//		printf("%d \r\n",GetTopElement(directionVec));
//		printf("%d \r\n",GetTopElement(changedirectionVec));
//		printf("%d \r\n",GetTopElement(speed));
//		printf("%d \r\n",GetTopElement(distanceVec));
//		
//		PopStack(directionVec);
//		PopStack(changedirectionVec);
//		PopStack(speed);
//		PopStack(distanceVec);
//	}
// 
  
	
	//DestoryStack(directionVec);
  //DestoryStack(changedirectionVec);
  //DestoryStack(speed);
	//DestoryStack(distanceVec);
	
	//flash测试
	//STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)TEXT_Buffer,SIZE);
	//Led_On(false);
	//STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,SIZE);
	//printf("%s",datatemp);

}

