
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "control.h"
#include "ec_motor.h"
#include "ec_delay.h"


/*功能描述：把需要用到的引脚全部初始化
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：
            1、STM32 的引脚是按 A、B、C、D ... 这样分组的，每组有 16 个
            2、每组引脚都有一个独立的时钟控制，必须开启，才能控制引脚
 *修订记录：
  * Charlie @2019/02/05
 *  整理代码
 *Charlie @2019/02/2
 *  新建
 */
void T_Motor_Initialize(void)
{
    {
        //定义一个GPIO_InitTypeDef 类型的结构体
        GPIO_InitTypeDef GPIO_InitStructure;
        //开启 A 端口外设时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        //选择要控制的 GPIO 引脚
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_8;
        //设置引脚模式为输出模式
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //设置引脚速率为2MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        //调用库函数，使用上面配置的 GPIO_InitStructure 初始化GPIO
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    {
        //定义一个GPIO_InitTypeDef 类型的结构体
        GPIO_InitTypeDef GPIO_InitStructure;
        //开启 B 端口外设时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        //选择要控制的 GPIO 引脚
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        //设置引脚模式为输出模式
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //设置引脚速率为2MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        //调用库函数，使用上面配置的 GPIO_InitStructure 初始化GPIO
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }

    {
        //定义一个GPIO_InitTypeDef 类型的结构体
        GPIO_InitTypeDef GPIO_InitStructure;
        //开启 C 端口外设时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        //选择要控制的 GPIO 引脚
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
        //设置引脚模式为输出模式
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //设置引脚速率为2MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        //调用库函数，使用上面配置的 GPIO_InitStructure 初始化GPIO
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }
}


/*功能描述：延时 ( 不精确 )
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/2
 *  新建
 */
void T_Motor_Delay(volatile unsigned ms)
{
    for (; ms != 0; ms--)
        ;
}

/*功能描述：小车旋转 （轮子旋转一周，注意不是小车旋转一周）
         （这个用于测试，了解原理，在实际应用中一般不直接使用该函数）
 *参数描述：
            direction   - true:逆时针旋转 / false:顺时针旋转
            delay       - 脉冲延时（越小越快），参考值 400
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/05
 *  新建
 */
void T_Motor_Rotate(bool direction, unsigned delay)
{
    int i = 0;

    if (delay < 20)
        delay = 20;
    if (delay > 100000)
        delay = 100000;

    //电机使能
    Motor_EnableDriver(true);
    //设置电机旋转方向
    Motor_SetCCW(1, direction);
    Motor_SetCCW(2, direction);
    Motor_SetCCW(3, direction);

    //每个脉冲转 1.8 度（电机参数），360 度为 200 个脉冲
    for (i = 0; i < 200; i++)
    {
        T_Motor_Plus(1, true);
        T_Motor_Plus(2, true);
        T_Motor_Plus(3, true);
        T_Motor_Delay(delay);

        T_Motor_Plus(1, false);
        T_Motor_Plus(2, false);
        T_Motor_Plus(3, false);
        T_Motor_Delay(delay);
    }
}

/*功能描述：小车沿两个电机中间方向移动 ( 轮子转动 1 周 )
         （这个用于测试，了解原理，在实际应用中一般不直接使用该函数）
 *参数描述：
            idx_a   - 左边电机 ( 取值 1，2，3 )
            idx_b   - 右边电机 ( 取值 1，2，3  - 不能与 idx_a 相同)
            direction - true : 前进 / false : 后退 
            delay     - 脉冲延时（越小越快），参考值 400
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/05
 *  新建
 */
void T_Motor_Move(unsigned idx_a, unsigned idx_b, bool direction, unsigned delay)
{
    int i = 0;

    if (idx_a == idx_b)
        return;

    if (delay < 20)
        delay = 20;
    if (delay > 100000)
        delay = 100000;

    //电机使能
    Motor_EnableDriver(true);
    //设置电机旋转方向
    Motor_SetCCW(idx_a, direction);
    Motor_SetCCW(idx_b, !direction);

    //每个脉冲转 1.8 度（注意：参考电机参数 - 有可能不对），360 度为 200 个脉冲
    for (i = 0; i < 200; i++)
    {
        T_Motor_Plus(idx_a, true);
        T_Motor_Plus(idx_b, true);
        T_Motor_Delay(delay);

        T_Motor_Plus(idx_a, false);
        T_Motor_Plus(idx_b, false);
        T_Motor_Delay(delay);
    }
}

/*功能描述：电机驱动 （这个用于测试，了解原理，在实际应用中一般不直接使用该函数）
 *参数描述：
            idx     - 电机序号（俯视机器，做了标志的为 1 号电机，左边是 2 号电机，右边是 3 号电机）
            level   - 电平 （ true - 高电平 / false - 低电平 ） 
 *返回信息：无
 *注意事项：无
 *知识参考：参考原理图，A8 / A0 / A6 分别驱动 3 个电机 （ 电机本身是靠 LV8731 驱动的 ）
 *修订记录：
 *Charlie @2019/02/05
 *  新建
 */
void T_Motor_Plus(unsigned idx, bool level)
{
    switch (idx)
    {
    case 1:
        if (level)
            GPIO_SetBits(GPIOA, GPIO_Pin_8);
        else
            GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        break;
    case 2:
        if (level)
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
        else
            GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        break;
    case 3:
        if (level)
            GPIO_SetBits(GPIOA, GPIO_Pin_6);
        else
            GPIO_ResetBits(GPIOA, GPIO_Pin_6);
        break;
    default:
        break;
    }
}

/*功能描述：1、把需要用到的引脚全部初始化  2、定时器初始化
        IO 模拟版本只适合于了解原理，而定时器版本可以用在产品中。
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：
            1、STM32 的引脚是按 A、B、C、D ... 这样分组的，每组有 16 个
            2、每组引脚都有一个独立的时钟控制，必须开启，才能控制引脚
            3、需要用到 3 个定时器，分别输出 3 个驱动信号 （ A8、A0、A6）
               其中 A8 是 TIM1_CH1 ( 1 号电机 )
               其中 A0 是 TIM2_CH1_ETR （ 2 号电机 ）
               其中 A6 是 TIM3_CH1 ( 3 号电机 )
            4、C4、B0、C5 分别控制 1，2，3 号电机的旋转方向
            5、C3 控制电机驱动芯片的使能
 *修订记录：
  * Charlie @2019/02/05
 *  新建
 */
void Motor_Initialize(void)
{
    {
        //定义一个GPIO_InitTypeDef 类型的结构体
        GPIO_InitTypeDef GPIO_InitStructure;
        //开启 B 端口外设时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        //选择要控制的 GPIO 引脚
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        //设置引脚模式为输出模式
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //设置引脚速率为2MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        //调用库函数，使用上面配置的 GPIO_InitStructure 初始化GPIO
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }

    {
        //定义一个GPIO_InitTypeDef 类型的结构体
        GPIO_InitTypeDef GPIO_InitStructure;
        //开启 C 端口外设时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        //选择要控制的 GPIO 引脚
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
        //设置引脚模式为输出模式
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        //设置引脚速率为2MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        //调用库函数，使用上面配置的 GPIO_InitStructure 初始化GPIO
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }
}

/*功能描述：控制电机驱动芯片（ 使能 或 禁用）， 3 个电机的芯片是独立的，但目前测试用的实验板是连接在一起的，
           所以 3 个电机的驱动芯片要么同时有效，要么同时无效。
 *参数描述：
            bEnable - true: 使能驱动芯片  / false: 关闭驱动芯片
 *返回信息：无
 *注意事项：无
 *知识参考：电路图和 《三洋步进驱动芯片LV8731.pdf》
 *修订记录：
 *Charlie @2019/02/05
 *  新建
 */
void Motor_EnableDriver(bool bEnable)
{
    if (bEnable)
        GPIO_SetBits(GPIOC, GPIO_Pin_3);
    else
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);
}

/*功能描述： 根据电机编号，返回控制电机的定时器
 *参数描述：
            idx - 电机编号
 *返回信息：定时器
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/07
 *  新建
 */
TIM_TypeDef *GetTIM_Index( uint8_t idx )
{
    if (idx == 1)
        return TIM1;
    else if (idx == 2)
        return TIM2;
    else if (idx == 3)
        return TIM3;
    else
        return 0;
}

/*功能描述： 配置电机参数，配置后电机不会立即启动
 *参数描述：
            idx - 电机编号
            steps - 每秒电机走多少步
            start - true - 立即启动 / false - 不启动（后面调用 Motor_Start)
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/07
 *  新建
 */
void Motor_Config(uint8_t idx, uint16_t steps, bool start)
{
    RCC_ClocksTypeDef RCC_ClocksTypeStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    TIM_TypeDef* tim = GetTIM_Index( idx );
    if( tim == 0 )
        return;

    //开启定时器时钟
    if (idx == 1)
        //1 号电机采用定时器 1 控制，输出端口是 A8，挂载在 APB2 总线上
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    else if (idx == 2)
        //2 号电机采用定时器 2 控制，输出端口是 A0,挂载在 APB1 总线上
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    else if (idx == 3)
        //3 号电机采用定时器 3 控制，输出端口是 A6，挂载在 APB1 总线上
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    //获取时钟频率
    RCC_GetClocksFreq(&RCC_ClocksTypeStructure);
    //分频和定时周期配置
    TIM_TimeBaseStructure.TIM_Prescaler = 500 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = RCC_ClocksTypeStructure.SYSCLK_Frequency / (500 * steps) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

    //配置为PWM 模式1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = (TIM_TimeBaseStructure.TIM_Period >> 1) - 1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(tim, &TIM_OCInitStructure);

    //禁止 ARR 寄存器预装载 ( 这样改变 ARR 寄存器值可以立即生效 )
    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Disable);

    //自动输出使能，断路、死区时间和锁定配置
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_DeadTime = 0;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRConfig(tim, &TIM_BDTRInitStructure);

    //主动输出使能
    TIM_CtrlPWMOutputs(tim, ENABLE);

    //开启端口时钟 ( 3 个定时器采用的端口都是 A )
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //配置输出端口为复用 - 推挽输出
    if (idx == 1)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    else if (idx == 2)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    else if (idx == 3)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    if( start )
        TIM_Cmd( tim, ENABLE);
}


//分别产生控制三个轮子的pwm波，
void Motor_Config1( uint16_t Va, uint16_t Vb,uint16_t Vc )
{   
    RCC_ClocksTypeDef RCC_ClocksTypeStructure1;
    GPIO_InitTypeDef GPIO_InitStructure1;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure1;
    TIM_OCInitTypeDef TIM_OCInitStructure1;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure1;
	
	  RCC_ClocksTypeDef RCC_ClocksTypeStructure2;
    GPIO_InitTypeDef GPIO_InitStructure2;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;
    TIM_OCInitTypeDef TIM_OCInitStructure2;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure2;
	
	  RCC_ClocksTypeDef RCC_ClocksTypeStructure3;
    GPIO_InitTypeDef GPIO_InitStructure3;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3;
    TIM_OCInitTypeDef TIM_OCInitStructure3;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure3;

   TIM_TypeDef* tim1 = GetTIM_Index( 1 );
	 TIM_TypeDef* tim2 = GetTIM_Index( 2 );
	 TIM_TypeDef* tim3 = GetTIM_Index( 3 );
   // if( tim == 0 )
   
	
    
    //开启定时器时钟
    //if (idx == 1)
        //1 号电机采用定时器 1 控制，输出端口是 A8，挂载在 APB2 总线上
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //else if (idx == 2)
        //2 号电机采用定时器 2 控制，输出端口是 A0,挂载在 APB1 总线上
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //else if (idx == 3)
        //3 号电机采用定时器 3 控制，输出端口是 A6，挂载在 APB1 总线上
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    //获取时钟频率
    RCC_GetClocksFreq(&RCC_ClocksTypeStructure1);
    //分频和定时周期配置
    TIM_TimeBaseStructure1.TIM_Prescaler = 500 - 1;
    TIM_TimeBaseStructure1.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure1.TIM_Period = RCC_ClocksTypeStructure1.SYSCLK_Frequency / (500 * Va) - 1;
    TIM_TimeBaseStructure1.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure1.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim1, &TIM_TimeBaseStructure1);
		
		RCC_GetClocksFreq(&RCC_ClocksTypeStructure2);
    //分频和定时周期配置
    TIM_TimeBaseStructure2.TIM_Prescaler = 500 - 1;
    TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure2.TIM_Period = RCC_ClocksTypeStructure2.SYSCLK_Frequency / (500 * Vb) - 1;
    TIM_TimeBaseStructure2.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure2.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim2, &TIM_TimeBaseStructure2);
		
		 RCC_GetClocksFreq(&RCC_ClocksTypeStructure3);
    //分频和定时周期配置
    TIM_TimeBaseStructure3.TIM_Prescaler = 500 - 1;
    TIM_TimeBaseStructure3.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure3.TIM_Period = RCC_ClocksTypeStructure3.SYSCLK_Frequency / (500 * Vc) - 1;
    TIM_TimeBaseStructure3.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim3, &TIM_TimeBaseStructure3);
		

    //配置为PWM 模式1
    TIM_OCInitStructure1.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure1.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure1.TIM_OutputNState = TIM_OutputState_Disable;
    TIM_OCInitStructure1.TIM_Pulse = (TIM_TimeBaseStructure1.TIM_Period >> 1) - 1;
    TIM_OCInitStructure1.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure1.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure1.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure1.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(tim1, &TIM_OCInitStructure1);
		
		TIM_OCInitStructure2.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure2.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure2.TIM_OutputNState = TIM_OutputState_Disable;
    TIM_OCInitStructure2.TIM_Pulse = (TIM_TimeBaseStructure2.TIM_Period >> 1) - 1;
    TIM_OCInitStructure2.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure2.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure2.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure2.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(tim2, &TIM_OCInitStructure2);
		
		
		TIM_OCInitStructure3.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure3.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure3.TIM_OutputNState = TIM_OutputState_Disable;
    TIM_OCInitStructure3.TIM_Pulse = (TIM_TimeBaseStructure3.TIM_Period >> 1) - 1;
    TIM_OCInitStructure3.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure3.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure3.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure3.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(tim3, &TIM_OCInitStructure3);
		

    //禁止 ARR 寄存器预装载 ( 这样改变 ARR 寄存器值可以立即生效 )
    TIM_OC1PreloadConfig(tim1, TIM_OCPreload_Disable);
		TIM_OC1PreloadConfig(tim2, TIM_OCPreload_Disable);
		TIM_OC1PreloadConfig(tim2, TIM_OCPreload_Disable);

    //自动输出使能，断路、死区时间和锁定配置
    TIM_BDTRInitStructure1.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRInitStructure1.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure1.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure1.TIM_DeadTime = 0;
    TIM_BDTRInitStructure1.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure1.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStructure1.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRConfig(tim1, &TIM_BDTRInitStructure1);
		
		
		TIM_BDTRInitStructure2.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRInitStructure2.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure2.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure2.TIM_DeadTime = 0;
    TIM_BDTRInitStructure2.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure2.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStructure2.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRConfig(tim2, &TIM_BDTRInitStructure2);
		
		
		TIM_BDTRInitStructure3.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRInitStructure3.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure3.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure3.TIM_DeadTime = 0;
    TIM_BDTRInitStructure3.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure3.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStructure3.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRConfig(tim3, &TIM_BDTRInitStructure3);
		
   
   if(Va!=0){TIM_CtrlPWMOutputs(tim1, ENABLE);}
	 if(Vb!=0){TIM_CtrlPWMOutputs(tim2, ENABLE);}
	 if(Vc!=0){TIM_CtrlPWMOutputs(tim3, ENABLE);}	 
    //主动输出使能
//	 
//    TIM_CtrlPWMOutputs(tim1, ENABLE);
//		TIM_CtrlPWMOutputs(tim2, ENABLE);
//		TIM_CtrlPWMOutputs(tim3, ENABLE);

    //开启端口时钟 ( 3 个定时器采用的端口都是 A )
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    //配置输出端口为复用 - 推挽输出
    //if (idx == 1)
        GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_8;
    //else if (idx == 2)
        GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0;
   // else if (idx == 3)
        GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_6;

    GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure1);
		
		GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure2);
		
		GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure3);
		
	 if( Va>0 )
      TIM_Cmd( tim1, ENABLE);
	 if( Vb>0 )
     TIM_Cmd( tim2, ENABLE);
	 if( Vc>0 )
      TIM_Cmd( tim3, ENABLE);
	 
	  Delay_ms(1000);
	 
		TIM_CtrlPWMOutputs(tim1, DISABLE);
		TIM_CtrlPWMOutputs(tim2, DISABLE);
		TIM_CtrlPWMOutputs(tim3, DISABLE);
}



/*功能描述：启动电机
 *参数描述：
            idx: 电机编号
            enable: true - 允许计数  / false - 禁止计数
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/07
 *  新建
 */
void Motor_Start(uint8_t idx)
{
    TIM_TypeDef* tim = GetTIM_Index( idx );
    if( tim == 0 )
        return;

    TIM_Cmd( tim, ENABLE);
}

/*功能描述：停止电机
 *参数描述：
            idx: 电机编号
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/07
 *  新建
 */
void Motor_Stop(uint8_t idx)
{
    TIM_TypeDef* tim = GetTIM_Index( idx );
    if( tim == 0 )
        return;

    TIM_Cmd( tim, DISABLE);
}

/*功能描述：改变电机的转速
 *参数描述：
            idx - 电机编号
		    steps - 每秒多少步（电机），电机有自己的极限速度，不能过高（具体值需要参考电机参数）
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/07
 *  新建
 */
void Motor_Speed(uint8_t idx, uint16_t steps)
{
    RCC_ClocksTypeDef RCC_ClocksTypeStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_TypeDef* tim = GetTIM_Index( idx );
    if( tim == 0 )
        return;

    //获取时钟频率
    RCC_GetClocksFreq(&RCC_ClocksTypeStructure);

    //保证参数正确
    if (steps < 2)
        return;
    if (500 * steps >= RCC_ClocksTypeStructure.SYSCLK_Frequency >> 2)
        return;

    Motor_Stop(idx);

    //分频和定时周期配置
    TIM_TimeBaseStructure.TIM_Prescaler = 500 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = RCC_ClocksTypeStructure.SYSCLK_Frequency / (500 * steps) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    //配置为PWM 模式1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = (TIM_TimeBaseStructure.TIM_Period >> 1) - 1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
    TIM_OC1Init(tim, &TIM_OCInitStructure);

    Motor_Start(idx);
}


/*功能描述：设置电机反转或正转
 *参数描述：
            idx - 电机序号（机器朝北，俯视机器，北方是 1 号电机，左边是 2 号电机，右边是 3 号电机）
            ccw - true - 反转 / false - 正转
 *返回信息：无
 *注意事项：无
 *知识参考：参考原理图，C4 / B0 / C5 分别控制正转还是反转
 *修订记录：
 *Charlie @2019/02/05
 *  新建
 */
void Motor_SetCCW(uint8_t idx, bool ccw)
{
    switch (idx)
    {
    case 1:
        if (ccw)
            GPIO_SetBits(GPIOC, GPIO_Pin_4);
        else
            GPIO_ResetBits(GPIOC, GPIO_Pin_4);
        break;
    case 2:
        if (ccw)
            GPIO_SetBits(GPIOB, GPIO_Pin_0);
        else
            GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        break;
    case 3:
        if (ccw)
            GPIO_SetBits(GPIOC, GPIO_Pin_5);
        else
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);
        break;
    default:
        break;
    }
}


/*功能描述：获取当前电机的转向
 *参数描述：
            idx - 电机序号（机器朝北，俯视机器，北方是 1 号电机，左边是 2 号电机，右边是 3 号电机）
 *返回信息：反转时返回 true，否则返回 false
 *注意事项：无
 *知识参考：参考原理图，C4 / B0 / C5 分别控制正转还是反转
 *修订记录：
 *Charlie @2019/02/05
 *  新建
 */
bool Motor_IsCCW( uint8_t idx )
{
    if( idx == 1 )
        return  ( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_4 ) != 0 );
    else if( idx == 2 )
        return  ( GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_0 ) != 0 );
    else if( idx == 3 )
        return  ( GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_5 ) != 0 );
    else
        return false;
}
