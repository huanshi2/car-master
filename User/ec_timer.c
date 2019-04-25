
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#include "ec_timer.h"
#include "ec_led.h"



/*功能描述：中断初始化
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/05
 *  新建
 */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //设置中断组为 0
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    //设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    //设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*功能描述：定时器 2 初始化
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：定时器从 0 开始计数，到 TIM_Period 就产生一个中断
 *修订记录：
 * Charlie @2019/02/05
 *  新建
 */
void TIM2_Mode_Config(void)
{
    RCC_ClocksTypeDef RCC_ClocksTypeStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    //开启定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    //获取时钟频率
    RCC_GetClocksFreq(&RCC_ClocksTypeStructure);

    //时钟分频 （ X + 1 )
    TIM_TimeBaseStructure.TIM_Prescaler = 5000 - 1;
    //计数值（从 0 开始计数到这个数，则中断 1 次）
    TIM_TimeBaseStructure.TIM_Period = RCC_ClocksTypeStructure.SYSCLK_Frequency / 10000 - 1;
    //初始化定时器TIMx, x[2,3,4,5]
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    //清除定时器更新中断标志位
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    //开启定时器更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    //使能定时器
    TIM_Cmd(TIM2, ENABLE);
}

/*功能描述：定时器 2 的中断响应函数
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 * Charlie @2019/02/05
 *  新建
 */
void TIM2_IRQHandler(void)
{
    static bool on = false;
    //清除中断标志
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    //取反让灯闪烁
    on = !on;
    Led_On(on);
}
