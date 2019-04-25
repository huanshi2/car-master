#include    "stm32f10x_gpio.h"
#include    "stm32f10x_rcc.h"
#include    "ec_led.h"


/*功能描述：LED 灯管脚初始化
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/01/31
 *  新建
 */
void    Led_Initialize( void )
{
    //定义一个GPIO_InitTypeDef 类型的结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    //开启 LED 相关的 GPIO 外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    //选择要控制的 GPIO 引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    //设置引脚模式为输出模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //设置引脚速率为2MHz
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    //调用库函数，使用上面配置的 GPIO_InitStructure 初始化GPIO
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*功能描述：点亮或熄灭 LED 灯
 *参数描述：
            on  true: 亮  false：灭
 *返回信息：无
 *注意事项：无
 *知识参考：参考电路图可知，管脚输出低电平 LED 亮
 *修订记录：
 *Charlie @2019/02/2
 *  新建
 */
void    Led_On( bool on )
{
    if( !on )
        GPIO_SetBits(GPIOB, GPIO_Pin_13 );
    else
        GPIO_ResetBits(GPIOB, GPIO_Pin_13 );
}
