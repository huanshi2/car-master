
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "ec_key.h"

/*功能描述：初始化键盘使用的端口
 *参数描述：无 
 *返回信息：无
 *注意事项：无
 *知识参考：
            1、S1 Key 连接的端口是 PortB.14
 *修订记录：
 *   Charlie @2019/02/07
 *   新建
 */
void Key_Initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure);

    //开启端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

/*功能描述：检查按键是否按下 (没有处理抖动等问题)
 *参数描述：无 
 *返回信息：按下返回 true，否则返回 false
 *注意事项：无 
 *知识参考：
            1、S1 Key 连接的端口是 PortB.14
            2、如果键盘按下，PortB.14 是低电平，释放后就是高电平
            3、端口配置成输入模式
 *修订记录：
 *   Charlie @2019/02/07
 *   新建
 */
bool is_key_down( void )
{
    uint8_t key = GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_14 );
    return (key == 0);
}
