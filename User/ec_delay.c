
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "ec_delay.h"

static uint8_t  fac_us=0;
static uint16_t fac_ms=0;


/*功能描述：初始化延时函数相关参数
 *参数描述：无
 *返回信息：无
 *注意事项：无
 *知识参考：https://blog.csdn.net/ABAP_Brave/article/details/52370293
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void Delay_Initialize( void )
{
    //选择时钟源-外部时钟-HCLK/8
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
    //72/8 延时1微秒9个时钟周期
    fac_us=SystemCoreClock/8000000;
    //延时1毫秒9000个Cystic时钟周期 
    fac_ms=( uint16_t )fac_us*1000;
}


/*功能描述：微秒级延时
 *参数描述：
            nus - 微秒数
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void Delay_us(uint32_t nus)
{
    u32 temp;

    //设置重载值:n(us)*延时1us需要多少个SysTick时钟周期 （ 最大不能超过SysTick->LOAD(24位)-1 ）
    SysTick->LOAD=nus*fac_us;
    //VAL初始化为 0
    SysTick->VAL=0x00;
    //使能SysTick定时器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 
    
    //等待计数时间到达(位16)
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16))); 
    
    //关闭使能，重置 VAL
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; 
    SysTick->VAL =0X00;
}

/*功能描述：毫秒级延时
 *参数描述：
            nms - 毫秒数
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void Delay_ms(u16 nms)
{
    u32 temp;
    SysTick->LOAD=(u32)nms*fac_ms;
    SysTick->VAL =0x00;
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL =0X00;
}


/*功能描述：毫秒级延时 ( 为移植 MPU 库增设的 )
 *参数描述：
            num_ms - 毫秒数
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  为移植 DMP 库而定义的
 */
void MPU6050_delay_ms(unsigned long num_ms)
{
    Delay_ms( num_ms );
}

/*功能描述：
 *参数描述：
            count - 
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  为移植 DMP 库而定义的
 */
void MPU6050_get_ms(unsigned long *count)
{

}
