
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "ec_usart.h"

//接收状态标记
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA = 0;
//接收缓冲,最大USART_REC_LEN个字节.
uint8_t USART_RX_BUF[USART_REC_LEN];

//加入以下代码,支持 printf 函数,而不需要选择 use MicroLIB
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
//参考知识库：http://www.stmcu.org.cn/module/forum/thread-598294-1-1.html
_sys_exit(int x)
{
    x = x;
}

/*功能描述：重定义fputc函数
 *参数描述：
            ch - 字符
            f - 文件描述结构体
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
int fputc(int ch, FILE *f)
{
    //循环发送,直到发送完毕
    while ((USART1->SR & 0X40) == 0)
        ;
    USART1->DR = (u8)ch;
    return ch;
}

/*功能描述：串口 1 初始化
 *参数描述：
            bound - 波特率，比如 9600
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void Uart1_Initialize(uint32_t bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //开启 USART1 、GPIOA 的时钟 ( USART1 TX --> PA.9 / RX --> PA.10)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    //引脚配置 （TX -- PA.09）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //引脚配置 （RX  -- PA.10）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //( 浮空输入)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1 NVIC 配置 （ 抢占优先级 3 / 子优先级 3 / IRQ通道使能 )
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //USART1 初始化设置 ( 波特率、字长 8 、1个停止位、无奇偶校验、无硬件数据流控制、双工 )
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    //开启中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //使能串口
    USART_Cmd(USART1, ENABLE);
}

/*功能描述：串口1 中断服务程序
 *参数描述：无
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void USART1_IRQHandler(void)
{
    uint8_t Res;

    //接收中断 ( 自定义协议要求接收的数据必须是 0x0d 0x0a 结尾 )
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        //读取数据 ( USART1->DR )
        Res = USART_ReceiveData(USART1);
        //接收未完成
        if ((USART_RX_STA & 0x8000) == 0)
        {
            //接收到了0x0d
            if (USART_RX_STA & 0x4000)
            {
                if (Res != 0x0a)
                    USART_RX_STA = 0; //接收错误,重新开始
                else
                    USART_RX_STA |= 0x8000; //接收完成了
            }
            else //还没收到0X0D
            {
                if (Res == 0x0d)
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0; //接收数据错误,重新开始接收
                }
            }
        }
    }
}
