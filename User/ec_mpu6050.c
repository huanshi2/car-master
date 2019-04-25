
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "ec_mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"

#define q30  1073741824.0f

//使用的 I2C 总线
I2C_TypeDef *i2c = I2C2;


//原工程代码摘抄（意义待确定）
static signed char gyro_orientation[9] = {-1, 0, 0,
                                          0, -1, 0,
                                          0, 0, 1};


/*功能描述：I2C 初始化 - MPU6050 挂载在 I2C(2#)总线上
 *参数描述：无
 *返回信息：无
 *注意事项：无
 *知识参考：I2C_OwnAddress1 是 STM32 自身的地址,只要不与外设冲突即可。
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
																					
/*
void I2C_Initialize(void)
{
    I2C_InitTypeDef i2c_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    //开启端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    //开启 I2C 总线的时钟( I2C(2#)挂在 APB1 总线上 )
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    //初始化 I2C 用到的 GPIO 口 (PB.10 / PB.11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //初始化 I2C
    i2c_InitStructure.I2C_Ack = I2C_Ack_Enable;
    i2c_InitStructure.I2C_ClockSpeed = 400000;
    i2c_InitStructure.I2C_Mode = I2C_Mode_I2C;
    i2c_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_InitStructure.I2C_OwnAddress1 = 0x11;
    i2c_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &i2c_InitStructure);

    //使能 I2C
    I2C_Cmd(I2C2, ENABLE);
}
*/

/*功能描述：初始化 MPU6050
 *参数描述：无
 *返回信息：无
 *注意事项： 0xD0 表示 MPU6050 的地址。我们知道 I2C从器件(在此当然是指 MPU6050)有 8 位的地址,
            前 7 位由 WHO AM I 确定,第 8 位由 AD0 的电平决定。WHO AM I 默认值是 0x68H(1101000B),
            AD0 接低电平,所以 MPU6050 的 I2C 地址是 0xD0H(11010000B)。
 *知识参考：http://m.elecfans.com/article/600674.html
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void MPU6050_Initialize()
{   
	  printf("3");
    //唤醒, 8M 内部时钟源
    MPU6050_I2C_ByteWrite(devAddr, 0x00, MPU6050_RA_PWR_MGMT_1);
	  printf("4");
    //采用频率 1000
    MPU6050_I2C_ByteWrite(devAddr, 0x07, MPU6050_RA_SMPLRT_DIV);
    MPU6050_I2C_ByteWrite(devAddr, 0x06, MPU6050_RA_CONFIG);
    //加速度量程 2g
    MPU6050_I2C_ByteWrite(devAddr, 0x01, MPU6050_RA_ACCEL_CONFIG);
    //角速度量程 2000度/s
    MPU6050_I2C_ByteWrite(devAddr, 0x18, MPU6050_RA_GYRO_CONFIG);
}




/*功能描述：向 MPU6050 写数据
 *参数描述：
            slaveAddr - 设备地址
            pBuffer - 需要写入的值
            writeAddr - 写入的寄存器地址
 *返回信息：如果成功返回 0，否则返回非零值
 *注意事项： 0xD0 表示 MPU6050 的地址。我们知道 I2C从器件(在此当然是指 MPU6050)有 8 位的地址,
            前 7 位由 WHO AM I 确定,第 8 位由 AD0 的电平决定。WHO AM I 默认值是 0x68H(1101000B),
            AD0 接低电平,所以 MPU6050 的 I2C 地址是 0xD0H(11010000B)。
 *知识参考：http://m.elecfans.com/article/600674.html
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
int MPU6050_I2C_ByteWrite(u8 slaveAddr, u8 pBuffer, u8 writeAddr)
{
    //发送开始信号
    I2C_GenerateSTART(i2c, ENABLE);
    //检查 EV5 事件(参考 I2C 总线描述)
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    //向 MPU6050 发送地址(写)
    I2C_Send7bitAddress(i2c, slaveAddr, I2C_Direction_Transmitter);
    //检查 EV6 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;
    //向 MPU6050 的发送即将写的内部寄存器地址
    I2C_SendData(i2c, writeAddr);
    //检查 EV8 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
    //发送要写的内容
    I2C_SendData(i2c, pBuffer);
    //检查 EV8 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
    //发送结束信号
    I2C_GenerateSTOP(i2c, ENABLE);

    return 0;
}

/*功能描述：向 MPU6050 写多字节数据
 *参数描述：
            slave_addr  - 设备地址
            reg_addr    - 读取的寄存器地址
            length      - 待写入的长度
            data        - 需要写入的数据
 *返回信息： 如果成功返回 0，否则返回非零值
 *注意事项： 无
 *知识参考：http://m.elecfans.com/article/600674.html
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
int MPU6050_I2C_BufferWrite(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    int i = 0;
    for (i = 0; i < length; i++)
    {
        MPU6050_I2C_ByteWrite(slave_addr, data[i], reg_addr + i);
    }
    return 0;
}

/*功能描述：从 MPU6050 读数据
 *参数描述：
            slave_addr - 设备地址
            reg_addr - 读取的寄存器地址
            length - 待读取长度
            data - 读取的值放到哪里
 *返回信息： 如果成功返回 0，否则返回非零值
 *注意事项： 0xD0 表示 MPU6050 的地址。我们知道 I2C从器件(在此当然是指 MPU6050)有 8 位的地址,
            前 7 位由 WHO AM I 确定,第 8 位由 AD0 的电平决定。WHO AM I 默认值是 0x68H(1101000B),
            AD0 接低电平,所以 MPU6050 的 I2C 地址是 0xD0H(11010000B)。
 *知识参考：http://m.elecfans.com/article/600674.html
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
int MPU6050_I2C_BufferRead(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    //等待总线空闲
    while (I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY))
        ;
    //发送开始信号
    I2C_GenerateSTART(i2c, ENABLE);
    //检查 EV5 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    //发送地址（写）
    I2C_Send7bitAddress(i2c, slave_addr, I2C_Direction_Transmitter);
    //检查 EV6 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;
    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(i2c, ENABLE);
    //向芯片发送待读取的寄存器地址（ 这个动作是写 ）
    I2C_SendData(i2c, reg_addr);
    //检查 EV8 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
    //发送重复起始信号
    I2C_GenerateSTART(i2c, ENABLE);
    //检查 EV5 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    //发送地址 （ 读 ）
    I2C_Send7bitAddress(i2c, slave_addr, I2C_Direction_Receiver);
    //检查 EV6 事件
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        ;
    //读取数据
    while (length > 0)
    {
        if (length == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(i2c, DISABLE);
            //发送停止信号
            I2C_GenerateSTOP(i2c, ENABLE);
        }

        //检查 EV7 事件
        if (I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            //读数据
            *data = I2C_ReceiveData(i2c);
            data++;

            length--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(i2c, ENABLE);

    return 0;
}


void MPU6050_GetRawAccelGyro(s16* AccelGyro) //读加速度值 和 角速度值
{
	
}


unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        printf("setting bias succesfully ......\r\n");
    }
}

/*功能描述：读取 ID
 *参数描述：无
 *返回信息：成功的话，应该返回 0x68
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
uint8_t MPU6050_WhoAmI(void)
{
    u8 buffer[2] = {0};
    MPU6050_I2C_BufferRead(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/*功能描述：读取温度
 *参数描述：无
 *返回信息：返回摄氏温度值
 *注意事项：测试中，温度变化很大（不确定是否正确）
 *知识参考：温度保存在 0x41 0x42 中，两个字节，0x41 保存高字节。
        温度换算公式参考 MPU6050 的数据手册
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */
void MPU6050_ReadTemperature(float *value)
{
    short temp = 0;
    u8 buf[2] = {0};
    MPU6050_I2C_BufferRead(devAddr, MPU6050_RA_TEMP_OUT_H, 2, buf);
    temp = (buf[0] << 8) | buf[1];
    //printf("buf[0] = %d( 0x%x ), buf[1] = %d (0x%x) \r\n", buf[0], buf[0], buf[1], buf[1]);
    *value = ((double)(temp / 340.0)) + 36.53;
}



/*功能描述：测试芯片读写
 *参数描述：无
 *返回信息：无
 *注意事项：无
 *知识参考：无
 *修订记录：
 *Charlie @2019/02/09
 *  新建
 */

void DMP_Init(void)
{ 
	    if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))    //设置需要的传感器
         printf("mpu_set_sensor complete ......\r\n");
			
      if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) //设置fifo
         printf("mpu_configure_fifo complete ......\r\n");
      if(!mpu_set_sample_rate(200))              //设置采集样率
         printf("mpu_set_sample_rate complete ......\r\n");
      if(!dmp_load_motion_driver_firmware())                //加载dmp固件
        printf("dmp_load_motion_driver_firmware complete ......\r\n");
      if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
         printf("dmp_set_orientation complete ......\r\n"); //设置陀螺仪方向
      if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
         printf("dmp_enable_feature complete ......\r\n");
      if(!dmp_set_fifo_rate(200))    //设置速率
         printf("dmp_set_fifo_rate complete ......\r\n");
      run_self_test();                          //自检
      if(!mpu_set_dmp_state(1))                 //使能
         printf("mpu_set_dmp_state complete ......\r\n");

}



uint8_t Read_DMP(float* Pitch,float* Roll,float* Yaw)
{	
		short gyro[3], accel[3], sensors;
		float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	  unsigned long sensor_timestamp;
		unsigned char more;
		long quat[4];
				if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)) return 0;		
				if (sensors & INV_WXYZ_QUAT)
				{    
					 q0=quat[0] / q30;
					 q1=quat[1] / q30;
					 q2=quat[2] / q30;
					 q3=quat[3] / q30;
					 *Pitch = (float)asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
					 *Roll = (float)atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
					 *Yaw = (float)atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
					
				}	
		return 1;
}


void T_MPU6050(void)
{
//直接读取测试
#if 0
        {
            float temperature = 0.0f;
            //MPU 初始化
            I2C_Initialize();
            MPU6050_Initialize();

            //读取 ID
            if (MPU6050_WhoAmI() != 0x68)
            {
                printf("can't open MPU6050!\r\n");
                return;
            }
            printf("ID got!\r\n");

            //读取温度
            MPU6050_ReadTemperature(&temperature);
            printf("temperature is %.2f\r\n", temperature);
        }
#endif
#if 1
    {
        
        //I2C 总线初始化
        
        //MPU 初始化
        if (mpu_init() != 0)
        {
            printf("Failed to initialize mpu!\r\n");
            return;
        }

        printf("Successed to initialize mpu!\r\n");

        /*
        //设置参数（在 mpu_init 函数中也有这些设置）
        if ( mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0 )
        {
            printf("mpu_set_sensor failed\r\n");
            return;
        }    
        printf("mpu_set_sensor complete\r\n");
        
        if ( mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0 )
        {
            printf("mpu_configure_fifo failed!\r\n");
            return;
        }
        printf("mpu_configure_fifo complete\r\n");
        
        if ( mpu_set_sample_rate( 200 ) != 0 )
        {
            printf("mpu_set_sample_rate failed!\r\n");
            return;
        }    
        printf("mpu_set_sample_rate complete\r\n");
        
        if (dmp_load_motion_driver_firmware() != 0)
        {
            printf("dmp_load_motion_driver_firmware failed!\r\n");
            return;
        }
        printf("dmp_load_motion_driver_firmware complete\r\n");

        if ( dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)) !=0 )
        {
            printf("dmp_set_orientation failed\r\n");
            return;
        }
            
        printf("dmp_set_orientation complete\r\n");
        if ( dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                DMP_FEATURE_GYRO_CAL) != 0 )
                                {
                                    printf("dmp_enable_feature failed!\r\n");
                                    return;
                                }
        printf("dmp_enable_feature complete\r\n");

        if ( dmp_set_fifo_rate( 200 ) != 0 )
        {
            printf("dmp_set_fifo_rate failed!\r\n");
            return;
        }
            
        run_self_test();
        if (mpu_set_dmp_state(1) != 0 )
        {
            printf("mpu_set_dmp_state failed!\r\n");
            return;
        }
        
        printf("mpu_set_dmp_state complete\r\n");
        */

       if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))    //设置需要的传感器
         printf("mpu_set_sensor complete ......\r\n");
      if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) //设置fifo
         printf("mpu_configure_fifo complete ......\r\n");
      if(!mpu_set_sample_rate(200))              //设置采集样率
         printf("mpu_set_sample_rate complete ......\r\n");
      if(!dmp_load_motion_driver_firmware())                //加载dmp固件
        printf("dmp_load_motion_driver_firmware complete ......\r\n");
      if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
         printf("dmp_set_orientation complete ......\r\n"); //设置陀螺仪方向
      if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
         printf("dmp_enable_feature complete ......\r\n");
      if(!dmp_set_fifo_rate(200))    //设置速率
         printf("dmp_set_fifo_rate complete ......\r\n");
      run_self_test();                          //自检
      if(!mpu_set_dmp_state(1))                 //使能
         printf("mpu_set_dmp_state complete ......\r\n");

        printf("passed!\r\n");
    }
#endif
}
