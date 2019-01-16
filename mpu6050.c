

#include "include.h"

int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化MPU6050
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void InitMPU6050(void)
{
//    simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	   //解除休眠状态
//    simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);      //125HZ采样率
//    simiic_write_reg(MPU6050_DEV_ADDR, CONFIG, 0x04);          //
//    simiic_write_reg(MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);     //2000
//    simiic_write_reg(MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x10);    //8g
//    simiic_write_reg(MPU6050_DEV_ADDR, User_Control, 0x00);
//    simiic_write_reg(MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);


	simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	   //解除休眠状态
	simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x00);      //125HZ采样率
	simiic_write_reg(MPU6050_DEV_ADDR, CONFIG, 0x04);          //
	simiic_write_reg(MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);     //2000
	simiic_write_reg(MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x10);    //8g
	simiic_write_reg(MPU6050_DEV_ADDR, User_Control, 0x00);
	simiic_write_reg(MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取MPU6050加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void Get_AccData(void)
{
	uint8 dat[6];

	simiic_read_regs(MPU6050_DEV_ADDR, ACCEL_XOUT_H, dat, 6, IIC);
	mpu_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]))>>2;
	mpu_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]))>>2;
	mpu_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]))>>2;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取MPU6050陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void Get_Gyro(void)
{
	uint8 dat[6];

	simiic_read_regs(MPU6050_DEV_ADDR, GYRO_XOUT_H, dat, 6, IIC);
	mpu_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]))>>3;
	mpu_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]))>>3;
	mpu_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]))>>3;
}







void mpu6050_init(void)    //硬件初始化
{
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	   //解除休眠状态
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, SMPLRT_DIV, 0x00);      //1KHZ采样率    //南师大
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, CONFIG, 0x02);          //
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);     //2000
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x00);    //2g
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, User_Control, 0x00);
	i2c_write_reg(i2c1,MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);

}




//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取MPU6050加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_read_ACC(void)
{
	uint8 dat[6];

	i2c_read_reg_bytes(i2c1,MPU6050_DEV_ADDR, ACCEL_XOUT_H, 6,dat);
	mpu_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));         //>>2
	mpu_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));         //>>2
	mpu_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));         //>>2
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取MPU6050陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_read_Gyro(void)
{
	uint8 dat[6];

	i2c_read_reg_bytes(i2c1,MPU6050_DEV_ADDR, GYRO_XOUT_H,6,dat);
	mpu_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));//>>3
	mpu_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));//>>3
	mpu_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));//>>3
}



int mpu6050_read_aX(void)
{
	uint8 dat[2];
	i2c_read_reg_bytes(i2c1,MPU6050_DEV_ADDR, ACCEL_XOUT_H,2,dat);
	return (int16)(((uint16)dat[0]<<8 | dat[1]));
}


int mpu6050_read_gY(void)
{
	uint8 dat[2];
	i2c_read_reg_bytes(i2c1,MPU6050_DEV_ADDR, GYRO_YOUT_H,2,dat);
	return (int16)(((uint16)dat[0]<<8 | dat[1]));
}

int mpu6050_read_gZ(void)
{
	uint8 dat[2];
	i2c_read_reg_bytes(i2c1,MPU6050_DEV_ADDR, GYRO_ZOUT_H,2,dat);
	return (int16)(((uint16)dat[0]<<8 | dat[1]));
}
