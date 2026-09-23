#include "LSM6DS3TR.h"







uint8_t	LSM_ReadI2C(unsigned char Reg)
{
	uint8_t WData[2], RData;
	WData[0] = Reg;

	HAL_I2C_Master_Transmit(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, WData, 1, 10);
	HAL_I2C_Master_Receive(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, &RData, 1, 10);

	return RData;
}

void	LSM_WriteI2C(unsigned char Reg, unsigned char Data)
{
	uint8_t WData[3];
	WData[0] = Reg;
	WData[1] = Data;

	HAL_I2C_Master_Transmit(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, WData, 2, 10);
}


void	LSM6DS3TR_Reset(void)
{
	LSM_WriteI2C(CTRL3_C,0x01);
}

void	LSM6DS3TR_Init(void)
{
	LSM_WriteI2C(FUNC_CFG_ACCESS,0x00);
	
	LSM_WriteI2C(CTRL1_XL, 	ACCELEROMETER_ODR_6660_HZ | ACCELEROMETER_FULL_SCALE_8G);
	LSM_WriteI2C(CTRL2_G, 	GYROSCOPE_ODR_416_HZ | GYROSCOPE_FULL_SCALE_2000DPS);
	//LSM_WriteI2C(CTRL6_C, 0x81);//Gyroscope LPF1 bandwidth selection on 937 Hz
	
}

int16_t	LSM_ReadGyroZ(void)
{
	int16_t Result;
	uint8_t WData[2], RData[2];
	WData[0] = OUTZ_L_G;
	
	HAL_I2C_Master_Transmit(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, WData, 1, 10);
	HAL_I2C_Master_Receive(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, RData, 2, 10);
	
	Result = (int16_t) (RData[1] << 8) | RData[0];
	
	return Result;
}

int16_t	LSM_ReadAccX(void)
{
	int16_t Result;
	uint8_t WData[2], RData[2];
	WData[0] = OUTX_L_XL;
	
	HAL_I2C_Master_Transmit(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, WData, 1, 10);
	HAL_I2C_Master_Receive(&LSM_I2C_BUS, LSM6DS3TR_ADDRESS, RData, 2, 10);
	
	Result = (int16_t) (RData[1] << 8) | RData[0];
	
	return Result;
}

