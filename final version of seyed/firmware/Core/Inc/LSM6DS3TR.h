#ifndef __LSM6DS3TR_H
#define __LSM6DS3TR_H


#include "main.h"

#define LSM_I2C_BUS					hi2c2
extern I2C_HandleTypeDef 	LSM_I2C_BUS;


#define LSM6DS3TR_ADDRESS		0xD4

/////	LSM6DS3TR Registers
#define FUNC_CFG_ACCESS						0x01
#define SENSOR_SYNC_TIME_FRAME		0x04
#define SENSOR_SYNC_RES_RATIO 		0x05
#define FIFO_CTRL1								0x06

#define DRDY_PULSE_CFG_G 					0x0B
#define INT1_CTRL 								0x0D

#define WHO_AM_I 									0x0F
#define CTRL1_XL 									0x10	//Linear acceleration sensor control registers
#define CTRL2_G										0x11	//Angular rate sensor control register
#define CTRL3_C 									0x12	//BOOT /BDU /H_LACTIVE /PP_OD /SIM IF_INC /BLE /SW_RESET
#define CTRL4_C 									0x13	//DEN_XL_EN /SLEEP /INT2_on_INT1 /DEN_DRDY_INT1 /DRDY_ MASK /I2C_disable /LPF1_SEL_G /0
#define CTRL5_C 									0x14	//ROUNDING2 /ROUNDING1 /ROUNDING0 /DEN_LH /ST1_G /ST0_G /ST1_XL /ST0_XL
#define CTRL6_C 									0x15	//TRIG_EN /LVL_EN /LVL2_EN /XL_HM_MODE /USR_OFF_W /0 /FTYPE_1 /FTYPE_0
#define CTRL7_G 									0x16	//G_HM_MODE /HP_EN_G /HPM1_G /HPM0_G /0 /ROUNDING_STATUS /0 /0
#define CTRL8_XL 									0x17
#define CTRL9_XL 									0x18

#define STATUS_REG								0x1E //0 0 0 0 0 TDA GDA XLDA
#define OUT_TEMP_L 								0x20
#define OUT_TEMP_H 								0x21
#define OUTX_L_G									0x22
#define OUTX_H_G									0x23
#define OUTY_L_G									0x24
#define OUTY_H_G									0x25
#define OUTZ_L_G									0x26
#define OUTZ_H_G									0x27
#define OUTX_L_XL									0x28
#define OUTX_H_XL									0x29
#define OUTY_L_XL									0x2A
#define OUTY_H_XL									0x2B
#define OUTZ_L_XL									0x2C
#define OUTZ_H_XL									0x2D

/////	CTRL1_XL
#define ACCELEROMETER_FULL_SCALE_2G		0x00
#define ACCELEROMETER_FULL_SCALE_4G		0x08
#define ACCELEROMETER_FULL_SCALE_8G		0x0C
#define ACCELEROMETER_FULL_SCALE_16G	0x04
#define ACCELEROMETER_ODR_104_HZ			0x40
#define ACCELEROMETER_ODR_208_HZ			0x50
#define ACCELEROMETER_ODR_416_HZ			0x60
#define ACCELEROMETER_ODR_833_HZ			0x70
#define ACCELEROMETER_ODR_1660_HZ			0x80
#define ACCELEROMETER_ODR_3330_HZ			0x90
#define ACCELEROMETER_ODR_6660_HZ			0xA0

/////	CTRL2_G
#define GYROSCOPE_FULL_SCALE_245DPS		0x00
#define GYROSCOPE_FULL_SCALE_500DPS		0x04
#define GYROSCOPE_FULL_SCALE_1000DPS	0x08
#define GYROSCOPE_FULL_SCALE_2000DPS	0x0C
#define GYROSCOPE_ODR_104_HZ					0x40
#define GYROSCOPE_ODR_208_HZ					0x50
#define GYROSCOPE_ODR_416_HZ					0x60
#define GYROSCOPE_ODR_833_HZ					0x70
#define GYROSCOPE_ODR_1660_HZ					0x80
#define GYROSCOPE_ODR_3330_HZ					0x90
#define GYROSCOPE_ODR_6660_HZ					0xA0


#define ANGULAR_RATE_SENSITIVITY_125DPS		0.004375 	//dps/LSB
#define ANGULAR_RATE_SENSITIVITY_245DPS		0.00875 	//dps/LSB
#define ANGULAR_RATE_SENSITIVITY_200DPS		0.0175 		//dps/LSB
#define ANGULAR_RATE_SENSITIVITY_1000DPS	0.035 		//dps/LSB
#define ANGULAR_RATE_SENSITIVITY_2000DPS	0.070 		//dps/LSB

uint8_t LSM_ReadI2C(unsigned char Reg);
void	LSM_WriteI2C(unsigned char Reg, unsigned char Data);
void	LSM_Init(void);
void	LSM6DS3TR_Init(void);
void	LSM6DS3TR_Reset(void);
int16_t	LSM_ReadGyroZ(void);
int16_t	LSM_ReadAccX(void);

#endif /* __LSM6DS3TR_H */




