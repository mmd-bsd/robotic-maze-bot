
#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "main.h"


#define IR_PWR(x)    		(x ? (GPIOA ->BSRR =0x0100) : (GPIOA ->BRR =0x0100) ) //PA8
#define BUZZER(x)    		(x ? (GPIOA ->BSRR =0x0001) : (GPIOA ->BRR =0x0001) ) //PA0
#define GTD_SLEEP(x)		(x ? (GPIOA ->BSRR =0x0010) : (GPIOA ->BRR =0x0010) ) //PA4

#define MUX_A(x)				(x ? (GPIOA ->BSRR =0x8000) : (GPIOA ->BRR =0x8000) ) //PA15
#define MUX_B(x)				(x ? (GPIOB ->BSRR =0x0008) : (GPIOB ->BRR =0x0008) ) //PB3
#define MUX_C(x)				(x ? (GPIOB ->BSRR =0x0010) : (GPIOB ->BRR =0x0010) ) //PB4

#define KEY1						((GPIOB->IDR & 0x0020) == 0) //PB5
#define KEY2						((GPIOC->IDR & 0x8000) == 0) //PC15
#define KEY3						((GPIOC->IDR & 0x4000) == 0) //PC14

#define NUM_OF_LED_BIT	72
#define RGB_PWM_0_BIT		23
#define RGB_PWM_1_BIT		46

/////	Bluetooth USART
void USART1_Init (void);
void DMA1_CH2_Init(void);
void BLT_SendData(int length);

/////	GTD33 USART
void	USART2_Init(void);
void	DMA1_CH4_Init(void);
void	GTDMotor_SendData(int length);
void 	DMA1_CH5_Init(void);
	
/////	Timers
void	TIM14_Init(void);	//1ms   Interrupter
void	TIM17_Init (void);	//100us Interrupter
void	TIM16_Init(void);		//RGB LED PWM
void	DMA1_CH1_Init(void);//DMA for RGB PWM
void	RGB_SendData(void);
void 	Set_RGB(uint8_t NumOfLED,uint8_t	Color,uint8_t Brightness);

/////	Analog
void	ADC_Init(void);
void 	Read_IRSensors(void);



#define CLEAR				0
#define GREEN				1
#define	RED					2
#define BLUE				3
#define BLUE_LIGHT	4
#define PINK				5
#define YELLOW			6
#define WHITE				7






#endif


