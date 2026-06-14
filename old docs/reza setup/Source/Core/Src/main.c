/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "Hardware.h"
#include "Serial.h"
#include "LSM6DS3TR.h"
#include "Definitions.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

///////// Bluetooth
char                 	BLT_TX_Buffer[UART_BUFFER_SIZE];
volatile 	char 				BLT_RX_Buffer[UART_BUFFER_SIZE];

/////////	GTD Motors
char                 	GTD_TX_Buffer[UART_BUFFER_SIZE];
volatile 	char 				GTD_RX_Buffer[UART_BUFFER_SIZE];
_Bool									GTD_NewPacketF=0,GTD_NewFeedback=0,GTD_Shutdown=1;
volatile	uint8_t			GTD_RxModeF=0,GTD_WaitingForFeedbackTimer=0;
uint8_t								GTD_PacketByteCounter=0,GTD_SyncStatus=0,GTD_PacketLen=0,GTD_ReceivedByte[50],GTD_CommandController=0;
uint16_t							GTD_LastTxDMACNT=0,GTD_LastRxDMACNT=0;
uint16_t							GTD_RXDataCNT=0,GTD_PacketError=0;

/////////	Timer
volatile	uint16_t		Pr=0,Task2Ms=0,Task10Ms=0,Task100Ms=0,Task1000Ms=0,Task20Ms=0;

/////////	Gyro & Angle
_Bool									GyroCalF=0;
uint16_t							GyroCalTimer=0;
float 								Gyro_Z=0,Gyro_Z_Offset=0,Z_Angle=0,Z_AngleSetPoint=0;

/////////	Analog
uint16_t							adcv[4],IR_ReadCounter=0,IR_ADC[18],IR_Logic[18];


uint16_t 							RGB_Data[NUM_OF_LED_BIT+1];


volatile	uint16_t		DMA_IntTest=0;

struct	Motor
{
	float			RefAngle;
	int16_t		RefRPM;
	_Bool			Enable;
	_Bool			ResetEncoder;
	uint8_t		TorquePercent,Feedback_TorquePercent;
	uint8_t		Baudrate,Feedback_Baudrate;
	int16_t		RealRPM;
	int32_t		IncrementalEncoder;
	float 		AbsoluteEncoder;
	float 		Current;
	float 		Voltage;
}GTD[5];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void ClearErrors(void)
{
	if(DMA1_Channel1->CNDTR == 0)
	{
		DMA1_Channel1->CCR  &=~DMA_CCR_EN;
	}
	
	if(USART1->ISR & 0x0000000F)
	{
		USART1->ICR |= 0x0000000F;
		USART1->RQR |= USART_RQR_RXFRQ;
	}
	
	if(USART2->ISR & 0x0000000F)
	{
		USART2->ICR |= 0x0000000F;
		USART2->RQR |= USART_RQR_RXFRQ;
	}
}
void 	StartGyroCalibration(void)
{
	GyroCalF=1;
	GyroCalTimer=0;
}

void 	Calculate_Z_Angle(void)
{
	if(GyroCalF)
	{
		Gyro_Z = LSM_ReadGyroZ();
		
		Gyro_Z_Offset = (Gyro_Z_Offset * 0.995) + (Gyro_Z * 0.005);
		
		GyroCalTimer += 2;
		
		if(GyroCalTimer >= GYRO_CALIBRATION_TIME)
		{
			GyroCalF = 0;
			Z_Angle = 0;
		}
	}
	else
	{
		Gyro_Z = LSM_ReadGyroZ() - Gyro_Z_Offset;
		
		Z_Angle += Gyro_Z * (ANGULAR_RATE_SENSITIVITY_2000DPS / 500); //500Hz
	}
	
	if(Z_Angle < 0)				Z_Angle += 360;
	if(Z_Angle >=360.0)		Z_Angle -= 360;
}


uint8_t 	Checksum(char *Buf , uint16_t Length)
{
 uint8_t CheckByte=0;
 
 for(uint16_t i=0; i<Length; i++) 
 {
  CheckByte ^= *(Buf++); 
 }
 return CheckByte;
}

void 	GTDMotor_SetID(uint8_t PreviousID,uint8_t NewID)
{
	GTD_TX_Buffer[0] = 0xAA;
	GTD_TX_Buffer[1] = 0xAA;
	GTD_TX_Buffer[2] = 0;
	GTD_TX_Buffer[3] = PreviousID;
	GTD_TX_Buffer[4] = 0x11; //Command
	GTD_TX_Buffer[5] = NewID;	//Data
	
	GTD_TX_Buffer[6] = Checksum(&GTD_TX_Buffer[2],4);
	
	GTDMotor_SendData(7);
}

void 	GTD_SendFeedbackCommand(uint8_t ID)
{
	GTD_TX_Buffer[0] = 0xAA;
	GTD_TX_Buffer[1] = 0xAA;
	GTD_TX_Buffer[2] = 0;
	GTD_TX_Buffer[3] = ID;
	GTD_TX_Buffer[4] = 0x00; //Command
	GTD_TX_Buffer[5] = 0x00;	//Data
	
	GTD_TX_Buffer[6] = Checksum(&GTD_TX_Buffer[2],4);
	
	GTDMotor_SendData(7);
}

void 	MotorMove(uint8_t	NumOfMotor)
{
	
	GTD_TX_Buffer[0] = 0xAA;
	GTD_TX_Buffer[1] = 0xAA;
	GTD_TX_Buffer[2] = NumOfMotor;
	
	uint16_t StartDataAdd = NumOfMotor + 3;
	
	for(uint8_t	i=0;i<NumOfMotor;i++)
	{
		int16_t	Speed = GTD[i].RefRPM;
		uint8_t	Speed_MSB = (Speed >> 8) & 0xFF;
		uint8_t	Speed_LSB = Speed & 0xFF;
		
		uint16_t	Angle = GTD[i].RefAngle * 10;
		uint8_t	Angle_MSB = (Angle >> 8) & 0xFF;
		uint8_t	Angle_LSB = Angle & 0xFF;
		
		uint8_t	ControlByte = 0;
		if(GTD[i].Enable)					ControlByte |= 0x0F;
		if(GTD[i].ResetEncoder) 	ControlByte |= 0xF0;
		
		GTD_TX_Buffer[i + 3] = i;
		GTD_TX_Buffer[StartDataAdd] = ControlByte;
		GTD_TX_Buffer[StartDataAdd + 1] = Speed_MSB;
		GTD_TX_Buffer[StartDataAdd + 2] = Speed_LSB;
		GTD_TX_Buffer[StartDataAdd + 3] = Angle_MSB;
		GTD_TX_Buffer[StartDataAdd + 4] = Angle_LSB;
		
		StartDataAdd += 5;
	}
	
	GTD_TX_Buffer[StartDataAdd] = Checksum(&GTD_TX_Buffer[2],StartDataAdd - 2);
	
	GTDMotor_SendData(StartDataAdd + 1);
}


void 	GTD_GetPacket(void)
{
	/////////////////////////////////////////////
	//////////  Get Data From Motor   ///////////
	/////////////////////////////////////////////
	while( GTD_RX_DMA_CNT != GTD_RXDataCNT )
	{
		GTD_RxModeF = 1;
		
		uint8_t DataIn = GTD_RX_Buffer[GTD_RXDataCNT];	
		
		
		if(DataIn == 0xAA && GTD_SyncStatus < 4) 		{GTD_SyncStatus++;}
		else if(GTD_SyncStatus < 4) 								GTD_SyncStatus=0;
		else if(GTD_SyncStatus == 4) //Packet length
		{
			GTD_SyncStatus++; 
			GTD_PacketByteCounter = 0;
			
			GTD_PacketLen = DataIn;
		}
		else if(GTD_PacketByteCounter < GTD_PacketLen && GTD_SyncStatus > 4)
		{
			GTD_ReceivedByte[GTD_PacketByteCounter] = DataIn;
			GTD_PacketByteCounter++;
			GTD_SyncStatus++;
		}
		else if(GTD_SyncStatus > 5)
		{
			if(GTD_PacketByteCounter == GTD_PacketLen && GTD_PacketLen > 1)
			{
				if(DataIn == (Checksum((char *)GTD_ReceivedByte,GTD_PacketLen)^GTD_PacketLen) )
				{
					GTD_NewPacketF = 1;
				}
			}
			
			if(GTD_NewPacketF == 0) {GTD_PacketError ++;}
			GTD_SyncStatus = 0;
			GTD_PacketByteCounter = 0;
		}
		
		
		if(GTD_NewPacketF)
		{
			uint8_t	Incoming_ID=0;
			
			if(GTD_PacketLen == 3) //Getting motor's registers value packet
			{
				Incoming_ID = GTD_ReceivedByte[0];
				
				GTD[Incoming_ID].Feedback_TorquePercent = GTD_ReceivedByte[1];
				GTD[Incoming_ID].Feedback_Baudrate = GTD_ReceivedByte[2];
			}
			else if(GTD_PacketLen == 11) //Getting motor's feedback packet
			{
				Incoming_ID = GTD_ReceivedByte[0];
				
				GTD[Incoming_ID].AbsoluteEncoder = (((int)GTD_ReceivedByte[1] << 8) | GTD_ReceivedByte[2]) / 10.0;
				
				int32_t	Data = 0;
				Data |= ((int32_t) GTD_ReceivedByte[3]) << 24;
				Data |= ((int32_t) GTD_ReceivedByte[4]) << 16;
				Data |= ((int32_t) GTD_ReceivedByte[5]) << 8;
				Data |= ((int32_t) GTD_ReceivedByte[6]);
				GTD[Incoming_ID].IncrementalEncoder = Data;
				
				GTD[Incoming_ID].RealRPM = (((int)GTD_ReceivedByte[7] << 8) | GTD_ReceivedByte[8]);
				
				GTD[Incoming_ID].Current = GTD_ReceivedByte[9] / 20.0;
				GTD[Incoming_ID].Voltage = GTD_ReceivedByte[10] / 10.0;
			}
			
			GTD_WaitingForFeedbackTimer = 0;
			GTD_NewFeedback = 1;
			GTD_NewPacketF = 0;
		}
		
		GTD_RXDataCNT++;
		if(GTD_RXDataCNT >= UART_BUFFER_SIZE) GTD_RXDataCNT = 0;
	}
}

void 	Motor(int left,int right,_Bool Enable)
{
	GTD[0].Enable = Enable;
	//GTD[0].ResetEncoder = 1;
	GTD[1].Enable = Enable;
	//GTD[1].ResetEncoder = 1;
	
	if(left > 1200) 	left = 1200;
	if(left <-1200) 	left =-1200;
	if(right > 1200) 	right = 1200;
	if(right <-1200) 	right =-1200;
	
	GTD[0].RefRPM =-left;
	GTD[0].RefAngle = 400;
	GTD[1].RefRPM = right;
	GTD[1].RefAngle = 400;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////  EVERY 100 us  //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void TIM17_IRQHandler(void)
{
	TIM17->SR =0;
	
	Task2Ms++;
	
	Read_IRSensors();
}

////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////  EVERY 1 ms  ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void TIM14_IRQHandler(void)
{
  TIM14->SR =0;
	
	Pr++;
	Task10Ms++;
	Task100Ms++;
	Task1000Ms++;
	Task20Ms++;
	
	/////////////////////////////////////////////// Receiving Flag Control
	if(GTD_RX_DMA_CNT != GTD_LastRxDMACNT)	{GTD_RxModeF=1;	GTD_LastRxDMACNT = GTD_RX_DMA_CNT;}
	else if(GTD_RxModeF)										{GTD_RxModeF--;}
	if(GTD_WaitingForFeedbackTimer)		GTD_WaitingForFeedbackTimer--;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	GTD_SLEEP(1);			//Sleep GTD Motors
	USART1_Init();		//Serial for the bluetooth
	DMA1_CH2_Init();	//TX Bluetooth
	USART2_Init();		//Serial for the GTD motors
	DMA1_CH4_Init();	//TX GTD
	DMA1_CH5_Init();	//RX GTD
	ADC_Init();				
	TIM14_Init();			//1ms   Interrupter
	TIM17_Init(); 		//100us Interrupter
	TIM16_Init(); 		//RGB LED PWM
	DMA1_CH1_Init();	//DMA for RGB PWM
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	Set_RGB(0,PINK,10);
	Set_RGB(1,BLUE_LIGHT,10);
	Set_RGB(2,GREEN,10);
	RGB_SendData();
	
	HAL_Delay(300);
	LSM6DS3TR_Reset();
	LSM6DS3TR_Init();
	Set_RGB(0,CLEAR,0);
	Set_RGB(1,CLEAR,0);
	Set_RGB(2,CLEAR,0);
	
	
//	BUZZER(1);
//	HAL_Delay(200);
//	BUZZER(0);
//	HAL_Delay(200);


//	BLT_SendData(sprintf(BLT_TX_Buffer,"\r\nStart I2C Scan (7Bit Address): \r\n"));
//	HAL_Delay(100);
//	uint8_t NumOfDevice = 0;

//	for(uint8_t i=1; i<128; i++)
//	{
//		uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5); //7 Bit Address
//		if (ret != HAL_OK) /* No ACK Received At That Address */
//		{
//			BLT_SendData(sprintf(BLT_TX_Buffer," - "));
//		}
//		else if(ret == HAL_OK)
//		{
//			NumOfDevice++;
//			BLT_SendData(sprintf(BLT_TX_Buffer,"0x%X", i));
//		}
//		HAL_Delay(10);
//	}
//	BLT_SendData(sprintf(BLT_TX_Buffer,"\r\n NumOfDevice: %u \r\n",NumOfDevice));
//	/*--[ Scanning Done ]--*/
	
	Task2Ms = 0;
	Task10Ms = 0;
	Task100Ms = 0;
	Task1000Ms = 0;
	Task20Ms = 0;
	Gyro_Z=0,Gyro_Z_Offset=0,Z_Angle=0;
	
	StartGyroCalibration();
	
	GTD_Shutdown = 1;
	Motor(0,0,0);
	
	uint8_t Lori = 0;
	int BaseSpeed = 140;
			
  while (1)
  {
		ClearErrors();
		GTD_GetPacket();
		
		
		//BUZZER(GyroCalF);
//		HAL_Delay(500);
		if(KEY1) IR_PWR(1);
		if(KEY2) IR_PWR(0);
//		HAL_Delay(500);
		
		//HAL_Delay(1);
		
		/////////////////////////////////////////////////////////////////////////////
		///////////////////////////// Task 2 ms (500Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task2Ms > 19)
		{
			Task2Ms -= 20;
			
			Calculate_Z_Angle();
			
			
			///////////	Motor control
			if(GTD_Shutdown == 0 && GTD_RxModeF == 0 && GTD_WaitingForFeedbackTimer == 0)
			{
				if(GTD_CommandController == 4)
				{
					GTD_SendFeedbackCommand(0x00);		//Left motor feedback
					GTD_WaitingForFeedbackTimer = 10; //Waiting for 10ms
				}
				else if(GTD_CommandController == 9)
				{
					GTD_SendFeedbackCommand(0x01);		//Right motor feedback
					GTD_WaitingForFeedbackTimer = 10;	//Waiting for 10ms
				}
				else
				{
					MotorMove(2);
					GTD[0].ResetEncoder = 0;
					GTD[1].ResetEncoder = 0;
				}
				
				GTD_CommandController++;
				if(GTD_CommandController > 9) GTD_CommandController = 0;
			}
			GTD_SLEEP(GTD_Shutdown);
		}
		
		
		/////////////////////////////////////////////////////////////////////////////
		//////////////////////////// Task 10 ms (100Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task10Ms > 9)
		{
			Task10Ms -= 10;
			
			//Z_AngleSetPoint = 0;
			
			float Angle1 = Z_Angle;
			float Angle2 = Z_AngleSetPoint;
			float DifAngle = Angle2 - Angle1;
			if(DifAngle > 180) DifAngle -= 360;
			if(DifAngle <-180) DifAngle += 360;
			
			float Error = DifAngle;
			
			int PID_Out = 0;
			if(BaseSpeed > 0) PID_Out = Error * 3.5;
			
			else PID_Out = Error * 2;
			
			#define MAX_PID 1000
			if(PID_Out > MAX_PID) PID_Out = MAX_PID;
			if(PID_Out <-MAX_PID) PID_Out =-MAX_PID;
			
			
			
			if(BaseSpeed >= 0) 
			{
				if(PID_Out >=0) Motor(BaseSpeed,BaseSpeed+PID_Out,1);
				else            Motor(BaseSpeed-PID_Out,BaseSpeed,1);
			}
			
			else 
			{
				if(PID_Out >=0) Motor(BaseSpeed-PID_Out,BaseSpeed,1);
				else            Motor(BaseSpeed,BaseSpeed+PID_Out,1);
			}
			if(Z_Angle < 0)		Z_Angle += 360;
			if(Z_Angle >=360)	Z_Angle -= 360;
		}
		
		
		/////////////////////////////////////////////////////////////////////////////
		//////////////////////////// Task 20 ms (50Hz) //////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		
		if(Task20Ms > 19)
		{
			Task20Ms -= 20;
			
		}
		/////////////////////////////////////////////////////////////////////////////
		//////////////////////////// Task 100 ms (10Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task100Ms > 99)
		{
			Task100Ms -= 100;
			
			if(GyroCalF) 
			{
				Set_RGB(0,RED,10);
				GTD_Shutdown = 1;
			}
			else         
			{
				Set_RGB(0,WHITE,5);
				GTD_Shutdown = 0;
			}
			Set_RGB(1,CLEAR,0);
			Set_RGB(2,CLEAR,0);
			
			
			RGB_SendData();
			
			
		}
		
		
		/////////////////////////////////////////////////////////////////////////////
		//////////////////////////// Task 1000 ms (1Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task1000Ms > 399)
		{
			Task1000Ms -= 400;
			
			if(GyroCalF == 0)
			{
				if(Lori == 0)				Z_AngleSetPoint = 0;
				else if(Lori == 1)	Z_AngleSetPoint = 90;
				else if(Lori == 2) 	Z_AngleSetPoint = 0;
				else if(Lori == 3) 	Z_AngleSetPoint = 270;
				else if(Lori == 4) 	Z_AngleSetPoint = 0;
				else if(Lori == 5)	Z_AngleSetPoint = 90;
				else if(Lori == 6) 	Z_AngleSetPoint = 0;
				else if(Lori == 7)	Z_AngleSetPoint = 270;
				else if(Lori == 8)	Z_AngleSetPoint = 0;
				else if(Lori == 9)	{GTD[0].ResetEncoder = 1; BaseSpeed =-140;}
				else if(GTD[0].IncrementalEncoder > 2200) BaseSpeed = 0;
//				if(Lori == 1)				Z_AngleSetPoint = 45;
//				else if(Lori == 2) 	Z_AngleSetPoint = 0;
//				else if(Lori == 3) 	Z_AngleSetPoint = 315;
//				else if(Lori == 4) 	Z_AngleSetPoint = 0;
//				else if(Lori == 5)	Z_AngleSetPoint = 45;
//				else if(Lori == 6) 	Z_AngleSetPoint = 0;
//				else if(Lori == 7)	Z_AngleSetPoint = 45;
//				else if(Lori == 8)	Z_AngleSetPoint = 0;
//				else if(Lori == 9)	Z_AngleSetPoint = 180;
//				else if(Lori == 10)	Z_AngleSetPoint = 180;
//				else if(Lori == 11)	Z_AngleSetPoint = 180;
//				else if(Lori == 12)	Z_AngleSetPoint = 180;
//				else if(Lori == 13)	Z_AngleSetPoint = 180;
				Lori++;
				
				//Z_AngleSetPoint += 45;
				//if(Z_AngleSetPoint >= 360) Z_AngleSetPoint -= 360;
			}
		}
		
		
		if(Pr > 50)
		{
			Pr = 0;
			
			BLT_SendData(sprintf(BLT_TX_Buffer,"%d\r\n",GTD[0].IncrementalEncoder));
			//BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%d\r\n",GTD[0].AbsoluteEncoder,GTD[0].IncrementalEncoder));
			//BLT_SendData(sprintf(BLT_TX_Buffer,"[%6.1f,%6d][%6.1f,%6d]\r\n",GTD[0].AbsoluteEncoder,GTD[0].IncrementalEncoder,GTD[1].AbsoluteEncoder,GTD[1].IncrementalEncoder));
			//BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%.1f,%.1f\r\n",Gyro_Z,Z_Angle,Gyro_Z_Offset));
			
//			BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u\r\n",
//				IR_ADC[0],IR_ADC[1]	,IR_ADC[2]	,IR_ADC[3]	,IR_ADC[4]	,IR_ADC[5]	,IR_ADC[6]	,IR_ADC[7]	,IR_ADC[8]
//			,	IR_ADC[9],IR_ADC[10],IR_ADC[11]	,IR_ADC[12]	,IR_ADC[13]	,IR_ADC[14]	,IR_ADC[15]	,IR_ADC[16]	,IR_ADC[17]));
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00602173;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
