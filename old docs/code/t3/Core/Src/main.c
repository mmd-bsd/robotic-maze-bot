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
uint16_t							adcv[4],IR_ReadCounter=0,IR_ADC[18],IR_Logic[18],IR_max[18],IR_min[18],
                      IR_mid[18]={1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

int8_t            s_delay[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};											
											
int s[18];
int left_poss=0;
int right_poss=0;
uint8_t head_delay=0;


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
	
	
	
	    if(Z_Angle < 0)	   {	Z_Angle += 360.0;    Z_AngleSetPoint += 360.0;}
			if(Z_Angle >=360.0){	Z_Angle -= 360.0;    Z_AngleSetPoint -= 360.0;}
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

int trans_s_left=0;
int trans_s_right=0;


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
	
	if(trans_s_left < left)      {trans_s_left++;}
	else if(trans_s_left > left) {trans_s_left--;}

	if(trans_s_right < right)      {trans_s_right++;}
	else if(trans_s_right > right) {trans_s_right--;}
		
	
	GTD[0].RefRPM =-trans_s_left;
	GTD[0].RefAngle = 400;
	GTD[1].RefRPM = trans_s_right;
	GTD[1].RefAngle = 400;
}

  int BaseSpeed = 0;
	float Error=0;
	float integral=0,derivative=0,prevError=0;
  float acc=800;
  float ti = 0;

void keep_head()
{
	
			float Angle1 = Z_Angle;
			float Angle2 = Z_AngleSetPoint;
			float DifAngle = Angle2 - Angle1;
			if(DifAngle > 180) DifAngle -= 360;
			if(DifAngle <-180) DifAngle += 360;
			
		  Error = DifAngle;
			float kp=1,kd=0.0,ki=0;
			integral   = integral + (Error/100) ;	 
      derivative = (Error - prevError)*100 ; 
			
			
			int PID_Out = 0;
			
			
	
		  PID_Out = (Error * kp) + (integral* ki) + (derivative * kd);
			
			prevError  = Error;
       
			#define MAX_PID 1000
			if(PID_Out > MAX_PID) PID_Out = MAX_PID;
			if(PID_Out <-MAX_PID) PID_Out =-MAX_PID;
			
		//	BaseSpeed= acc * ti ;
		//	if(BaseSpeed>700) BaseSpeed=700;
		//	PID_Out=0;
			 
			Motor(BaseSpeed-PID_Out,BaseSpeed+PID_Out,1);
				
		
	
	
}

void Forward()
{
	if(s[4] && s[5])
	{
			Motor(120,120,1);
	}
	else if (s[4] && s[5]==0)
	{
		
			Motor(120,125,1);
	}
	else if (s[4]==0 && s[5])
	{	
			Motor(125,120,1);
	}
  if (s[6])
	{	
			Motor(130,120,1);
	}
  else if (s[3])
	{
			Motor(120,130,1);
	}
  else if (s[2])
	{
			Motor(120,140,1);
	}
  else if (s[7])
	{
			Motor(140,120,1);
	}	
  	
	
	
}
void Forward_r()
{
	if(s[13] && s[14])
	{
			Motor(-120,-120,1);
	}
	else if (s[13] && s[14]==0)
	{
		
			Motor(-125,-120,1);
	}
	else if (s[13]==0 && s[14])
	{	
			Motor(-120,-125,1);
	}
  if (s[15])
	{	
			Motor(-120,-130,1);
	}
  else if (s[12])
	{
			Motor(-130,-120,1);
	}
  else if (s[11])
	{
			Motor(-140,-120,1);
	}
  else if (s[16])
	{
			Motor(-120,-140,1);
	}	
  	
	
	
}

	int         IR_init_val=0;
	uint16_t    cal_time=186;
	int         calibrat_now=0,loop_start=0;

void calibr_ir()
{
	      BUZZER(1);	
        if(IR_init_val==0)
				{		
          for(int i=0;i<18;i++)
					{					
							IR_max[i]=IR_ADC[i];
							IR_min[i]=IR_ADC[i];
					}				
					IR_init_val=1;
				}		
         	
				if(cal_time>0)
				{
						Motor(40,-40,1);
						for(int i=0;i<18;i++)
						{
							if      (IR_ADC[i]>IR_max[i])  IR_max[i]=IR_ADC[i];
							if      (IR_ADC[i]<IR_min[i])  IR_min[i]=IR_ADC[i];
			    	}
						
						cal_time--;
				}
				else 
				{
						 BUZZER(0);
					   Motor(0,0,1);
					 	 calibrat_now=0;
						 cal_time=186;
					   Z_AngleSetPoint=0;
						 for(int i=0; i<18; i++)
					   {
								IR_mid[i] = (IR_max[i]+IR_min[i])*0.5;
							
					 		
						 }
						
				}
	
	
}
int roatating=0,cheft=0;
uint8_t cross=0;
int head=0,step=0,angle_flag=0;


void turn_left()
{
	
	  left_poss=0;
	  right_poss=0;
	
	  if(s[1]==0 && s[8]==0 && cheft==0)
		{
			
			Motor(20,20,1);
			
		}
	  else if (angle_flag==0)
		{
			Z_AngleSetPoint+=90;
			
			if(Z_AngleSetPoint<0)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
			else if(Z_AngleSetPoint>=360) {Z_AngleSetPoint-=360; Z_Angle-=360;}
			
			cheft=1;
			angle_flag=1;
			
		}
		else
		{	 
				Error= Z_AngleSetPoint-Z_Angle;
		  	Motor(20,50+Error,1);
		  	
	
		}
		
		
		          if( step ==0 && s[4]==0 && s[5]==0)           step =1;
						  else	if( step ==1 && (s[4]==1  || s[5]==1))    step =3;
						//	else if( step ==2 && s[5]==1)                 step=3;
							else if( step ==3)
							{
							  cross=0;
								step =0;
								roatating=0;
								cheft=0;
								angle_flag=0;
							}
		
		
		
	
}
void turn_left_r()
{
	
	  left_poss=0;
	  right_poss=0;
	
	  if(s[10]==0 && s[17]==0 && cheft==0)
		{
			
			Motor(-20,-20,1);
			
		}
		else if (angle_flag==0)
		{
			Z_AngleSetPoint+=90;
			
			if(Z_AngleSetPoint<0)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
			else if(Z_AngleSetPoint>=360) {Z_AngleSetPoint-=360; Z_Angle-=360;}
			
			cheft=1;
			angle_flag=1;
			
		}
		else
		{	 
				Error= Z_AngleSetPoint-Z_Angle;
		  	Motor(-(50+Error),-20,1);
		  	
	
		}
		
		
		          if( step ==0 && s[13]==0 && s[14]==0)           step =1;
						  else	if( step ==1 && (s[13]==1  ||  s[14]==1)) step =3;
						//	else if( step ==2 && s[14]==1)                 step=3;
							else if( step ==3)
							{
							  cross=0;
								step =0;
								roatating=0;
								cheft=0;
								angle_flag=0;
							}
		
		
		
	
}




void turn_right()
{
	
	
	  left_poss=0;
	  right_poss=0;
	
	  if(s[1]==0 && s[8]==0 && cheft==0)
		{
			
			Motor(20,20,1);
			
		}
		else if (angle_flag==0)
		{
			Z_AngleSetPoint-=90;
			
			if(Z_AngleSetPoint<0)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
			else if(Z_AngleSetPoint>=360) {Z_AngleSetPoint-=360; Z_Angle-=360;}
			
			cheft=1;
			angle_flag=1;
			
		}
		else
		{	 
				Error= Z_AngleSetPoint-Z_Angle;
		  	Motor(50-Error,30,1);
		  	
	
		}
		          if     ( step ==0 && s[4]==0   && s[5]==0)    step =1;
							else if( step == 1 && (s[5]==1 || s[4]==1))   step =3;
						//	else if( step ==2  && s[4]==1)                step=3;
							else if( step ==3)
							{
							  cross=0;
								step =0;
								roatating=0;
								cheft=0;
								angle_flag=0;
							}
	
}

void turn_right_r()
{
	
	
	  left_poss=0;
	  right_poss=0;
	
	  if(s[10]==0 && s[17]==0 && cheft==0)
		{
			
			Motor(-20,-20,1);
			
		}
		else if (angle_flag==0)
		{
			Z_AngleSetPoint-=90;
			
			if(Z_AngleSetPoint<0)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
			else if(Z_AngleSetPoint>=360) {Z_AngleSetPoint-=360; Z_Angle-=360;}
			
			cheft=1;
			angle_flag=1;
			
		}
		else
		{	 
				Error= Z_AngleSetPoint-Z_Angle;
		  	Motor(-20,-(50-Error),1);
		  	
	
		}
		          if     ( step ==0 && s[13]==0   && s[14]==0)     step =1;
							else if( step == 1 && (s[14]==1 || s[13]==1))    step =3;
						//	else if( step ==2  && )                step=3;
							else if( step ==3)
							{
							  cross=0;
								step =0;
								roatating=0;
								cheft=0;
								angle_flag=0;
							}
	
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
	

	
	
	GTD_Shutdown = 1;
	Motor(0,0,0);
	
	uint8_t Lori = 0;

	
	while(KEY1==0)
	{
		
		
	}
	    HAL_Delay(500);
	    BUZZER(1);
	    StartGyroCalibration();
		 	HAL_Delay(100);
		  BUZZER(0);		
			
	
	
	
  while (1)
  {
		ClearErrors();
		GTD_GetPacket();
		
		IR_PWR(1);
		//BUZZER(GyroCalF);
//		HAL_Delay(500);
	
		if(KEY1) loop_start=1,ti=0;
		if(KEY2) calibrat_now=1;
		
	
	
		
		if(loop_start)
		{
			
			
			if(head==0)                       ///////// front is front
			{
				
		  		if (roatating==0)
					{
						    if (s[2]  && (s[3] || s[4] || s[5] ||s[6])) {left_poss=1;}
								if (s[7]  && (s[3] || s[4] || s[5] ||s[6])) {right_poss=1;}
						
						
						    if(right_poss || left_poss)
								{
						
						
												if(right_poss && left_poss && (s[1] || s[8]))
												{
													cross=1;                                     //////////////cross with both side open
												}
												else if (left_poss && right_poss==0 && s[1] )
												{
													cross =2;                                  /////////////////cross with left open only
													
												}
												else if (right_poss && left_poss==0 && s[8] )
												{
													cross =3;                                  /////////////////////////////cross with right open only
													
												}

							  }
	            	else if (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0 && head_delay < 50)
								{
                   head_delay++;
								

								}	
								else if (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0 && head_delay >= 50)
								{
                   head_delay = 0;
									 cross =4 ;

								}	
					}
				
				
				
				
				
				
						if(cross==0)
						{
								Forward();
						}
						else if (cross==1)
						{
							  roatating=1;
							
								turn_left();
							 
						}
						else if (cross==2)
						{
							roatating=1;
							turn_left();
					  	
							
						}
						else if (cross==3)
						{
							roatating=1;
							turn_right();
							
						}
						else if (cross==4)
						{
						   	head=1;
							  cross=0;
						 	
						
							
						}
						
			}
			else                                 ///////////////////bot algorithm roatate 180 deg
			{
				//BUZZER(1);
				if (roatating==0)
					{
						    if (s[11]  && (s[12] || s[13] || s[14] || s[15])) {left_poss=1;}
								if (s[16]  && (s[12] || s[13] || s[14] || s[15])) {right_poss=1;}
						
						
						    if(right_poss || left_poss)
								{
						
						
												if(right_poss && left_poss && (s[10] || s[17]))
												{
													cross=1;                                     //////////////cross with both side open
												}
												else if (left_poss && right_poss==0 && s[10] )
												{
													cross =2;                                  /////////////////cross with left open only
													
												}
												else if (right_poss && left_poss==0 && s[17] )
												{
													cross =3;                                  /////////////////////////////cross with right open only
													
												}

							  }
	            	else if (s[12]==0 && s[13]==0 && s[14]==0 && s[15]==0 && head_delay < 50)
								{
                   head_delay++;
							

								}
                else if (s[12]==0 && s[13]==0 && s[14]==0 && s[15]==0 && head_delay >= 50)
								{
                   
									 cross =4 ;
									 head_delay=0;

								}	
																
								
					}
				
				
				
				
				
				
						if(cross==0)
						{
								Forward_r();
						}
						else if (cross==1)
						{
							  roatating=1;
							
								turn_left_r();
							 
						}
						else if (cross==2)
						{
							roatating=1;
							turn_left_r();
					  	
							
						}
						else if (cross==3)
						{
							roatating=1;
							turn_right_r();
							
						}
						else if (cross==4)
						{
						   	head=0;
							  cross=0;
						
							
						}
				
				
				
			}
			
		}
		
		/////////////////////////////////////////////////////////////////////////////
		///////////////////////////// Task 2 ms (500Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task2Ms > 19)
		{
			Task2Ms -= 20;
			
			for(int i=0; i<18; i++)
			{
				
				if      (IR_ADC[i] >= IR_mid[i]+100)             s[i]=0;
				else if (IR_ADC[i] <= IR_mid[i]-500)             s[i]=1;
				
			
				//////////////////////////////////////for dirty surface
				
//				if      (IR_ADC[i] >= IR_mid[i]+100)         s_delay[i]++;        ////////////s[i]=0;
//				else if (IR_ADC[i] <= IR_mid[i]-100)         s_delay[i]--;        ////////////s[i]=1;
//				
//				if(s_delay[i]>=5)         {s_delay[i]=5;  s[i]=0;}
//				else if(s_delay[i]<=-5)   {s_delay[i]=-5;  s[i]=1;}
//				
				
				
				
			}
			
			
			
			
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
			ti += 0.01;
			
			if(calibrat_now)
			{
					calibr_ir();
			}
			
		
			
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
		if(Task1000Ms > 3999)
		{
			Task1000Ms -= 4000;
		//	Z_AngleSetPoint+=90;
			
			
			
			

		}
		
		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
		
			
			
			
			

			
		
		
		
		
		
		
		
		
		
		
		
		
		
		if(Pr > 50)
		{
			Pr = 0;
			
		//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d\r\n",GTD[0].IncrementalEncoder));
		//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d,%d,%d\r\n",KEY1,KEY2,KEY3));
		//	BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%d\r\n",GTD[0].AbsoluteEncoder,GTD[0].IncrementalEncoder));
			//BLT_SendData(sprintf(BLT_TX_Buffer,"[%6.1f,%6d][%6.1f,%6d]\r\n",GTD[0].AbsoluteEncoder,GTD[0].IncrementalEncoder,GTD[1].AbsoluteEncoder,GTD[1].IncrementalEncoder));
			//BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%.1f,%.1f\r\n",Gyro_Z,Z_Angle,Gyro_Z_Offset));
		//	BLT_SendData(sprintf(BLT_TX_Buffer,"%.2f,%.2f,%.2f \r\n",Z_AngleSetPoint,Z_Angle,Error));
		 // 	BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u\r\n",
		//		IR_ADC[0],IR_ADC[1]	,IR_ADC[2]	,IR_ADC[3]	,IR_ADC[4]	,IR_ADC[5]	,IR_ADC[6]	,IR_ADC[7]	,IR_ADC[8]
		//	,	IR_ADC[9],IR_ADC[10],IR_ADC[11]	,IR_ADC[12]	,IR_ADC[13]	,IR_ADC[14]	,IR_ADC[15]	,IR_ADC[16]	,IR_ADC[17]));
		 
		//	  BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u \r\n",IR_ADC[3],IR_max[3],IR_min[3],IR_mid[3],s[3]));
	  //	BLT_SendData(sprintf(BLT_TX_Buffer,"%d  ,%d    ,%d,%d,%d,%d,   %d,  %d\r\n",s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8]));
		//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d  ,%d,%d,%d,%d,%d,%d,  %d\r\n",s[10],s[11],s[12],s[13],s[14],s[15],s[16],s[17]));
			BLT_SendData(sprintf(BLT_TX_Buffer,"cross=%d  ,step=%d,head=%d,roat=%d,   %.1f,   %.1f\r\n",cross,step,head , roatating,Z_Angle,Z_AngleSetPoint));
		//		BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%.1f\r\n",Z_Angle,Z_AngleSetPoint));
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
