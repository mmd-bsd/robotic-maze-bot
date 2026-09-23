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
#include <string.h>

/* The decision core.  It replaces the legacy left-hand-rule explorer: the
 * firmware still owns every sensor, motor, encoder and line-following decision,
 * and the brain owns only "what the maze looks like and where to go next".
 * See robot codes/inc/brain.h for the contract. */
#include "brain.h"

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

int end_zone_timer = 0;

/////////	Gyro & Angle
_Bool									GyroCalF=0;
uint16_t							GyroCalTimer=0;
float 								Gyro_Z=0,Gyro_Z_Offset=0,Z_Angle=0,Z_AngleSetPoint=0;


int nav =0 ; //////0 = north     1 = west    2=south    3 = east

/////////	Analog
uint16_t							adcv[4],IR_ReadCounter=0,IR_ADC[18],IR_Logic[18],IR_max[18],IR_min[18],
                      IR_mid[18]={1400,1200,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400,1400};
											
											
_Bool s[18] , s_current[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int s_counter[18] ;	

#define OffToOnTrsh 5
#define OnToOffTrsh 5
	
	
int left_poss=0;
int right_poss=0;
uint8_t head_delay=0;
int on_link=0;

/* The legacy map (`link[100][4]` + `node[50][6]` = 2800 B) and its command log
 * (`path_discoverd[200]`) are gone.  They were a second, complete navigation
 * stack: the left-hand-rule explorer's own record of the maze, built by
 * path_append()/fill_map() and read by the loop_start==2 dump.
 *
 * Two full maps cannot share 8 KB, and the brain's is the one that is actually
 * used, so the brain owns the map now (see brain.c: MazeGraph + MazeRobot, and
 * its own dead-reckoned coordinates -- no `node[]` read-back is needed).
 *
 * Historical note worth keeping: both arrays were indexed by the running
 * command count, which no array size bounds.  On the real field a mission
 * issues ~70 commands, so the old code wrote ~500 B past node[50] into uwTick
 * and hi2c2 -- a silent HAL/IMU corruption.  The brain cannot repeat that: its
 * node ids are checked against MAZE_MAX_NODES inside maze_solver_update_position
 * and it stops cleanly (BRAIN_DONE) rather than writing out of range. */



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

int delay_cycle(int16_t cycels)
{
	if(cycels > 0)
	{	
		cycels--;
		return 0 ;
	}
	else { return 1 ; }
	
	
}

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
	
	
	
	    if(Z_Angle <= -360.0)	 {	Z_Angle += 360.0;  }
			if(Z_Angle >= 360.0)   {	Z_Angle -= 360.0;  }
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
		int16_t	Speed =GTD[i].RefRPM;
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
////////////////////////////////////////////////////////////////////////////////////////

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

//	
//	GTD[0].RefRPM =-trans_s_left;
//	GTD[0].RefAngle = 400;
//	GTD[1].RefRPM = trans_s_right;
//	GTD[1].RefAngle = 400;
}
///////////////////////////////////////////////////////////////////////////////

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
//////////////////////////////////////////////////////////////////////////////

int lef=0,righ=0;
int left=0,right=0;


void MotorA(int l , int r)
{
	  if(left< l) left+=2;
	  else if (left > l) left-=2;
	
	  if(right< r) right+=2;
	  else if (right > r) right-=2;
	
}

void MotorB(int l, int r)
{
	lef=l;
	righ=r;
	
}
void MotorC(int l, int r)
{
	left=l;
	right=r;
	
}
///////////////////////////////////////////////////////////////////////////////////////

int roatating=0,cheft=0;
uint8_t cross=0;
int head=0,step=0,angle_flag=0;
int cc=0;

void Forward()
{
	if      (s[4] && s[5])        {MotorB(120,120);}
	else if (s[4] && s[5]==0)     {MotorB(120,125);}
	else if (s[4]==0 && s[5])     {MotorB(125,120);}
  else if (s[6])                {MotorB(140,120);}
  else if (s[3])                {MotorB(120,140);}
  else if (s[2])                {MotorB(120,160);}
  else if (s[7])                {MotorB(160,120);}	
  	
  if(s[8] == 0 && s[1] ==0)      roatating=0;		

}
void Forward_r()
{
	if      (s[13] && s[14])      {MotorB(-120,-120);}
	else if (s[13] && s[14]==0)   {MotorB(-125,-120);}
	else if (s[13]==0 && s[14])   {MotorB(-120,-125);}
  else if (s[15])               {MotorB(-120,-140);}
  else if (s[12])               {MotorB(-140,-120);}
  else if (s[11])               {MotorB(-160,-120);}
  else if (s[16])               {MotorB(-120,-160);}	
  	
	if(s[17] == 0 && s[10]==0)     roatating=0;
		
}
///////////////////////////////////////////////////////////////

int         IR_init_val=0;
uint16_t    cal_time=185;
int         calibrat_now=0,loop_start=0;
int16_t cct = 0;

void calibr_ir()
{
	     // BUZZER(1);	
        if(IR_init_val==0)
				{		
					
          for(int i=0;i<18;i++)
					{					
							IR_max[i]=IR_ADC[i];
							IR_min[i]=IR_ADC[i];
					}				
					IR_init_val=1;
					
				}	
        if(cct<100)
				{					
						cct ++ ; 	
				}					
				if(cct==100)
				{
					if(cal_time>0)
					{
							MotorB(40,-40);
							for(int i=0;i<18;i++)
							{
								if      (IR_ADC[i]>IR_max[i])  IR_max[i]=IR_ADC[i];
								if      (IR_ADC[i]<IR_min[i])  IR_min[i]=IR_ADC[i];
							}
							
							cal_time--;
					}
					else 
					{
						   cct = 0 ;
							 BUZZER(0);
							 MotorB(0,0);
							 calibrat_now=0;
							 cal_time=185;
							 Z_AngleSetPoint=0;
							 for(int i=0; i<18; i++)
							 {
									IR_mid[i] = (IR_max[i]+IR_min[i])*0.5;
								
							 }	
							 
							BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u \r\n",IR_mid[0],IR_mid[1],IR_mid[2],
			        IR_mid[3],IR_mid[4],IR_mid[5],IR_mid[6],IR_mid[7],IR_mid[8],IR_mid[9],IR_mid[10],IR_mid[11],IR_mid[12],IR_mid[13],
			        IR_mid[14],IR_mid[15],IR_mid[16],IR_mid[17]));
					}
			}
}



void turn_left()
{

	  left_poss=0;
	  right_poss=0;

	  if (angle_flag==0)
		{
					Z_AngleSetPoint = 85 + Z_Angle;
			
					if     (Z_AngleSetPoint<=-360)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
					else if(Z_AngleSetPoint>=360)          {Z_AngleSetPoint-=360; Z_Angle-=360;}
					
					cheft=1;
					angle_flag=1;
					nav++;
					
				  Error= Z_AngleSetPoint - Z_Angle;
		}
		
		else
		{	 
				 	Error= Z_AngleSetPoint - Z_Angle;
					MotorC(-Error,Error); 	
		}
		
		if( step == 0 && Error<5  && Error>-5)           step =3;
		
		else if( step == 3)
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
	
	  if (angle_flag==0)
		{
						Z_AngleSetPoint = 85+ Z_Angle;
						
						if     (Z_AngleSetPoint<=-360)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
						else if(Z_AngleSetPoint>= 360)         {Z_AngleSetPoint-=360; Z_Angle-=360;}
						
						cheft=1;
						angle_flag=1;
            nav++;
						Error= Z_AngleSetPoint - Z_Angle;
		}
		else
		{	 
		  	Error= Z_AngleSetPoint - Z_Angle;
		  	MotorC(Error,-Error);
		}
		
		if( step ==0 && (Error<5 && Error>-5))           step = 3 ;
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
	  
		if (angle_flag==0)
		{
						Z_AngleSetPoint=Z_Angle-85;
						
						if     (Z_AngleSetPoint<=-360)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
						else if(Z_AngleSetPoint>= 360)         {Z_AngleSetPoint-=360; Z_Angle-=360;}
						
						cheft=1;
						angle_flag=1;
						nav--;
						Error= Z_AngleSetPoint - Z_Angle;
						
		}
		else
		{	 
				    Error= Z_AngleSetPoint - Z_Angle;
		  	    MotorC(70-Error,20);	
		}
		if     ( step ==0 && (Error<5 && Error>-5) )    step = 3;
		
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
	
	
		if (angle_flag==0)
		{
			Z_AngleSetPoint= Z_Angle-85;
			
			if     (Z_AngleSetPoint<=-360)         {Z_AngleSetPoint+=360; Z_Angle+=360;}
			else if(Z_AngleSetPoint>= 360)         {Z_AngleSetPoint-=360; Z_Angle-=360;}
			
			cheft=1;
			angle_flag=1;
			nav--;
	    Error= Z_AngleSetPoint - Z_Angle;
			
		}
		else
		{	 
			  Error= Z_AngleSetPoint - Z_Angle;
		  	MotorC(-20,-70+Error);
		}
		          if     ( step ==0 && (Error<5 && Error>-5))     step =3;
					
							else if( step ==3)
							{
							  cross=0;
								step =0;
								roatating=0;
								cheft=0;
								angle_flag=0;
								
							}
	
}

/* The explored-path string is KEPT, but nothing writes it during exploration any
 * more.  It is the replay buffer the fast run drives: on BRAIN_DONE the brain's
 * plan is copied in here and loop_start==6 replays it exactly as it always did.
 * 200 bytes is enough for MAZE_COMMAND_LOG_SIZE (192) plus the terminator. */
char path_discoverd_s[200];

/* The homeward route, driven by stage 4.  Written by brain_report() on
 * BRAIN_DONE; see the note where back_home() used to be. */
char path_back[200];

signed int ab(signed int num)
{
	if(num<0) num=-num;
	return num;
	
}
#ifdef USE_MAZE_TELEMETRY
/* ======================= M1 telemetry (diagnostic build) ====================
   Compact machine-parseable lines on the Bluetooth link (USART1, 115200 baud).
   Define USE_MAZE_TELEMETRY to get them; with the macro undefined this block
   and every hook below compile away, so the normal firmware is unaffected.

   WHY THE JUNCTION LINE IS QUEUED RATHER THAN SENT
   BLT_SendData() restarts the TX DMA from scratch (Hardware.c:135), so a second
   call before the first has drained silently truncates the first. The stream
   and the junction record would therefore eat each other some of the time --
   and the junction record is the one that matters. So events park their line in
   tlm_pending and the 8 ms slot sends *either* the pending event *or* a stream
   sample, never both. The line carries its own tlm_ms stamp, so queueing costs
   no timing accuracy.

   LINE FORMAT   (all times are ms since reset, from the 1 ms TIM14 tick)
     S,<ms>,<front>,<rear>,<e0>,<e1>,<L>,<R>,<cross>    125 Hz, while driving
     J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>
     Z,<ms>,<state>                                        target zone enter/leave

   <front> = s[0..9]   as hex, bit i = s[i]
   <rear>  = s[10..17] as hex, bit i = s[10+i]  (the banks stay separate)
   <raw>   = raw encoder counts for the link, |e1 - e0|, BEFORE the /(2.467*2)
             conversion -- this is the ground truth for calibrating both the
             divisor and the +25/+30/+85/+95/+104 offsets
   <cm>    = link[path_c][0], the firmware's own converted+offset length, so
             raw and cooked can be compared on one line
   <ch>    = path_append's move char: S/L/R/B, or D when the target was reached
   ---------------------------------------------------------------------------- */
volatile uint32_t tlm_ms = 0;
volatile uint8_t  tlm_fast = 0;   /* 1 ms divider -> tlm_due every 8 ms */
volatile _Bool    tlm_due  = 0;   /* set by the 1 ms tick, cleared by the loop */

static char  tlm_pending[64];
static _Bool tlm_has_pending = 0;

static unsigned tlm_front(void)
{
	int i;
	unsigned m = 0;
	for (i = 0; i < 10; i++) if (s[i])      m |= (1u << i);
	return m;
}

static unsigned tlm_rear(void)
{
	int i;
	unsigned m = 0;
	for (i = 0; i < 8; i++)  if (s[10 + i]) m |= (1u << i);
	return m;
}
#endif


/* ==========================================================================
 * THE BRAIN SEAM
 *
 * This replaces the legacy path_append().  Same call points -- every junction
 * decision in the mission passes through here -- but the decision itself is the
 * brain's now, and the firmware's job is to say what it sees and then do what
 * it is told.  The firmware keeps ownership of everything physical: line
 * following, crossing detection, the target test, the turn primitives.
 * See robot codes/inc/brain.h for the contract.
 * ========================================================================== */

/* The move made at the START of the link we are currently on.  The legacy code
 * got this from path_discoverd[path_c-1]; that log is gone, so it is kept here
 * -- one byte, and it is what the link-length offset below keys off. */
static char s_prev_move = 0;

/*
 * The length of the link just travelled, in MILLIMETRES.
 *
 * The units are the legacy ones and they are NOT cm: the GTD encoders give
 * 2.467 counts per mm per wheel (see parse_telemetry.py), so the differential
 * between the two wheels divided by (2.467*2) is millimetres of travel.
 *
 * The additive terms are EMPIRICAL and are not optional.  The encoder is reset
 * at the top of a link, which is *while the robot is still turning*, so the raw
 * count only covers the part of the link after the sweep finishes.  The offset
 * puts the turn's share back: ~85 mm after a 90-degree turn, ~104 mm after a
 * U-turn (the longest sweep), and ~25 mm after a straight, where there is no
 * turn to lose but a small constant still fits the data.  Without them the
 * brain's dead reckoning walks off the lattice.
 *
 * `prev` is the move made at the START of this link; 0 for the first link,
 * which follows no turn and takes no offset (the legacy did the same).
 *
 * ONE SIMPLIFICATION against the legacy, deliberate: it varied the offset by
 * the move ABOUT TO BE MADE as well (+95/+30 rather than +85/+25) -- a 10 mm
 * refinement.  The brain must be given the distance in order to CHOOSE that
 * move, so the distinction cannot be applied here.  Dropping it is safe: 10 mm
 * is a twentieth of a cell, and the brain rounds to the nearest whole 20 cm.
 */
static int link_mm(char prev)
{
	int d = ab(-GTD[0].IncrementalEncoder + GTD[1].IncrementalEncoder) / (2.467 * 2);

	if      (prev == 'L' || prev == 'R') return d + 85;
	else if (prev == 'F' || prev == 'S') return d + 25;
	else if (prev == 'B')                return d + 104;
	return d;                            /* the first link: no preceding turn */
}

/*
 * Copy one of the brain's plans into a replay buffer.
 *
 * The 'D' appended at the end is NOT padding -- it is the legacy end sentinel,
 * and both replay stages depend on it twice over:
 *
 *   - stage 4's arrival test is `cc == strlen(path)-1`, which has to mean "the
 *     robot is at the destination and only the terminator is left", so the last
 *     byte of the buffer must be consumed as a command;
 *   - the stage dispatches `path[cc-1]` and ends the stage when that byte is
 *     'D'.  Without a terminator the last command would be dispatched and the
 *     stage would then sit waiting for one more junction that never comes, so
 *     the robot would drive past the start and off the field.
 *
 * The brain's own strings are plain NUL-terminated command lists, deliberately
 * (brain.h is the contract, and it says nothing about a sentinel), so the
 * convention is applied here, at the boundary, where the legacy stages can see
 * it -- rather than leaking into the brain.
 */
static void set_plan(char* dst, unsigned cap, const char* src)
{
	unsigned n = 0;

	while (n + 2 < cap && src[n] != '\0') { dst[n] = src[n]; n++; }
	dst[n++] = 'D';
	dst[n]   = '\0';
}

/*
 * Execute one command out of a replay string.  Returns 1 when the string is
 * finished, 0 when the robot should carry on.
 *
 * ONE dispatch for both replay stages and all four of their bank halves.  The
 * legacy repeated this chain four times and knew only 'L'/'R'/'S'/'D' -- it had
 * no 'F' and, more importantly, no 'B', so a plan containing a reversal fell
 * through every branch and the robot simply stopped.  Both are needed now: the
 * brain says 'F' where the legacy said 'S', and the real field's fast path is
 * BFFLRLF.
 *
 * `head` picks the bank: the robot drives with the rear bank leading after a
 * reversal, and the turn primitives exist in both flavours.
 */
static int replay_dispatch(const char* cmds, int idx)
{
	switch (cmds[idx])
	{
	case 'L': if (head) turn_left_r();  else turn_left();  break;
	case 'R': if (head) turn_right_r(); else turn_right(); break;

	case 'F':                                  /* the brain's "carry on"     */
	case 'S':                                  /* the legacy's name for it   */
		if (head) Forward_r(); else Forward();
		break;

	case 'B':
		/* Reverse.  A head flip rather than a 180-degree spin, matching stage 1:
		   the robot drives back with the rear bank leading and nav+=2 records
		   the reversal.  cross is cleared so the drive-on test below does not
		   re-run in this pass. */
		head = (head == 0) ? 1 : 0;
		cross = 0;
		head_delay = 0;
		break;

	case 'D':                                  /* the end sentinel -- see set_plan() */
	default:
		/* Anything else is a bug in whatever wrote the string.  Ending the
		   stage is the safe failure: the robot stops where it is instead of
		   driving on with commands nobody wrote. */
		return 1;
	}
	return 0;
}

/*
 * Report the junction just reached to the brain, and return the move it wants.
 *
 * `target` is passed in rather than read from the global it comes from
 * (OnEndZoon test at the top of stage 1): that variable is a local of the
 * superloop, so it is in scope at the call site and not in here.
 *
 * Returns 'F' / 'L' / 'R' / 'B', or BRAIN_DONE (0) when exploration is complete
 * -- at which point the brain's two plans are copied into the replay buffers
 * that stages 4 and 6 drive.
 */
static char brain_report(int target)
{
	BrainIn in;
	char    move;

	in.left   = (uint8_t)(left_poss  != 0);
	in.right  = (uint8_t)(right_poss != 0);
	in.front  = (uint8_t)((s[3] || s[4] || s[5] || s[6]) ? 1 : 0);
	in.back   = 1;
	in.target = (uint8_t)(target ? 1 : 0);

	/* Distance driven since the previous report.  on_link was set when the
	   encoder was reset at the top of this link, so a closed link is exactly
	   the thing to report; with none closed there is nothing to add. */
	in.dist_cm = (on_link == 1) ? (uint16_t)(link_mm(s_prev_move) / 10) : 0;
	on_link    = 0;

	move = (char)brain_step(&in);

#ifdef USE_MAZE_TELEMETRY
	/* M1: one line per junction.  The first twelve fields are unchanged from
	   the legacy format so the existing parser keeps working; the three new
	   ones are what the brain saw and decided.  `cross` is derived from the
	   move rather than read, because the move is now the input, not the
	   output.  Queued, not sent -- see the note at the telemetry block. */
	{
		BrainStatus st;
		int tlm_raw   = ab(-GTD[0].IncrementalEncoder + GTD[1].IncrementalEncoder);
		int tlm_cross = (move == 'L') ? 1 : (move == 'R') ? 2 : (move == 'B') ? 4 : 0;

		brain_status(&st);
		sprintf(tlm_pending,
		        "J,%lu,%c,%d,%d,%d,%d,%03X,%02X,%d,%d,%d,%d,%u,%u\r\n",
		        (unsigned long)tlm_ms, (move == BRAIN_DONE) ? 'D' : move,
		        nav, head, tlm_raw, link_mm(s_prev_move),
		        tlm_front(), tlm_rear(),
		        in.left, in.right, tlm_cross,
		        in.target, in.dist_cm, st.node);
		tlm_has_pending = 1;
	}
#endif

	if (move == (char)BRAIN_DONE)
	{
		/* Exploration is over.  Hand the two plans to the replay stages: the
		   route home to stage 4, the fast run to stage 6.  Both stages are
		   otherwise untouched -- they always drove exactly these two strings,
		   and still do; only who writes them has changed. */
		set_plan(path_back, sizeof(path_back), brain_home_path());
		set_plan(path_discoverd_s, sizeof(path_discoverd_s), brain_fast_path());

		return BRAIN_DONE;
	}

	s_prev_move = move;
	return move;
}

/* back_home() used to live here: it built the return route by taking the
 * explored path string backwards and inverting every L/R.  That only works for
 * a route that is the reverse of the one driven, and the brain's homeward route
 * is not -- it is the shortest path on the discovered graph, which may leave by
 * a different branch than it entered.  The brain computes it directly
 * (brain_home_path()) and brain_report() copies it into path_back. */

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



/////////////////////////////////////////
void USART1_IRQHandler()
{
  char Data = USART1->RDR;
	if(Data == '2')
	{
	//   loop_start= 2;
	}
  
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
#ifdef USE_MAZE_TELEMETRY
	/* M1 time base.  tlm_ms is the stamp on every line.  tlm_due paces the
	   sensor stream at 125 Hz (every 8 ms), which costs ~3.2 ms of TX per
	   8 ms -- 40% of a 115200 link.  Deliberately NOT the 50 Hz Task20Ms
	   cadence: the branch-detection window is only ~20 ms, so a 20 ms
	   sample period could not resolve it at all. */
	tlm_ms++;
	if (++tlm_fast >= 8) { tlm_fast = 0; tlm_due = 1; }
#endif
	
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
	
	

    BLT_SendData(sprintf(BLT_TX_Buffer,"Hi ,mmdi \r\n"));
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
	


	/* Which replay stage has already consumed its first command.  Was
	   `flag_head`, whose other job -- flipping the robot around on entry to
	   the homeward run -- is gone: the brain's strings are written for the
	   pose the robot is actually in, so there is nothing to flip.  See the
	   entry blocks of stages 4 and 6. */
	int	replay_entered=0;
	

  IR_PWR(1);
	
	while(KEY1==0)
	{
		 if(KEY3)
		 {
					BUZZER(1);
					HAL_Delay(100);
					BUZZER(0);
					HAL_Delay(500);
					StartGyroCalibration();
					BUZZER(1);
					HAL_Delay(50);
					BUZZER(0);
		 }
		 Read_IRSensors();   
			 
	}

	    
	for(int i=0; i<18; i++)
	{
		
		if      (IR_ADC[i] >= IR_mid[i]+100)             s[i]=0;
		else if (IR_ADC[i] <= IR_mid[i]-500)             s[i]=1;
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

#ifdef USE_MAZE_TELEMETRY
		/* M1: exactly one TX slot per 8 ms tick -- a queued event if one is
		   waiting, otherwise a sensor sample. Never both, because a second
		   BLT_SendData() would truncate the first (Hardware.c:135).
		   The pending branch runs even when idle so a latched event cannot sit
		   unsent; only the stream is gated on the run being active. This sits
		   at the top of the loop rather than in a Task*Ms block so it is paced
		   by the 1 ms tick, not by the 20 ms task. */
		if (tlm_due)
		{
			tlm_due = 0;
			if (tlm_has_pending)
			{
				tlm_has_pending = 0;
				sprintf(BLT_TX_Buffer, "%s", tlm_pending);
				BLT_SendData(strlen(BLT_TX_Buffer));
			}
			else if (loop_start != 0)
			{
				sprintf(BLT_TX_Buffer, "S,%lu,%03X,%02X,%d,%d,%d,%d,%d\r\n",
				        (unsigned long)tlm_ms, tlm_front(), tlm_rear(),
				        GTD[0].IncrementalEncoder, GTD[1].IncrementalEncoder,
				        left_poss, right_poss, cross);
				BLT_SendData(strlen(BLT_TX_Buffer));
			}
		}
#endif
	
		/* KEY1 starts the mission.  The brain has to be re-initialised on the
		   PRESS, not while the key is held: this runs on every pass of the
		   superloop and KEY1 is a level, so an unguarded brain_init() here
		   would wipe the discovered map every millisecond the operator leaned
		   on the button.  The edge is detected with a one-shot flag.

		   brain_init() also resets the dead-reckoning frame and both plan
		   buffers, and s_prev_move / cc / replay_entered are the firmware-side
		   state that belongs to a run, so they are cleared here with it. */
		{
			static _Bool key1_last = 0;
			_Bool key1_now = (KEY1 != 0);
			if(key1_now && !key1_last)
			{
				brain_init();
				s_prev_move    = 0;
				cc             = 0;
				replay_entered = 0;
				on_link        = 0;
				loop_start     = 1;
				ti             = 0;
			}
			key1_last = key1_now;
		}
		if(KEY2) calibrat_now=1;
		
		_Bool ir_print = 1;
		if (calibrat_now){
			if(ir_print == 0 ) ir_print =1 ; 
			else ir_print = 0 ;
		}
			
	  //const _Bool OnEndZoon = ( (s[0] && s[1] && s[2] && s[7] && s[8] && s[9]) || 
		//													(s[0] && s[9] && s[10] && s[11] && s[16] && s[17]) );
		
		
		if ((s[1] && s[2] && s[3] && s[4] && s[5] && s[6] && s[7] && s[8]  ) || 
			  (s[10] && s[11] && s[12] && s[13] && s[14] && s[15] && s[16] && s[17])){

		end_zone_timer ++ ;
    
		}
		else{ end_zone_timer = 0 ;}				
		
    const _Bool OnEndZoon = end_zone_timer > 2 ? 1 : 0 ;

#ifdef USE_MAZE_TELEMETRY
		/* M1: log the target-zone latch on change only. On *entry* the 'D' line
		   path_append queues a few lines below overwrites this in the pending
		   slot -- intended, the 'D' is the more informative record. The exit
		   transition is the one this exists for: it says whether the robot
		   began moving off the disc before the zone test could re-arm.
		   Caveat: if a junction 'J' is still unflushed, this can supersede it.
		   Z fires at most twice per run, so that trade is the right way round. */
		{
			static _Bool tlm_zone_last = 0;
			if (OnEndZoon != tlm_zone_last)
			{
				tlm_zone_last = OnEndZoon;
				sprintf(tlm_pending, "Z,%lu,%d\r\n", (unsigned long)tlm_ms, OnEndZoon);
				tlm_has_pending = 1;
			}
		}
#endif		
		
		
		/* The target zone no longer ends the search on sight.  It used to: the
		   legacy explorer stopped the moment it saw the black disc.  The brain
		   must keep going until it can PROVE no undiscovered route is faster,
		   so the target is now just one more input (BrainIn.target) and the
		   move to the drive home comes from BRAIN_DONE instead.
		   Stage 6's arrival at the zone is untouched -- that one really is the
		   end of the mission. */
		if( loop_start==6  && OnEndZoon )
		{	 
			loop_start=7;
		}
		
	
		if(loop_start == 1)                        /////////////////////////// path discover (the brain)
		{
			/* The robot drives until it is standing on a node, tells the brain
			   what it can see, and executes the move it gets back.  The maze
			   reasoning lives in the brain; what is left here is sensing and
			   motion, which is the firmware's job and stays the firmware's.

			   Two things are deliberately NOT the brain's business, because
			   they are properties of the physical robot rather than the maze:
			     - the encoder reset at the top of a link, which is what makes
			       the reported distance mean "since the last junction";
			     - the dead-end debounce, because "the line has ended" is only
			       true once the centre group has been dark for a while. */
			if(roatating==0)
			{
				if(on_link==0)
				{
					GTD[0].ResetEncoder=1;
					GTD[1].ResetEncoder=1;
					on_link=1;
				}

				/* Which exits exist, in the robot's own frame.  The rear bank
				   when head==1, because the robot is driving backwards and its
				   "left" is then the rear row's left.  The centre group is the
				   same test either way: it is the line ahead of whichever end
				   is leading, and brain_report() takes it as BrainIn.front. */
				if(head==0)
				{
					if (s[2]  && (s[3] || s[4] || s[5] || s[6])) {left_poss=1;}
					if (s[7]  && (s[3] || s[4] || s[5] || s[6])) {right_poss=1;}
				}
				else
				{
					if (s[11] && (s[12] || s[13] || s[14] || s[15])) {left_poss=1;}
					if (s[16] && (s[12] || s[13] || s[14] || s[15])) {right_poss=1;}
				}

				/* ARRIVAL.  s[0] and s[9] sit on the rotation axis, so the pair
				   reading black means the robot's centre is over a node --
				   "you are here, decide now".  That holds at every node type,
				   crossing, T and corner alike, which is exactly what the brain
				   needs: it must hear about every junction, or it will never
				   learn a one-sided branch and will claim the search is done
				   with edges missing.  A node with no side branch is a straight
				   passthrough and is NOT reported; the brain reconstructs it
				   from the reported distance.

				   The rear bank has no equivalent pair (only s[10]..s[17] go
				   through the MUX), so the legacy test for the reversed robot
				   is kept here unchanged: either outer rear sensor. */
				int at_node    = (head==0) ? (s[0] && s[9]) : (s[10] || s[17]);
				int centre_dark = (head==0)
					? (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0)
					: (s[12]==0 && s[13]==0 && s[14]==0 && s[15]==0);

				if(centre_dark && !at_node)
				{
					/* No line under the centre and not standing on a node:
					   either mid-link over a gap, or the end of a dead end.
					   Only the second is real, and only once it persists. */
					head_delay++;
				}
				else
				{
					head_delay = 0;
				}

				if( (at_node && (left_poss || right_poss))
				    || (centre_dark && head_delay >= 50) )
				{
					char move = brain_report(OnEndZoon);

					if(move == (char)BRAIN_DONE)
					{
						/* Exploration is provably complete and both plans are
						   in path_back / path_discoverd_s.  Stage 2 used to
						   dump the local map; there is no local map to dump
						   now, so go straight to the wait before going home. */
						loop_start = 3;
					}
					else
					{
#ifdef USE_MAZE_TELEMETRY
						/* BRING-UP PAUSE.  Print the decision, wait five seconds,
						   then make the move -- so an operator watching the
						   Bluetooth log can read what the brain decided, check it
						   against the junction the robot is standing on, and stop
						   the run before the robot commits if it is wrong.  The
						   brain has never run on hardware; this is the only
						   moment where a bad decision is still free.

						   The 'J' line brain_report() queued is flushed HERE and
						   not left for the top-of-loop sender, because that sender
						   only runs once the superloop comes round again -- i.e.
						   after this pause and after the move is already under
						   way.  BLT_SendData is DMA and returns immediately, so
						   the delay doubles as the time the UART needs.

						   The motors are stopped and the stop is PUSHED to the
						   servos before the delay.  The superloop is what keeps
						   re-issuing Motor(left,right,1) and Task10Ms's MotorMove;
						   blocking here suspends both, so without this the servos
						   would hold their last commanded speed for the whole five
						   seconds and the robot would drive on. */
						Motor(0,0,1);
						MotorMove(2);

						{
							BrainStatus st;
							brain_status(&st);
							if (tlm_has_pending)
							{
								tlm_has_pending = 0;
								sprintf(BLT_TX_Buffer, "%s", tlm_pending);
								BLT_SendData(strlen(BLT_TX_Buffer));
							}
							/* B,<ms>,<node>,<x>,<y>,<drift>,<move> */
							sprintf(BLT_TX_Buffer,
							        "B,%lu,%u,%d,%d,%u,%c\r\n",
							        (unsigned long)tlm_ms, st.node,
							        st.x, st.y, st.drift_cm,
							        (move == BRAIN_DONE) ? 'D' : move);
							BLT_SendData(strlen(BLT_TX_Buffer));
						}
						HAL_Delay(5000);
#endif
						/* The one place the brain's decision becomes motion.
						   This is the legacy cross dispatch, unchanged: the
						   brain only replaces the CHOICE of cross. */
						cross = (move=='L') ? 1 : (move=='R') ? 2 : (move=='B') ? 4 : 0;

						roatating  = 1;
						left_poss  = 0;
						right_poss = 0;

						if     (cross==0) { Forward(); }
						else if(cross==1) { turn_left(); }
						else if(cross==2) { turn_right(); }
						else
						{
							/* 'B': reverse.  Done as a head flip rather than a
							   180-degree spin, which is what the legacy did and
							   what the drive-home and fast-run stages expect --
							   the robot drives back with the rear bank leading,
							   and nav+=2 records the reversal. */
							head = (head==0) ? 1 : 0;
							cross = 0;
							head_delay = 0;
							nav+=2;
						}
					}
				}
			}
		}
		else if(loop_start == 2)                        ////////////////////// the map dump used to live here
		{
			/* Stage 2 built the legacy map: it walked link[] and node[] and
			   printed the whole thing over Bluetooth.  Both arrays are gone --
			   the brain owns the map now -- so there is nothing left to dump.
			   The stage is kept (and falls straight through to 3) purely so
			   loop_start numbering, and the bring-up behaviour built around it,
			   do not shift.  If you are looking for the map, it is in the brain:
			   brain_status() reports the node, the dead-reckoned position and
			   the drift. */
			MotorB(0,0);
			loop_start=3;
		}
		else if(loop_start == 3)                        ///////////////////////////////////////////wait for gonig back to statr
		{ 
			if(KEY3) 
			{
			   BLT_SendData(sprintf(BLT_TX_Buffer,"%s  \r\n\n",path_back));
         while(KEY3);
				 loop_start=4;
			}			
		}
		
		else if(loop_start == 4)                        ///////////////////////////////////////////path back
		{
			BUZZER(0);

			if(replay_entered != 4)
			{
				/* ENTRY: consume the plan's FIRST command here, at the junction
				   the robot is already standing on.

				   The brain's commands are "turn now, then drive to the next
				   place you stop", so the turn for THIS junction must not wait
				   for the next one.  The legacy stage drove forward first and
				   dispatched only at the following junction, because back_home()
				   built its string as the reverse of the explored route -- the
				   first entry then described the action at the next node.  The
				   brain's string describes the action HERE, so the ordering has
				   to match it or every command lands one junction late.

				   cc is left at 1 so the ordinary path below dispatches
				   path_back[1] at the next junction, path_back[2] at the one
				   after, and so on.

				   This is the first arm of the if/else-if chain on purpose: on
				   the pass that consumes the entry command the stage must NOT
				   also fall through to the drive-on branch, which would issue a
				   Forward() on top of the turn just made. */
				replay_entered = 4;
				cc = 0;
				roatating = 1;
				if (replay_dispatch(path_back, 0)) loop_start = 5;
				cc = 1;
			}
			else if(head==0)                  ///////// front is front
			{

		  		if (roatating==0)
					{
						    if (s[2]  && (s[3] || s[4] || s[5] ||s[6])) {left_poss=1;}
								if (s[7]  && (s[3] || s[4] || s[5] ||s[6])) {right_poss=1;}


						    if(right_poss || left_poss)
								{


												if(s[1] || s[8])
												{
													cross=1;
													cc++;
											    roatating=1;
													right_poss=0;
													left_poss=0;
												}


							  }

							  if (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0  && (cc ==strlen(path_back)-1))
								{
                    head_delay = 0;
									  cross =1 ;
								  	cc++;
								}
					}



						if(cross==0)
						{
							  roatating=1;
								Forward();
						}
						else if (cross==1)
						{
							roatating=1;

							if (replay_dispatch(path_back, cc-1)) loop_start=5;


						}


			}
			else                              ///////////////////bot algorithm roatate 180 deg
			{
			
				if (roatating==0)
					{
						    if (s[11]  && (s[12] || s[13] || s[14] || s[15])) {left_poss=1;}
								if (s[16]  && (s[12] || s[13] || s[14] || s[15])) {right_poss=1;}
						
						
						    if(right_poss || left_poss)
								{
						
												if((s[10] || s[17]))
												{
													cross=1;         
													cc++;
													roatating=1;
													right_poss=0;
													left_poss=0;
												}		
								}
						
						 	  if (s[13]==0 && s[14]==0 && s[15]==0 && s[12]==0 && (cc ==strlen(path_back)-1))
								{
                    head_delay = 0;
									  cc++;
								  	cross=1; 
								}	
								
								
								
					}
				
						if(cross==0)
						{
							  roatating=1;
								Forward_r();
						}
						else if (cross==1)
						{
							roatating=1;

							if (replay_dispatch(path_back, cc-1)) loop_start=5;
						}

			}
			
		
			
			
			
		}
		else if(loop_start==5)                           //////////////////////////////////////////reach statrt zone
		{
		  	MotorC(0,0);
		  	if(KEY3)  loop_start=6;
		}
		else if(loop_start==6)
		{
			
			
			
			
			if(replay_entered != 6)
			{
				/* ENTRY, exactly as stage 4 -- see the long note there for why the
				   first command is consumed at the junction the robot is already
				   standing on rather than at the next one.

				   NO head flip here either.  The brain's fast-run string assumes
				   the heading the robot is left with after the home run, and its
				   two strings are documented as one continuous command stream
				   (brain.h), so stage 6 must carry straight on from stage 4's
				   final pose.  The legacy flipped `head` because back_home()
				   built the home string as a retrace; the brain's home route is
				   not a retrace, so the flip would now aim the fast run 180
				   degrees wrong at its very first command. */
				replay_entered = 6;
				cc = 0;
				roatating = 1;
				if (replay_dispatch(path_discoverd_s, 0)) loop_start = 7;
				cc = 1;
			}
			else if(head==0)                  ///////// front is front
			{
				
		  		if (roatating==0)
					{
						    if (s[2]  && (s[3] || s[4] || s[5] ||s[6])) {left_poss=1;}
								if (s[7]  && (s[3] || s[4] || s[5] ||s[6])) {right_poss=1;}
						
						
						    if(right_poss || left_poss)
								{
						
												if(s[1] || s[8])
												{
													cross=1;   
													cc++;
											    roatating=1;
													right_poss=0;
													left_poss=0;
												}

							  }
	         
//							  if (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0  && (cc == strlen(path_discoverd_s)-1))
//								{
//                    head_delay = 0;
//									  cross =1 ;
//								  	cc++;
//								}	
					}
				
				
				
						if(cross==0)
						{
							  roatating=1;
								Forward();
						}
						else if (cross==1)
						{
							roatating=1;

							if (replay_dispatch(path_discoverd_s, cc-1)) loop_start=7;

						}


			}
			else                              ///////////////////bot algorithm roatate 180 deg
			{
			
				if (roatating==0)
					{
						    if (s[11]  && (s[12] || s[13] || s[14] || s[15])) {left_poss=1;}
								if (s[16]  && (s[12] || s[13] || s[14] || s[15])) {right_poss=1;}
						
						
						    if(right_poss || left_poss)
								{
						
												if((s[10] || s[17]))
												{
													cross=1;         
													cc++;
													roatating=1;
													right_poss=0;
													left_poss=0;
												}		
								}
						
//						 	  if (s[13]==0 && s[14]==0 && s[15]==0 && s[12]==0 && (cc ==strlen(path_discoverd_s)))
//								{
//                    head_delay = 0;
//									  cc++;
//								  	cross=1; 
//								}	
								
								
								
					}
				
						if(cross==0)
						{
							  roatating=1;
								Forward_r();
						}
						else if (cross==1)
						{
							roatating=1;

							if (replay_dispatch(path_discoverd_s, cc-1)) loop_start=7;
						}

			}
			
		
			
			
		}
		
		else if(loop_start==7)                           //////////////////////////////////////////reach statrt zone
		{
		  	MotorC(0,0);
		  	if(KEY3)  loop_start=8;
		}
		
		
		Motor(left,right,1);
		
		
		
		/////////////////////////////////////////////////////////////////////////////
		///////////////////////////// Task 2 ms (500Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task2Ms > 19)
		{
			Task2Ms -= 20;
		
		  MotorA(lef,righ);
			
			for(int i=0; i<18; i++)
			{
					if (IR_ADC[i] >= IR_mid[i] + 50) 
					{
							s_current[i] = 0;
					}
					else if (IR_ADC[i] <= IR_mid[i] - 50)
					{
							s_current[i] = 1;
					}

					if (s_current[i] != s[i])
					{
							s_counter[i]++;

							if (s_current[i] == 1 && s_counter[i] > OffToOnTrsh)
							{
									s[i] = 1;
									s_counter[i] = 0;
							}
							else if (s_current[i] == 0 && s_counter[i] > OnToOffTrsh)
							{
									s[i] = 0;
									s_counter[i] = 0;
							}
					}
					else
					{
							s_counter[i] = 0;
					}
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
			
			if      (nav >=4) nav-=4;
			else if (nav<=-1) nav+=4;

			/* A reverse-collapsing pass over path_discoverd_s used to run here at
			   10 Hz, rewriting 'LBR'->'B', 'LBS'->'R', 'SBS'->'B' and friends in
			   place while the robot drove, indexed by PathIndex.  It is gone:
			   path_discoverd_s is no longer grown move by move, it is the brain's
			   fast-run plan written once at BRAIN_DONE, and PathIndex went with
			   the legacy string machinery.  Rewriting it here would corrupt the
			   plan, so the pass must not come back. */
			
			
		}
		
		 
      
		
		
		/////////////////////////////////////////////////////////////////////////////
		//////////////////////////// Task 1000 ms (1Hz) /////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		if(Task1000Ms > 999)
		{
			Task1000Ms -= 1000;
	

		}
		
		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////
		Pr = 0 ;
		if(Pr > 50)
		{
								Pr = 0;
									//BLT_SendData(sprintf(BLT_TX_Buffer,"%d\r\n",loop_start));
							//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d\r\n",GTD[0].IncrementalEncoder));
							//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d,%d,%d\r\n",KEY1,KEY2,KEY3));
							//	BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%d\r\n",GTD[0].AbsoluteEncoder,GTD[0].IncrementalEncoder));
						  //BLT_SendData(sprintf(BLT_TX_Buffer,"[%6.1f,%6d][%6.1f,%6d]\r\n",GTD[0].AbsoluteEncoder,GTD[0].IncrementalEncoder,GTD[1].AbsoluteEncoder,GTD[1].IncrementalEncoder));
							//BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%.1f,%.1f\r\n",Gyro_Z,Z_Angle,Gyro_Z_Offset));
							//	BLT_SendData(sprintf(BLT_TX_Buffer,"%.2f,%.2f,%.2f \r\n",Z_AngleSetPoint,Z_Angle,Error));
						//	  	BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u\r\n",
						//			IR_ADC[0],IR_ADC[1]	,IR_ADC[2]	,IR_ADC[3]	,IR_ADC[4]	,IR_ADC[5]	,IR_ADC[6]	,IR_ADC[7]	,IR_ADC[8]
						//		,	IR_ADC[9],IR_ADC[10],IR_ADC[11]	,IR_ADC[12]	,IR_ADC[13]	,IR_ADC[14]	,IR_ADC[15]	,IR_ADC[16]	,IR_ADC[17]));
							 
								 // BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u \r\n",IR_ADC[3],IR_max[3],IR_min[3],IR_mid[3],s[3]));
								 // BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u \r\n",IR_ADC[1],IR_max[1],IR_min[1],IR_mid[1],s[1]));
			       // BLT_SendData(sprintf(BLT_TX_Buffer,"%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u,%4u%4u,%4u,%4u \r\n",IR_mid[0],IR_mid[1],IR_mid[2],
			     //   IR_mid[3],IR_mid[4],IR_mid[5],IR_mid[6],IR_mid[7],IR_mid[8],IR_mid[9],IR_mid[10],IR_mid[11],IR_mid[12],IR_mid[13],
			      //  IR_mid[14],IR_mid[15],IR_mid[16],IR_mid[17]));
			
							//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d  ,%d    ,%d,%d,%d,%d,   %d,  %d    ,%d,%d,%d,%f,%d \r\n",s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],left_poss,cross,roatating,Error,step));
							//	BLT_SendData(sprintf(BLT_TX_Buffer,"%d  ,%d,%d,%d,%d,%d,%d,  %d\r\n",s[10],s[11],s[12],s[13],s[14],s[15],s[16],s[17]));
							// 	BLT_SendData(sprintf(BLT_TX_Buffer,"cross=%d  ,step=%d,head=%d,roat=%d,   %.1f,   %.1f\r\n",cross,step,head , roatating,Z_Angle,Z_AngleSetPoint));
							//		BLT_SendData(sprintf(BLT_TX_Buffer,"%.1f,%.1f\r\n",Z_Angle,Z_AngleSetPoint));
							//	if(loop_start==1)
							//	{
										// BLT_SendData(sprintf(BLT_TX_Buffer,"%s  \r\n",path_discoverd));
					//				BLT_SendData(sprintf(BLT_TX_Buffer,"cross=%d  ,roat=%d, LP=%d ,RP=%d , H=%d , path=%s    ,L:%d,R:%d ,on_link=%d  , nav=%d  , LL=%d \r\n"
					//				,cross,roatating,left_poss,right_poss ,head,path_discoverd ,-GTD[0].IncrementalEncoder,GTD[1].IncrementalEncoder,on_link,nav,link[path_c-1][0]));
						


      //      BLT_SendData(sprintf(BLT_TX_Buffer,"0:%d , 1:%d , 2:%d , 3:%d , 4:%d , 5:%d , 6:%d , 7:%d , 8:%d , 9:%d , 10:%d ,  \r\n",link[0][0]
				//		,link[1][0],link[2][0],link[3][0],link[4][0],link[5][0],link[6][0],link[7][0],link[8][0],link[9][0],link[10][0]));
			//	if(ir_print){
							//		BLT_SendData(sprintf(BLT_TX_Buffer,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d   \r\n",s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8]
						//			,s[9],s[10],s[11],s[12],s[13],s[14],s[15],s[16],s[17]));
			//}
							
							//	}
							//	else
							//	{
						//					 BLT_SendData(sprintf(BLT_TX_Buffer,"LS=%d  , H=%d ,cc= %d  \r\n",loop_start, head , cc));

							//	}
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
