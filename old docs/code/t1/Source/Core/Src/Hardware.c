#include "Hardware.h"
#include "Serial.h"

///////// Serial 
extern	char                 	BLT_TX_Buffer[UART_BUFFER_SIZE];
extern	volatile 	char 				BLT_RX_Buffer[UART_BUFFER_SIZE];
extern	char                 	GTD_TX_Buffer[UART_BUFFER_SIZE];
extern	volatile 	char 				GTD_RX_Buffer[UART_BUFFER_SIZE];

extern	uint16_t							adcv[4],IR_ReadCounter,IR_ADC[18],IR_Logic[18];

extern	uint16_t 							RGB_Data[100];


void	ADC_Init(void)
{
	/*Pin Configuration*/
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER |= ( (uint32_t) ( (0x00000003 << 10) | (0x00000003 << 12) ) );  //PA5,6 Analog mode
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	GPIOB->MODER |= ( (uint32_t) ( (0x0003) | (0x0003 << 2) ) );  //PB0,1 Analog mode
	
	/* Clock Source configuration*/
	RCC->APBENR2 |= RCC_APBENR2_ADCEN;  //Enable ADC Clock
	
	RCC->CCIPR &=~RCC_CCIPR_ADCSEL;  		//ADCs clock source selection: System clock = 64MHz
	
	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;  	//Asynchronous clock mode
	//ADC1->CFGR2 |= ADC_CFGR2_CKMODE_0;
	
	ADC1->CFGR1 |=  (0x00     ) |     									/* Select 12 bit resoulation */
	                ADC_CFGR1_CONT |   									/* Select Continuous conversion mode */
                  ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;	/*Enable DMA transfer on ADC and circular mode*/

	ADC1->CHSELR = ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL8 | ADC_CHSELR_CHSEL9 ; /* Channels Selection */
  ADC1->SMPR = 0x00000007;              /* Sample Rate  111: 160.5 ADC clock cycles  */
  //ADC->CCR |= ADC_CCR_VREFEN;       /* Enable Vrefint */
	//ADC1->IER = ADC_IER_EOCIE;// | ADC_IER_EOSEQIE | ADC_IER_OVRIE; /* Enable Interrupts */ 
	//NVIC_EnableIRQ(ADC1_IRQn);     /* enable DMA1 Channel1 Interrupt   */
	
  /* ADC Calibrate befor DMA config and Enable ADC*/
	//ADC1->CR |= ADC_CR_ADCAL;
	//while (ADC1->CR & ADC_CR_ADCAL);
	
	/* Enable DMA */	
	/* (1) Enable the peripheral clock on DMA */
	/* (3) Configure the peripheral data register address */
	/* (4) Configure the memory address */
	/* (5) Configure the number of DMA tranfer to be performs on DMA channel 1 */
	/* (6) Configure increment, size, interrupts and circular mode */
	/* (7) Enable DMA Channel 1 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;                  /* (1) */
	
	DMAMUX1_Channel2->CCR = 0x00000005; //Request MUX Input: ADC
	DMAMUX1_RequestGenStatus->RGCFR = 0;
	
	DMA1_Channel3->CPAR = (uint32_t) (&(ADC1->DR));    /* (3) */
	DMA1_Channel3->CMAR = (uint32_t)(adcv);            /* (4) */
	DMA1_Channel3->CNDTR = 4;                          /* (5) */
	DMA1_Channel3->CCR |= DMA_CCR_MINC  | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC; /* (6) */
	//DMA1_Channel3->CCR |= (DMA_CCR_PL);
  DMA1_Channel3->CCR |= DMA_CCR_EN; /* (7) */
  /* Enable ADC */
	ADC1->CR |= ADC_CR_ADEN; 
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
	
	
	ADC1->CR |= ADC_CR_ADSTART;  /* Start the ADC conversion */
}


void	USART1_Init (void)
{
	int i;

	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	GPIOB->MODER &= (uint32_t)~( ((uint32_t)0x03 << 12) | ((uint32_t)0x03 << 14) );  //PB6,7 Clear
	GPIOB->MODER |= (uint32_t) ( ((uint32_t)0x02 << 12) | ((uint32_t)0x02 << 14) );  //PB6,7 Alternate function mode
	
	GPIOB->AFR[0] &= (uint32_t)~( ((uint32_t)0xFF << 24) ); //PB6,7 AF Clear
	
	RCC->APBENR2 |= RCC_APBENR2_USART1EN;    // enable USART1 clock
	
	USART1->BRR   = 0x022C;                  // 115200 baud @ PCLK2 64MHz
	USART1->CR1   = 0;                       // Clear Control Register 1
	USART1->CR1   = USART_CR1_TE  | USART_CR1_RE;     // enable TX & RX

	
	USART1->CR3  |= USART_CR3_DMAT | USART_CR3_OVRDIS;        // Enable TX DMA Bit
	for (i = 0; i < 0x1000; i++) __NOP();    // avoid unwanted output

	USART1->CR1  |= USART_CR1_UE;// | USART_CR1_RXNEIE_RXFNEIE;   // Enable USART1 & RX Interrupts
	//NVIC_EnableIRQ(USART1_IRQn);                   /* USART2  RX Interrupt enable */
}


void	DMA1_CH2_Init(void)
{
	DMA1_Channel2->CCR  = 0; 	/* DMA Channel 1 Disable  */

	RCC->AHBENR |= RCC_AHBENR_DMA1EN; 	/* enable peripheral clock for DMA1 */

	DMAMUX1_Channel1->CCR = 0x00000033; //Request MUX Input: USART1_TX
	DMAMUX1_RequestGenStatus->RGCFR = 0;
	//DMAMUX1_ChannelStatus->CSR = 0;
	//DMAMUX1_ChannelStatus->CFR = 0;
	//DMAMUX1_RequestGenStatus->RGSR = 0;

	DMA1_Channel2->CMAR  = (uint32_t) &USART1->TDR;    	/* set chn1 memory address    */
	DMA1_Channel2->CPAR  = (uint32_t) BLT_TX_Buffer;    		/* set chn1 peripheral address*/
	DMA1_Channel2->CNDTR = sizeof(UART_BUFFER_SIZE)-1;     		/* transmit size word         */
	
	/* configure DMA channel      */
	DMA1_Channel2->CCR	= 	(0<<14) |       //Memory to memory mode Enable
													(0<<12) |       //Channel priority level 0:Low 1:Medium 2:High 3:Very high
													(0<<10) |       //Memory size 0:8bit 1:16bit 2:32bit
													(0<< 8) |       //Peripheral size 0:8bit 1:16bit 2:32bit
													(0<< 7) |       //Memory increment mode Disable
													(1<< 6) |       //Peripheral increment mode Enable
													(0<< 5) |       //Circular mode Disable
													(0<< 4) |       //Data transfer direction  0: Read from peripheral  1: Read from memory
													(0<< 3) |       //Transfer error interrupt Disable
													(0<< 2) |       //Half transfer interrupt Disable
													(0<< 1) |       //Transfer complete interrupt Enable
													(0<< 0) ;       //Channel Disable


	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	//NVIC_EnableIRQ(DMA1_Channel2_IRQn);     /* enable DMA1 Channel1 Interrupt   */
	DMA1_Channel2->CCR  |= DMA_CCR_EN;     /* DMA Channel 1 enable  */
}

void	BLT_SendData(int length)
{
	DMA1_Channel2->CCR  = 0;
	DMA1_Channel2->CNDTR = length;              /* transmit size word          */
	DMA1_Channel2->CCR   = DMA_CCR_PINC | DMA_CCR_EN;       //Peripheral increment mode Enable
}


void	USART2_Init(void)
{
	int i;

	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER &= (uint32_t)~( ((uint32_t)0x03 << 2) | ((uint32_t)0x03 << 4) | ((uint32_t)0x03 << 6) );  //PA1,2,3 Clear
	GPIOA->MODER |= (uint32_t) ( ((uint32_t)0x02 << 2) | ((uint32_t)0x02 << 4) | ((uint32_t)0x02 << 6) );  //PA1,2,3 Alternate function mode
	
	GPIOA->AFR[0] &= (uint32_t)~( ((uint32_t)0xFFF << 4) ); //PA1,2,3 AF Clear (DE,TX,RX)
	GPIOA->AFR[0] |= (uint32_t) ( ((uint32_t)0x111 << 4) ); //PA1,2,3 AF 1 for USART2	(DE,TX,RX)
	
	RCC->APBENR1 |= RCC_APBENR1_USART2EN;    // enable USART1 clock
	
	USART2->BRR   = 0x022C;                  // 115200 baud @ PCLK2 64MHz
	USART2->CR1   = 0;                       // Clear Control Register 1
	USART2->CR1   = USART_CR1_TE  | USART_CR1_RE;     // enable TX & RX

	
	USART2->CR3  |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;        // Enable TX DMA Bit & RX DMA Bit
	for (i = 0; i < 0x1000; i++) __NOP();    // avoid unwanted output
	
	
	/*DE Pin Config for RS485*/
	USART2->CR1   |=	USART_CR1_DEAT_0 |	//	time between the activation of the DE signal and the beginning of the start bit
										USART_CR1_DEDT_0;		//	time between the end of the last stop bit
	USART2->CR3		|= 	USART_CR3_DEM;			//	DE function is enabled
	USART2->CR3		&= ~USART_CR3_DEP;			//	DE signal is active high
	
	/*Enable USART*/
	USART2->CR1  |= USART_CR1_UE;// | USART_CR1_RXNEIE_RXFNEIE;   // Enable USART2 & RX Interrupts
	//NVIC_EnableIRQ(USART2_IRQn);                   /* USART2  RX Interrupt enable */
}


void	DMA1_CH4_Init(void)
{
	DMA1_Channel4->CCR  = 0; 	/* DMA Channel 1 Disable  */

	RCC->AHBENR |= RCC_AHBENR_DMA1EN; 	/* enable peripheral clock for DMA1 */

	DMAMUX1_Channel3->CCR = 53; //Request MUX Input: USART2_TX
	DMAMUX1_RequestGenStatus->RGCFR = 0;
	//DMAMUX1_ChannelStatus->CSR = 0;
	//DMAMUX1_ChannelStatus->CFR = 0;
	//DMAMUX1_RequestGenStatus->RGSR = 0;

	DMA1_Channel4->CMAR  = (uint32_t) &USART2->TDR;    	/* set chn1 memory address    */
	DMA1_Channel4->CPAR  = (uint32_t) GTD_TX_Buffer;    		/* set chn1 peripheral address*/
	DMA1_Channel4->CNDTR = sizeof(UART_BUFFER_SIZE)-1;     		/* transmit size word         */
	
	/* configure DMA channel      */
	DMA1_Channel4->CCR	= 	(0<<14) |       //Memory to memory mode Enable
													(0<<12) |       //Channel priority level 0:Low 1:Medium 2:High 3:Very high
													(0<<10) |       //Memory size 0:8bit 1:16bit 2:32bit
													(0<< 8) |       //Peripheral size 0:8bit 1:16bit 2:32bit
													(0<< 7) |       //Memory increment mode Disable
													(1<< 6) |       //Peripheral increment mode Enable
													(0<< 5) |       //Circular mode Disable
													(0<< 4) |       //Data transfer direction  0: Read from peripheral  1: Read from memory
													(0<< 3) |       //Transfer error interrupt Disable
													(0<< 2) |       //Half transfer interrupt Disable
													(0<< 1) |       //Transfer complete interrupt Enable
													(0<< 0) ;       //Channel Disable


	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	//NVIC_EnableIRQ(DMA1_Channel4_IRQn);     /* enable DMA1 Channel1 Interrupt   */
	DMA1_Channel4->CCR  |= DMA_CCR_EN;     /* DMA Channel 1 enable  */
}

void	GTDMotor_SendData(int length)
{
	DMA1_Channel4->CCR  = 0;
	DMA1_Channel4->CNDTR = length;              /* transmit size word          */
	DMA1_Channel4->CCR   = DMA_CCR_PINC | DMA_CCR_EN;       //Peripheral increment mode Enable
}

void 	DMA1_CH5_Init(void)
{
	DMA1_Channel5->CCR  = 0;     /* DMA Channel 2 Disable  */

	RCC->AHBENR |= RCC_AHBENR_DMA1EN; 	/* enable peripheral clock for DMA1 */

	DMAMUX1_Channel4->CCR = 52; 	//Request MUX Input: USART2_RX
	DMAMUX1_RequestGenStatus->RGCFR = 0;
	//DMAMUX1_ChannelStatus->CSR = 0;
	//DMAMUX1_ChannelStatus->CFR = 0;
	//DMAMUX1_RequestGenStatus->RGSR = 0;

	DMA1_Channel5->CMAR  = (uint32_t) GTD_RX_Buffer;    /* set chn1 memory address    */
	DMA1_Channel5->CPAR  = (uint32_t) &USART2->RDR;    /* set chn1 peripheral address*/
	DMA1_Channel5->CNDTR = UART_BUFFER_SIZE;         /* transmit size word         */
	
	/* configure DMA channel      */
	DMA1_Channel5->CCR	= 	(0<<14) |       //Memory to memory mode Enable
													(0<<12) |       //Channel priority level 0:Low 1:Medium 2:High 3:Very high
													(0<<10) |       //Memory size 0:8bit 1:16bit 2:32bit
													(0<< 8) |       //Peripheral size 0:8bit 1:16bit 2:32bit
													(1<< 7) |       //Memory increment mode Disable
													(0<< 6) |       //Peripheral increment mode Disable
													(1<< 5) |       //Circular mode Disable
													(0<< 4) |       //Data transfer direction  0: Read from peripheral  1: Read from memory
													(0<< 3) |       //Transfer error interrupt Disable
													(0<< 2) |       //Half transfer interrupt Disable
													(0<< 1) |       //Transfer complete interrupt Disable
													(0<< 0) ;       //Channel Disable


	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	//NVIC_EnableIRQ(DMA1_Channel4_IRQn);     /* enable DMA1 Channel1 Interrupt   */
	DMA1_Channel5->CCR  |= DMA_CCR_EN;     /* DMA Channel 1 enable  */
}

void	TIM14_Init(void) 
{
  RCC->APBENR2 |= RCC_APBENR2_TIM14EN;          /* enable clock for TIM14   */
  TIM14->PSC  = 0;
  TIM14->CR1  = 0;
  TIM14->PSC   = ( 63 );                        /* set prescaler   = 1MHz   */
  TIM14->ARR   = ( 999 );                       /* set auto-reload = 1ms    */

	TIM14->DIER = TIM_DIER_UIE;                   /* Update Interrupt enable  */
  NVIC_EnableIRQ(TIM14_IRQn);                   /* TIM14   Interrupt enable */

  TIM14->CR1  |= TIM_CR1_CEN;                   /* timer enable             */
}


void	TIM17_Init (void)
{
	RCC->APBENR2 |= RCC_APBENR2_TIM17EN;          /* enable clock for TIM17   */
	TIM17->PSC  = 0;
	TIM17->CR1  = 0;

	TIM17->PSC = 0;
	TIM17->ARR = 6399; 														//100us AutoReload
	TIM17->DIER = TIM_DIER_UIE;                   /* Update Interrupt enable */
	NVIC_EnableIRQ(TIM17_IRQn);       	  				/* TIM17   Interrupt enable */

	TIM17->CR1  |= TIM_CR1_CEN;                   /* timer enable            */
}


void	TIM16_Init(void) 
{
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	GPIOB->MODER &= (uint32_t)~( ((uint32_t)0x03 << 16) );  //PB8 Clear
	GPIOB->MODER |= (uint32_t) ( ((uint32_t)0x02 << 16) );  //PB8 Alternate function mode
	GPIOB->OTYPER |= 0x0100; //B8 Open Drain
	GPIOB->AFR[1] &= (uint32_t)~( ((uint32_t)0x0000000F ) ); //PB8 AF Clear
	GPIOB->AFR[1] |= (uint32_t) ( ((uint32_t)0x00000002 ) ); //PB8 AF2 for TIM16_CH1
	
	RCC->APBENR2 |= RCC_APBENR2_TIM16EN;
	
	TIM16->PSC = 0;    	// 0+1=1 -> CK_INT=64MHZ
  TIM16->ARR = 71;   	// for 888 KHz
  
	TIM16->CCR1 = 0; 	// Duty cicle channel 1

		
	// TIM16->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // Compare Mode OC1
	//// TIM1->CCMR2 |= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1; // Compare Mode OC4
	TIM16->CCMR1 = 0x006C; // PWM mode 1 , OCxPE=1 , OCxFE=1 , CC1 channel is configured as output	
  TIM16->CCMR2 = 0x0000; // 
	
	TIM16->CCER = TIM_CCER_CC1E ;  // OC1 Enable , OC1 Active High
	
	TIM16->CR2	= 0;
	TIM16->BDTR |=  TIM_BDTR_MOE;
	
	TIM16->DIER = TIM_DIER_CC1DE; 	/* DMA request enable for OC1 & OC2 */
	////NVIC_EnableIRQ(TIM1_CC_IRQn);    							/* TIM1 Interrupt enable */
	////NVIC_SetPriority(TIM1_CC_IRQn,0);
	TIM16->CR1 |= TIM_CR1_CEN ;
}

void	DMA1_CH1_Init(void)
{
	DMA1_Channel1->CCR  = 0; 	/* DMA Channel 1 Disable  */

	RCC->AHBENR |= RCC_AHBENR_DMA1EN; 	/* enable peripheral clock for DMA1 */

	DMAMUX1_Channel0->CCR = 44; //Request MUX Input: TIM16_CH1
	DMAMUX1_RequestGenStatus->RGCFR = 0;
	//DMAMUX1_ChannelStatus->CSR = 0;
	//DMAMUX1_ChannelStatus->CFR = 0;
	//DMAMUX1_RequestGenStatus->RGSR = 0;

	DMA1_Channel1->CMAR  = (uint32_t) &TIM16->CCR1;    	/* set chn1 memory address    */
	DMA1_Channel1->CPAR  = (uint32_t) RGB_Data;    		/* set chn1 peripheral address*/
	DMA1_Channel1->CNDTR = 73;     		/* transmit size word         */
	
	/* configure DMA channel      */
	DMA1_Channel1->CCR	= 	(0<<14) |       //Memory to memory mode 			0:Disable 1:Enable
													(3<<12) |       //Channel priority level 			0:Low 1:Medium 2:High 3:Very high
													(1<<10) |       //Memory size 								0:8bit 1:16bit 2:32bit
													(1<< 8) |       //Peripheral size 						0:8bit 1:16bit 2:32bit
													(0<< 7) |       //Memory increment mode 			0:Disable 1:Enable
													(1<< 6) |       //Peripheral increment mode 	0:Disable 1:Enable
													(0<< 5) |       //Circular mode 							0:Disable 1:Enable
													(0<< 4) |       //Data transfer direction  		0:Read from peripheral  1: Read from memory
													(0<< 3) |       //Transfer error interrupt 		0:Disable 1:Enable
													(0<< 2) |       //Half transfer interrupt 		0:Disable 1:Enable
													(0<< 1) |       //Transfer complete interrupt 0:Disable 1:Enable
													(0<< 0) ;       //Channel 										0:Disable 1:Enable


	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	//NVIC_EnableIRQ(DMA1_Channel1_IRQn);     /* enable DMA1 Channel1 Interrupt   */
	DMA1_Channel1->CCR  |= DMA_CCR_EN;     /* DMA Channel 1 enable  */
}


void 	RGB_SendData(void)
{
	for(uint8_t i=0;i<NUM_OF_LED_BIT;i++)
	{
		if(RGB_Data[i] < RGB_PWM_0_BIT) RGB_Data[i] = RGB_PWM_0_BIT;
	}
	
	RGB_Data[0] = 0;
	RGB_Data[NUM_OF_LED_BIT] = 0;
	
	DMA1_Channel1->CCR  = 0;
	DMA1_Channel1->CNDTR = NUM_OF_LED_BIT+2;              /* transmit size word          */
	DMA1_Channel1->CCR	= DMA_CCR_PL | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_PINC | DMA_CCR_EN;
	/*
													(0<<14) |       //Memory to memory mode Enable
													(3<<12) |       //Channel priority level 0:Low 1:Medium 2:High 3:Very high
													(1<<10) |       //Memory size 0:8bit 1:16bit 2:32bit
													(1<< 8) |       //Peripheral size 0:8bit 1:16bit 2:32bit
													(0<< 7) |       //Memory increment mode Disable
													(1<< 6) |       //Peripheral increment mode Enable
													(0<< 5) |       //Circular mode 0:Disable 1:Enable
													(0<< 4) |       //Data transfer direction  0: Read from peripheral  1: Read from memory
													(0<< 3) |       //Transfer error interrupt Disable
													(0<< 2) |       //Half transfer interrupt Disable
													(0<< 1) |       //Transfer complete interrupt Enable
													(1<< 0) ;       //Channel Disable 0x3541
	*/
}


/*
	Color: 	CLEAR, GREEN ,RED ,BLUE ,BLUE_LIGHT ,PINK ,YELLOW ,WHITE
	Brightness: 0~100
*/

const uint8_t	COLORS_GREEN[]={0		,150	,0		,0		,100	,0		,200	,150};
const uint8_t	COLORS_RED[]	={0		,0		,255	,0		,0		,255	,150	,150};
const uint8_t	COLORS_BLUE[]	={0		,0		,0		,255	,255	,255	,0		,150};

void 	Set_RGB(uint8_t NumOfLED,uint8_t	Color,uint8_t Brightness)
{
	uint8_t	StartAdd=0,EndAdd=0;
	
	StartAdd = NumOfLED * 24;
	EndAdd = StartAdd + 8;
	uint8_t BitCut = 0x80;
	
	uint8_t Green = COLORS_GREEN[Color] * (Brightness / 100.0);
	uint8_t Red 	= COLORS_RED[Color] 	* (Brightness / 100.0);
	uint8_t Blue 	= COLORS_BLUE[Color] 	* (Brightness / 100.0);
	
	for(uint8_t	i=StartAdd;i<EndAdd;i++)
	{
		RGB_Data[i+1] = 	( (Green 	& BitCut) ? (RGB_PWM_1_BIT) : (RGB_PWM_0_BIT) );
		RGB_Data[i+8+1] =	( (Red 		& BitCut) ? (RGB_PWM_1_BIT) : (RGB_PWM_0_BIT) );
		RGB_Data[i+16+1] =( (Blue 	& BitCut) ? (RGB_PWM_1_BIT) : (RGB_PWM_0_BIT) );
		BitCut >>=1;
	}
}



void	Read_IRSensors(void)
{
	IR_ADC[9] = adcv[0];
	IR_ADC[0] = adcv[2];
	
	if(IR_ReadCounter == 0)
	{
		IR_ADC[5] = adcv[3];
		IR_ADC[14] = adcv[1];
	}
	else if(IR_ReadCounter == 1)
	{
		IR_ADC[4] = adcv[3];
		IR_ADC[13] = adcv[1];
	}
	else if(IR_ReadCounter == 2)
	{
		IR_ADC[3] = adcv[3];
		IR_ADC[12] = adcv[1];
	}
	else if(IR_ReadCounter == 3)
	{
		IR_ADC[6] = adcv[3];
		IR_ADC[15] = adcv[1];
	}
	else if(IR_ReadCounter == 4)
	{
		IR_ADC[7] = adcv[3];
		IR_ADC[16] = adcv[1];
	}
	else if(IR_ReadCounter == 5)
	{
		IR_ADC[1] = adcv[3];
		IR_ADC[10] = adcv[1];
	}
	else if(IR_ReadCounter == 6)
	{
		IR_ADC[8] = adcv[3];
		IR_ADC[17] = adcv[1];
	}
	else if(IR_ReadCounter == 7)
	{
		IR_ADC[2] = adcv[3];
		IR_ADC[11] = adcv[1];
	}
	
	
	IR_ReadCounter++;
	if(IR_ReadCounter > 7)
	{
		IR_ReadCounter = 0;
	}
	
	MUX_A(IR_ReadCounter & 0x01);
	MUX_B(IR_ReadCounter & 0x02);
	MUX_C(IR_ReadCounter & 0x04);
}




