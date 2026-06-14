#ifndef __SERIAL_H
#define __SERIAL_H


#include "main.h"



#define UART_BUFFER_SIZE			500

#define	GTD_TX_DMA_CNT  			DMA1_Channel4->CNDTR
#define	GTD_RX_DMA_CNT  			(UART_BUFFER_SIZE - DMA1_Channel5->CNDTR)

#define	BLT_TX_DMA_CNT  			DMA1_Channel2->CNDTR


#endif /* __SERIAL_H */



















