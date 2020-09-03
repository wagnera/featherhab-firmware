#ifndef __usart_H
#define __usart_H

//#include "stm32f0xx_hal.h"

void uart_init(void);
void uart_deinit(void);
UART_HandleTypeDef* uart_gethandle(void);
DMA_HandleTypeDef* uart_get_txdma_handle(void);
DMA_HandleTypeDef* uart_get_rxdma_handle(void);

#endif 
