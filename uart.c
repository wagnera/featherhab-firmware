#include "stm32f0xx_hal.h"

#include "uart.h"
#include "config.h"
#include "gpio.h"

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
uint8_t uart_initted = 0;

void uart_init(void)
{
    __GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // Init gpio pins for uart
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Init UART periph
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
    HAL_UART_Init(&huart1);

    HAL_Delay(100);
	uint8_t switch_baud[] = "$PUBX,41,1,0003,0001,115200,0*1E\r\n";
	HAL_UART_Transmit(uart_gethandle(), switch_baud, sizeof(switch_baud)/sizeof(uint8_t), 1000);

    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = 115200;
    HAL_UART_Init(&huart1);


//
//    __DMA1_CLK_ENABLE();
//
//  // Init UART DMA
//    hdma_usart1_rx.Instance = DMA1_Channel3;
//    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
//    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
//    HAL_DMA_Init(&hdma_usart1_rx);
//
//    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
//
//    hdma_usart1_tx.Instance = DMA1_Channel2;
//    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart1_tx.Init.MemInc = DMA_MINC_DISABLE;
//    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
//    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
//    HAL_DMA_Init(&hdma_usart1_tx);
//
//    __HAL_LINKDMA(&huart1,hdmatx,hdma_usart1_tx);
//
////    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
////    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    uart_initted = 1;
}

void uart_deinit(void)
{
    if(uart_initted == 1)
    {
        //HAL_DMA_DeInit(&hdma_usart1_rx);
        //HAL_DMA_DeInit(&hdma_usart1_tx);
        HAL_UART_DeInit(&huart1);
        __HAL_RCC_USART1_CLK_DISABLE();

        uart_initted = 0;
    }
}

UART_HandleTypeDef* uart_gethandle(void)
{
    return &huart1;
} 

DMA_HandleTypeDef* uart_get_txdma_handle(void)
{
    return &hdma_usart1_tx;
}
DMA_HandleTypeDef* uart_get_rxdma_handle(void)
{
    return &hdma_usart1_rx;
}

