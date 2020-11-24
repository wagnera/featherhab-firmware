#ifndef USART_H
#define USART_H

#define DEBUG_USART USART1
#define DEBUG_IRQ NVIC_USART1_IRQ

#define DEBUG_TX_PORT GPIOA
#define DEBUG_TX_PIN GPIO2
#define DEBUG_TX_AF GPIO_AF1

#define DEBUG_RX_PORT GPIOA
#define DEBUG_RX_PIN GPIO3
#define DEBUG_RX_AF GPIO_AF1

void usart_init(void);

#endif

