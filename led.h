#ifndef _LED_H_INCLUDED_
#define _LED_H_INCLUDED_

#include <stdint.h>

#define LED_RCC     RCC_GPIOB
#define LED_GPIO    GPIOB
#define LED_PIN     GPIO0

#define LED_ON      gpio_set(LED_GPIO, LED_PIN)
#define LED_OFF     gpio_clear(LED_GPIO, LED_PIN)

#endif
