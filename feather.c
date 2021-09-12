/*
 * FeatherHAB 
 *
 * This file is part of FeatherHAB.
 *
 * FeatherHab is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatherHab is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with FeatherHAB. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Ethan Zonca
 *
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>

#include "config.h"
#include "gps.h"
#include "usart.h"
#include "afsk.h"
#include "aprs.h"
#include "adc.h"
#include "delay.h"
#include "sleep.h"
#include "si446x.h"
#include "led.h"

static void clockdisable(void)
{
    // Enable clocks 
    rcc_periph_clock_disable(RCC_GPIOB);
    rcc_periph_clock_disable(RCC_GPIOA);
//    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_disable(RCC_TIM1);
    rcc_periph_clock_disable(RCC_SPI1);
 
}
static void clockenable(void)
{
    rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);
	rcc_set_sysclk_source(RCC_HSI);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_prefetch_enable();
	flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);

	/* 8MHz * 12 / 2 = 48MHz */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL4);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 16000000;
	rcc_ahb_frequency = 16000000;

    // Enable clocks 
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOA);
//    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_SPI1);
 
}

static void configLED(void)
{
    /* setup LED */
    rcc_periph_clock_enable(LED_RCC);
    gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

int main(void)
{
    // Init clocks

    clockenable();

    rtc_init();
    sleep_init();
    

    adc_init();

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);

    systick_setup(1000);

    si446x_setup();

    //delay(500); //wait a bit before turning on GPS
    gps_init();

    afsk_init();
    usart_init();
    configLED();

    // Main loop
    uint32_t last_aprs = APRS_TRANSMIT_PERIOD;
    uint32_t last_gps  = GPS_PARSE_PERIOD;
    
    // TODO: Leave GPS on for 10 minutes at the start, then only turn it on a bit before each transmission

    
    uint32_t aprs_period = 60000;

    LED_ON;
    delay(400);
    LED_OFF;
    delay(4000);
    while(1)
    {
        // Transmit very often for the first 60 minutes
        if(get_millis_elapsed() > START_UP_PERIOD)
        {
            aprs_period = APRS_TRANSMIT_PERIOD;
        }
        else
        {
            if (gps_hasfix())
           {
                int ii;
                for (ii=0; ii < 10; ii++)
                {
                    LED_ON;
                    delay(500);
                    LED_OFF;
                    delay(100);
                }       
           }
        }
        
            

        if(get_millis_elapsed() - last_aprs > aprs_period) 
        {
            /*int ii;
            for (ii=0; ii < 20; ii++)
            {
                LED_ON;
                delay(30);
                LED_OFF;
                delay(30);
            }*/
            aprs_send();
            last_aprs = get_millis_elapsed();
        }

        if(get_millis_elapsed() - last_gps > GPS_PARSE_PERIOD)
        {
            parse_gps_transmission();
            last_gps = get_millis_elapsed();
        }

        if(afsk_request_cwoff())
        {
            si446x_cw_off();
            si446x_shutdown();
        }

        if (gps_hasfix())
        {
            /*int ii;
            for (ii=0; ii < 10; ii++)
            {
                LED_ON;
                delay(500);
                LED_OFF;
                delay(100);
            }*/        
        }
        else
        {
            /*LED_ON;
            delay(1000);
            LED_OFF;
            delay(1000);*/
        }

       // pwr_set_stop_mode();
        sleep_now();
      // clockenable();
    }
}

// vim:softtabstop=4 shiftwidth=4 expandtab 
