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
#include <libopencm3/cm3/nvic.h>
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


volatile bool send_aprs_check = false;

uint32_t last_aprs = APRS_TRANSMIT_PERIOD;
uint32_t last_gps  = GPS_PARSE_PERIOD;

// TODO: Leave GPS on for 10 minutes at the start, then only turn it on a bit before each transmission


uint32_t aprs_period = 60000;

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

static void setupTimers(void)
{
    rcc_periph_clock_enable(RCC_TIM3);
    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ,1);
    TIM_CNT(TIM3) = 1;
    TIM_PSC(TIM3) = 16000;
    TIM_ARR(TIM3) = 1000;
    TIM_DIER(TIM3) |= TIM_DIER_UIE;
    //TIM_CR1(TIM3) |= TIM_CR1_CEN;

    rcc_periph_clock_enable(RCC_TIM14);
    nvic_enable_irq(NVIC_TIM14_IRQ);
    nvic_set_priority(NVIC_TIM14_IRQ,2);
    TIM_CNT(TIM14) = 1;
    TIM_PSC(TIM14) = 64000;
    TIM_ARR(TIM14) = 1506;
    TIM_DIER(TIM14) |= TIM_DIER_UIE;
    TIM_CR1(TIM14) |= TIM_CR1_CEN;

}

void tim3_isr(void)
{
    //systick_setup(1000);
    //gpio_toggle(GPIOB,GPIO0);

    parse_gps_transmission();

    if(afsk_request_cwoff())
    {
        si446x_cw_off();
        si446x_shutdown();
    }
    //systick_stop();
    TIM_SR(TIM3) &= ~TIM_SR_UIF;
}

void tim14_isr(void)
{
    //gpio_toggle(GPIOB,GPIO0);
    send_aprs_check = true;
    TIM_SR(TIM14) &= ~TIM_SR_UIF;
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
    //sleep_init();
    

    adc_init();

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);

    systick_setup(1000);

    si446x_setup();

    //delay(500); //wait a bit before turning on GPS
    gps_init();

    afsk_init();
    usart_init();
    configLED();
    parse_gps_transmission();
    setupTimers();
    systick_stop();
    sleep_init();


   // gps_poweroff();



    // LED_ON;
    // delay(400);
    // LED_OFF;
    // delay(4000);
    while(1)
    {

        // if (send_aprs_check)
        // {
        //      systick_setup(1000);
        //      gpio_toggle(GPIOB,GPIO0);
        //      //LED_ON;
        //      //aprs_send();
        //      //LED_OFF;
        //      send_aprs_check = false;
        //      systick_stop();
        // }

       // pwr_set_stop_mode();
        sleep_now();
      // clockenable();
    }
    
}

// vim:softtabstop=4 shiftwidth=4 expandtab 
