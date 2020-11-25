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

#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "usart.h"
#include "gps.h"
#include "config.h"


void usart_init(void)
{
    // Init clocks
    rcc_periph_clock_enable(RCC_USART1);

    // GPS FET control
    gpio_mode_setup(GPS_ONOFF_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPS_ONOFF_PIN);
    gpio_set_output_options(GPS_ONOFF_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, GPS_ONOFF_PIN);
    gps_poweron();

    // USART TX
    gpio_mode_setup(DEBUG_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, DEBUG_TX_PIN);
    gpio_set_output_options(DEBUG_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, DEBUG_TX_PIN);
    gpio_set_af(DEBUG_TX_PORT, DEBUG_TX_AF, DEBUG_TX_PIN);

    // USART RX
    gpio_mode_setup(DEBUG_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, DEBUG_RX_PIN);
    gpio_set_output_options(DEBUG_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, DEBUG_RX_PIN);
    gpio_set_af(DEBUG_RX_PORT, DEBUG_RX_AF, DEBUG_RX_PIN);

    // USART Config
    usart_set_baudrate(DEBUG_USART, DEBUG_BAUDRATE);
    usart_set_databits(DEBUG_USART, 8);
    usart_set_parity(DEBUG_USART, USART_PARITY_NONE);
    usart_set_stopbits(DEBUG_USART, USART_STOPBITS_1);
    usart_set_mode(DEBUG_USART, USART_MODE_TX_RX);
    usart_set_flow_control(DEBUG_USART, USART_FLOWCONTROL_NONE);

    // Enable interrupts
    //usart_enable_rx_interrupt(DEBUG_USART);
    //nvic_enable_irq(DEBUG_IRQ);

    // Enable USART
    usart_enable(DEBUG_USART);

}

void serial0_sendString(const char* string) {
	while(*string != 0x00)
	{
		usart_send_blocking(USART1, *string);
		string++;
	}
}


// vim:softtabstop=4 shiftwidth=4 expandtab 
