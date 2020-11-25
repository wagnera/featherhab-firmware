#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

#include "i2c.h"
#include "gps.h"

static void i2c_setup(void)
{
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
	//rcc_set_i2c_clock_hsi(I2C1);

	i2c_reset(I2C1);
	gpio_mode_setup(GPS_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPS_SCL_PIN);
	gpio_set_af(GPS_SCL_PORT, GPS_SCL_AF, GPS_SCL_PIN);
	gpio_mode_setup(GPS_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPS_SDA_PIN);
	gpio_set_af(GPS_SDA_PORT, GPS_SDA_AF, GPS_SDA_PIN);
	i2c_peripheral_disable(I2C1);

	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, 0);
	/* HSI is at 8Mhz */
	i2c_set_speed(I2C1, 0, 8);
	//configure No-Stretch CR1 (only relevant in slave mode)
	//i2c_enable_stretching(I2C1);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C1);
	i2c_peripheral_enable(I2C1);
}