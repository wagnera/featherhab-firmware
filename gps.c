//
// GPS: communicate with ublox GPS module via ubx protocol
//

#include "stm32f0xx_hal.h"

#include "config.h"
#include "gpio.h"
#include "uart.h"
#include "gps.h"


volatile gps_data_t position;
uint8_t gpson = 0;

// Private methods
static void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb);
static uint8_t _gps_verify_checksum(uint8_t* data, uint8_t len);


// Poll for fix data from the GPS and update the internal structure
void gps_update_data(void)
{
	// Error!
	if(!gpson)
	{
		led_blink(5);
		return;
	}

    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0xFF, 0xFF};


    volatile uint8_t check_a = 0;
    volatile uint8_t check_b = 0;
    for(uint8_t i = 2; i<6; i++)
    {
    	check_a += request[i];
    	check_b += check_a;
    }
    request[6] = check_a;
    request[7] = check_b;

    uint8_t flushed = uart_gethandle()->Instance->RDR;
    HAL_UART_Transmit(uart_gethandle(), request, 8, 100);



    // Get the message back from the GPS
    uint8_t buf[100];
    for(uint8_t i=0; i<100; i++)
    	buf[i] = 0xaa;
    volatile HAL_StatusTypeDef res = HAL_UART_Receive(uart_gethandle(), buf, 100, 3000);

    // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
    if( !_gps_verify_checksum(&buf[2], 96) )
        led_blink(2);


    //volatile uint32_t gpstime_ms = (buf[6+0] << 24) | (buf[6+1] << 16) | buf[6+2] << 8) | (buf[6+3]);

    position.month = buf[6+6];
    position.day = buf[6+7];
    position.hour = buf[6+8];
    position.minute = buf[6+9];
    position.second = buf[6+10];
    position.valid = buf[6+11] & 0b1111;
    position.fixtype = buf[6+20];

    position.sats_in_solution = buf[6+23];

    position.longitude = (buf[6+24] << 0) | (buf[6+25] << 8) | (buf[6+26] << 16) | (buf[6+27] << 24); // degrees
    position.latitude =  (buf[6+28] << 0) | (buf[6+29] << 8) | (buf[6+30] << 16) | (buf[6+31] << 24); // degrees

    position.altitude = (buf[6+36] << 0) | (buf[6+37] << 8) | (buf[6+38] << 16) | (buf[6+39] << 24); // mm above sealevel
    position.altitude /= 1000; // mm => m

    position.speed = (buf[6+60] << 0) | (buf[6+61] << 8) | (buf[6+62] << 16) | (buf[6+63] << 24); // mm/second
    position.speed /= 1000; // mm/s -> m/s
    
    position.pdop = (buf[6+76] << 0) | (buf[6+77] << 8);
    position.pdop /= 100; // scale to dop units

    position.heading = (buf[6+84] << 0) | (buf[6+85] << 8) | (buf[6+86] << 16) | (buf[6+87] << 24); // mm above sealevel
    position.heading /= 100000; // 1e-5

//    // Return the value if GPSfixOK is set in 'flags'
//    if( buf[17] & 0x01 )
//        *lock = buf[16];
//    else
//        *lock = 0;

}

// TODO: Add data valid flag: invalidate data when GPS powered off


// Verify that the uBlox 6 GPS receiver is set to the <1g airborne navigaion mode.
uint8_t gps_check_nav(void)
{
    uint8_t request[8] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00,
        0x2A, 0x84};
    uint8_t flushed = uart_gethandle()->Instance->RDR;
    HAL_UART_Transmit(uart_gethandle(), request, 8, 100);

    // Get the message back from the GPS
    uint8_t buf[44];
    HAL_UART_Receive(uart_gethandle(), buf, 44, 100);

//    // Verify sync and header bytes
//    if( buf[0] != 0xB5 || buf[1] != 0x62 )
//        led_set(LED_RED, 1);
//    if( buf[2] != 0x06 || buf[3] != 0x24 )
//        led_set(LED_RED, 1);

    // Check 40 bytes of message checksum
//    if( !_gps_verify_checksum(&buf[2], 40) ) led_set(LED_RED, 1);

    // Clock in and verify the ACK/NACK
    uint8_t ack[10];
//    for(uint8_t i = 0; i < 10; i++)
//        ack[i] = _gps_get_byte();

    HAL_UART_Receive(uart_gethandle(), ack, 10, 100);

    // If we got a NACK, then return 0xFF
    if( ack[3] == 0x00 ) return 0xFF;

    // Return the navigation mode and let the caller analyse it
    return buf[8];
}


// Verify the checksum for the given data and length.
static uint8_t _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return 0;
    else
        return 1;
}


// Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
static void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}


// Power on GPS module and initialize UART
void gps_poweron(void)
{
    // NOTE: pchannel
    HAL_GPIO_WritePin(GPS_NOTEN, 0);
    uart_init();



	// Disable messages
	uint8_t setGGA[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XFF, 0X23};
	HAL_UART_Transmit(uart_gethandle(), setGGA, sizeof(setGGA)/sizeof(uint8_t), 100);
	HAL_Delay(100);

	uint8_t ackbuffer[10];
	for(uint8_t i=0; i<10; i++)
		ackbuffer[i] = 0xaa;
	HAL_UART_Receive(uart_gethandle(), ackbuffer, 10, 100);

	uint8_t setZDA[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X08, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X07, 0X5B};
	HAL_UART_Transmit(uart_gethandle(), setZDA, sizeof(setZDA)/sizeof(uint8_t), 100);
	HAL_Delay(100);

	uint8_t setGLL[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X01, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X2A};
	HAL_UART_Transmit(uart_gethandle(), setGLL, sizeof(setGLL)/sizeof(uint8_t), 100);
	HAL_Delay(100);

	uint8_t setGSA[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X02, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X01, 0X31};
	HAL_UART_Transmit(uart_gethandle(), setGSA, sizeof(setGSA)/sizeof(uint8_t), 100);
	HAL_Delay(100);

	uint8_t setGSV[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X03, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X02, 0X38};
	HAL_UART_Transmit(uart_gethandle(), setGSV, sizeof(setGSV)/sizeof(uint8_t), 100);
	HAL_Delay(100);

	uint8_t setRMC[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X04, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X03, 0X3F};
	HAL_UART_Transmit(uart_gethandle(), setRMC, sizeof(setRMC)/sizeof(uint8_t), 100);
	HAL_Delay(100);

	uint8_t setVTG[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X05, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X04, 0X46};
	HAL_UART_Transmit(uart_gethandle(), setVTG, sizeof(setRMC)/sizeof(uint8_t), 100);
	HAL_Delay(100);


//    // Disable GLONASS mode
//    uint8_t disable_glonass[20] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x8F, 0xB2};
//
//    //gps_sendubx(disable_glonass, 20);
//    volatile HAL_StatusTypeDef res = HAL_UART_Transmit(uart_gethandle(), disable_glonass, 20, 100);
//
//    // Enable power saving
//    uint8_t enable_powersave[10] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
//    //gps_sendubx(enable_powersave, 10);
//    res = HAL_UART_Transmit(uart_gethandle(), enable_powersave, 10, 100);
//
//
//    // Set dynamic model 6 (<1g airborne platform)
//    uint8_t airborne_model[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
//    //gps_sendubx(airborne_model, sizeof(airborne_model)/sizeof(uint8_t));
//    res = HAL_UART_Transmit(uart_gethandle(), airborne_model, sizeof(airborne_model)/sizeof(uint8_t), 100);





    // Begin DMA reception
    //HAL_UART_Receive_DMA(uart_gethandle(), nmeaBuffer, NMEABUFFER_SIZE);

    gpson = 1;
}


// Power off GPS module
void gps_poweroff(void)
{
    // NOTE: pchannel
//	position.hour = 0;
//	position.minute = 0;
//	position.second = 0;
//	position.altitude = 0;
//	position.latitude = 0;
//	position.longitude = 0;
//	position.day = 0;
//	position.month = 0;
//	position.fixtype = 0;
//	position.valid = 0;
	position.pdop = 0;
	position.sats_in_solution = 0;
//	position.speed = 0;

    uart_deinit();
    HAL_GPIO_WritePin(GPS_NOTEN, 1);
    gpson = 0;
}

gps_data_t* gps_getdata(void)
{
    return &position;
}

uint8_t gps_ison(void)
{
    return gpson;
}

// vim:softtabstop=4 shiftwidth=4 expandtab 
