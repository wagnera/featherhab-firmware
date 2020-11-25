/*
 * Master Firmware: NMEA Parser
 *
 * This file is part of OpenTrack.
 *
 * OpenTrack is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OpenTrack is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with OpenTrack. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Ethan Zonca
 * Matthew Kanning
 * Kyle Ripperger
 * Matthew Kroening
 *
 */


#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>

// Hardware config
#define GPS_SCL_PORT GPIOB
#define GPS_SCL_PIN GPIO6
#define GPS_SCL_AF GPIO_AF1

#define GPS_SDA_PORT GPIOB
#define GPS_SDA_PIN GPIO7
#define GPS_SDA_AF GPIO_AF1

#define GPS_ONOFF_PORT GPIOA
#define GPS_ONOFF_PIN GPIO1
#define GPS_ONOFF GPS_ONOFF_PORT, GPS_ONOFF_PIN

// Messages (REMOVEME?)
#define GGA_MESSAGE
#define RMC_MESSAGE
#define UKN_MESSAGE

void gps_poweron(void);
void gps_poweroff(void);
void gps_init(void);
void gps_sendubx(uint8_t* data, uint8_t size);
char* get_longitudeTrimmed(void);
char* get_longitudeLSBs(void);
char* get_latitudeTrimmed(void);
char* get_latitudeLSBs(void);
char* get_timestamp(void);
char* get_gpsaltitude(void);
char* get_speedKnots(void);
char* get_course(void);
char* get_hdop(void);
char* get_sv(void);
char* get_dayofmonth(void);
uint8_t gps_hasfix(void);
void parse_gps_transmission(void);
void XORbyteWithChecksum(uint8_t byte);

#endif /* GPS_H_ */
