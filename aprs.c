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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "config.h"
#include "aprs.h"
#include "gps.h"
#include "adc.h"
#include "ax25.h"
#include "float2char.h"
#include "str2float.h"


int32_t meters_to_feet(int32_t m)
{
  // 10000 ft = 3048 m
  return (float)m / 0.3048;
}

void aprs_send(void)
{

    struct s_address addresses_pathless[] = { 
        {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
        {"", S_CALLSIGN_ID},  // Source callsign (-11 = balloon, -9 = car)
      };

    struct s_address addresses_wide[] = { 
        {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
        {"", S_CALLSIGN_ID},  // Source callsign (-11 = balloon, -9 = car)
        //{S_CALLSIGN, S_CALLSIGN_ID},
        {DIGI_PATH1, DIGI_PATH1_TTL},
      };

    char* altitude = get_gpsaltitude();
    if (altitude[0] != 0x00)
    {
      if (str2float(altitude) * 3.2808399 > PATH_ALT_THRES)
      {
        strncpy(addresses_pathless[1].callsign, S_CALLSIGN, 7);
        // emz: modified this to get the size of the first address rather than the size of the struct itself, which fails
        ax25_send_header(addresses_pathless, sizeof(addresses_pathless)/sizeof(addresses_pathless[0]));
      }
      else
      {
        strncpy(addresses_wide[1].callsign, S_CALLSIGN, 7);
        // emz: modified this to get the size of the first address rather than the size of the struct itself, which fails
        ax25_send_header(addresses_wide, sizeof(addresses_wide)/sizeof(addresses_wide[0]));
      }
    }
    else
    {
      strncpy(addresses_wide[1].callsign, S_CALLSIGN, 7);
     	// emz: modified this to get the size of the first address rather than the size of the struct itself, which fails
      ax25_send_header(addresses_wide, sizeof(addresses_wide)/sizeof(addresses_wide[0]));
    }

  /*struct s_address addresses[] = { 
        {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
        {"", S_CALLSIGN_ID},  // Source callsign (-11 = balloon, -9 = car)
      };
  strncpy(addresses[1].callsign, S_CALLSIGN, 7);
     	// emz: modified this to get the size of the first address rather than the size of the struct itself, which fails
  ax25_send_header(addresses, sizeof(addresses)/sizeof(addresses[0]));*/

  char* dayofmonth = get_dayofmonth();
  if (dayofmonth[0] != 0x00)
  {
    ax25_send_byte('/');                // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
    // ax25_send_string("021709z");     // 021709z = 2nd day of the month, 17:09 zulu (UTC/GMT)
    ax25_send_string(get_dayofmonth()); ///! Needs to be day hour minute        // 170915 = 17h:09m:15s zulu (not allowed in Status Reports)
    ax25_send_string(get_timestamp()); 
    ax25_send_byte('z'); // zulu time. h for nonzulu
  }
  else
  {
    ax25_send_byte('!'); //realtime if no timestamp 
  }
  
  char* lattitude_check = get_latitudeTrimmed();
  char* longitude_check = get_longitudeTrimmed();
  if (lattitude_check[0] != 0x00 && longitude_check[0] != 0x00)
  {
    //ax25_send_string("4215.37");//get_latitudeTrimmed());     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
    ax25_send_string(get_latitudeTrimmed());     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
    ax25_send_byte('N');
    ax25_send_byte('/');                // Symbol table
    ax25_send_string(get_longitudeTrimmed());     // Lon: 000deg and 25.80 min
    ax25_send_byte('W');
    ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
  }
  else
  {
    ax25_send_string("0000.00");     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
    ax25_send_byte('N');
    ax25_send_byte('\\');                // Symbol table
    ax25_send_string("00000.00");     // Lon: 000deg and 25.80 min
    ax25_send_byte('W');
    ax25_send_byte('.');              //unknown/indeterminate position symbol
  }
  


  
  char* course = get_course();
  if (course[0] != 0x00)
  {
    // TODO: ENSURE THAT THE COURSE IS FORMATTED CORRECTLY!
    ax25_send_string(get_course());             // Course (degrees)
    ax25_send_byte('/');                // and
    ax25_send_string(get_speedKnots());             // speed (knots)
  }
  else
  {
    ax25_send_string("...");             // Course (degrees)
    ax25_send_byte('/');                // and
    ax25_send_string("...");             // speed (knots)
  }
  
  //char* altitude = get_gpsaltitude();
  if (altitude[0] != 0x00)
  {
    int alt_in_ft = atoi(altitude) * 3.2808399;
    ax25_send_string("/A=");            // Altitude (feet). Goes anywhere in the comment area
    char alt_field[10];
    itoa(alt_in_ft, alt_field, 10);
    uint8_t zero_pad;
    for (zero_pad = 6 - strlen(alt_field); zero_pad > 0; zero_pad--)
    {
      ax25_send_byte('0');
    }
    ax25_send_string(alt_field);
  }
  //No else, don't bother sending altitude if we don't have it
  


  //ADC Temperature
  char temp[10];
  itoa(adc_gettemp(), temp, 10);
  ax25_send_string(" T=");
  ax25_send_string(temp);

  //Number of satellites
  ax25_send_string(" Sats=");
  char* n_sats = get_sv();
  if (n_sats[0] == 0x00)
  {
    ax25_send_string("00");
  }
  else
  {
    ax25_send_string(get_sv());
  }

  //Battery/Solar Voltage  
  char temp2[4] = "";
  char* voltage_string = float_to_char(adc_getsolar(), temp2);
  ax25_send_string(" V=");
  ax25_send_string(voltage_string);

  //Comment
  ax25_send_string(" /ul-aprs flight 2");
  ax25_send_byte(' ');
  
  #define COMMENTBUFFER_SIZE 128
  //char commentBuffer[COMMENTBUFFER_SIZE];
  //ax25_send_string(slavesensors_getAPRScomment(commentBuffer, COMMENTBUFFER_SIZE));
  
  ax25_send_footer();
  ax25_flush_frame();
}

// vim:softtabstop=4 shiftwidth=4 expandtab 
