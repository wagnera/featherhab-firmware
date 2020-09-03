#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>


typedef struct _gps_data
{
    uint32_t pdop;
    uint8_t sats_in_solution;
    int32_t speed;
    int32_t heading;

    int32_t latitude;
    int32_t longitude;
    int32_t altitude;

    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint8_t valid;
    uint8_t fixtype;

} gps_data_t;


void gps_update_data(void);

uint8_t gps_check_nav(void);


void gps_poweron(void);
void gps_poweroff(void);

void gps_acquirefix(void);
uint8_t gps_getstate(void);

gps_data_t* gps_getdata(void);
uint8_t gps_ison(void);

#endif /* GPS_H_ */
