#include <math.h>
uint8_t CHAR_BUFF_SIZE = 5;

static char * float_to_char(float x, char *p) {
    char *s = p + CHAR_BUFF_SIZE; // go to end of buffer
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    int n_dec = 3; // number of decimal places
    if (x < 0) { // take care of negative numbers
        int mul_factor = pow(10, n_dec - 1);
        decimals = (int)(x * -mul_factor) % mul_factor; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    }
    /*else if (x < 0){
        n_dec--;
        int mul_factor = pow(10, n_dec - 1);
        decimals = (int)(x * -mul_factor) % mul_factor; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    }
    else if (x < 1){
        n_dec--;
        int mul_factor = pow(10, n_dec - 1);
        decimals = (int)(x * mul_factor) % mul_factor;
        units = (int)x;
    }*/
    else { // positive numbers
        int mul_factor = pow(10, n_dec - 1);
        decimals = (int)(x * mul_factor) % mul_factor;
        units = (int)x;
    }

    *--s = (decimals % 10) + '0';
    if (n_dec - 2 > 0)
    {
        int i = 0;
        for (i = 0; i < n_dec - 2; i++)
        {
            decimals /= 10; // repeat for as many decimal places as you need
            *--s = (decimals % 10) + '0';
        }
    }
    *--s = '.';

    while (units > 0) {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    if (x < 0) *--s = '-'; // unary minus sign for negative numbers
    if (x > 0 && x < 1) *--s = '0';
    return s;
}
