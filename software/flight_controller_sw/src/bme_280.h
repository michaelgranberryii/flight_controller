#ifndef BME_280   /* prevent circular inclusions */
#define BME_280

#include "xiic.h"

// Default I2C address for the BME_280
#define BME_280_ADDRESS                        	0x68

#define BME_280_HUM_LSB                   		0xFE
#define BME_280_HUM_MSB                   		0xFD

void iic_bme_init();
void iic_bme_setup();
void read_bme();

#endif  /* end of protection macro */
