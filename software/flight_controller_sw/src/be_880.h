#ifndef BE_880   /* prevent circular inclusions */
#define BE_880

#include "xuartlite.h"
#include "xparameters.h"
#include "xil_printf.h"

void uart_be_init();
void read_be_data();

#endif  /* end of protection macro */
