#ifndef BE_880   /* prevent circular inclusions */
#define BE_880

#include "xuartlite.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"

#define TEST_BUFFER_SIZE        2048

void be_880_init();
void be_880_uart_loopback_test();
s32 be_880_interrupt_init(XScuGic *gic);
void be_880_send_handler(void *CallBackRef, unsigned int EventData);
void be_880_recv_handler(void *CallBackRef, unsigned int EventData);
#endif  /* end of protection macro */
