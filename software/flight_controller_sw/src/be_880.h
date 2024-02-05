#ifndef BE_880   /* prevent circular inclusions */
#define BE_880

#include "xuartlite.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"

#define TEST_BUFFER_SIZE        2048

void uart_be_init();
void uart_loopback_test();
s32 uart_be_intr_init(XScuGic *gic);
void test();
void SendHandler(void *CallBackRef, unsigned int EventData);
void RecvHandler(void *CallBackRef, unsigned int EventData);
#endif  /* end of protection macro */
