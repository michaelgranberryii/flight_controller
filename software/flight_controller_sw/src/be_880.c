#include "be_880.h"

u8 SendBuffer[TEST_BUFFER_SIZE];
u8 ReceiveBuffer[TEST_BUFFER_SIZE];

static volatile int TotalReceivedCount;
static volatile int TotalSentCount;

XUartLite_Config *be_880_config;
XUartLite be_880;

void be_880_init(){
	be_880_config = XUartLite_LookupConfig(XPAR_UARTLITE_0_DEVICE_ID);
	int status = XUartLite_CfgInitialize(&be_880,be_880_config, be_880_config->RegBaseAddr);

	if((status) == XST_SUCCESS)
		xil_printf("UART INIT SUCCESSFUL\n\r");
	else
		xil_printf("UART INIT FAILED\n\r");
	status = XUartLite_SelfTest(&be_880);
	if(status == XST_SUCCESS) {
		xil_printf("UART SELF TEST SUCCUSSFUL\n\r");
	} else {
		xil_printf("UART SELF TEST FAILED\n\r");
	}
}

s32 be_880_interrupt_init(XScuGic *gic) {
	int Status;
	XScuGic_SetPriorityTriggerType(gic, XPAR_FABRIC_UARTLITE_0_VEC_ID, 0xA0, 0x3);
	Status = XScuGic_Connect(gic, XPAR_FABRIC_UARTLITE_0_VEC_ID,
				 (Xil_ExceptionHandler) XUartLite_InterruptHandler,
				 &be_880);
	if (Status != XST_SUCCESS) {
		return Status;
	}
	XScuGic_Enable(gic, XPAR_FABRIC_UARTLITE_0_VEC_ID);


	XUartLite_SetSendHandler(&be_880, be_880_send_handler, &be_880);
	XUartLite_SetRecvHandler(&be_880, be_880_recv_handler, &be_880);

	/*
	 * Enable the interrupt of the UartLite so that interrupts will occur.
	 */
	XUartLite_EnableInterrupt(&be_880);
	XUartLite_Recv(&be_880, ReceiveBuffer, TEST_BUFFER_SIZE);
	return XST_SUCCESS;
}

void be_880_uart_loopback_test() {
	int Index;
	/*
	 * Initialize the send buffer bytes with a pattern to send and the
	 * the receive buffer bytes to zero to allow the receive data to be
	 * verified.
	 */
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
//		SendBuffer[Index] = Index;
		ReceiveBuffer[Index] = 0;
	}

	/*
	 * Start receiving data before sending it since there is a loopback.
	 */
	XUartLite_Recv(&be_880, ReceiveBuffer, TEST_BUFFER_SIZE);

	/*
	 * Send the buffer using the UartLite.
	 */
	XUartLite_Send(&be_880, SendBuffer, TEST_BUFFER_SIZE);

	/*
	 * Wait for the entire buffer to be received, letting the interrupt
	 * processing work in the background, this function may get locked
	 * up in this loop if the interrupts are not working correctly.
	 */
	while ((TotalReceivedCount != TEST_BUFFER_SIZE)
	       || (TotalSentCount != TEST_BUFFER_SIZE)) {
	}


	/*
	 * Verify the entire receive buffer was successfully received.
	 */
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
		if (ReceiveBuffer[Index] != SendBuffer[Index]) {
			return XST_FAILURE;
		}
	}

}

void be_880_send_handler(void *CallBackRef, unsigned int EventData)
{
	TotalSentCount = EventData;
	xil_printf("Called SendHandler: %d\n\r", TotalSentCount);

}

void be_880_recv_handler(void *CallBackRef, unsigned int EventData)
{
	TotalReceivedCount = EventData;
	xil_printf("Called RecvHandler: %d\n\r", TotalSentCount);
	xil_printf("ReceiveBuffer: \n\r %s\n\r", ReceiveBuffer);
}
