#include "be_880.h"

u8 SendBuffer[TEST_BUFFER_SIZE];
u8 ReceiveBuffer[TEST_BUFFER_SIZE];

static volatile int TotalReceivedCount;
static volatile int TotalSentCount;

XUartLite_Config *uart_config1;
XUartLite uart1;

void uart_be_init(){
	uart_config1 = XUartLite_LookupConfig(XPAR_UARTLITE_0_DEVICE_ID);
	int status = XUartLite_CfgInitialize(&uart1,uart_config1, uart_config1->RegBaseAddr);

	if((status) == XST_SUCCESS)
		xil_printf("UART INIT SUCCESSFUL\n\r");
	else
		xil_printf("UART INIT FAILED\n\r");
	status = XUartLite_SelfTest(&uart1);
	if(status == XST_SUCCESS) {
		xil_printf("UART SELF TEST SUCCUSSFUL\n\r");
	} else {
		xil_printf("UART SELF TEST FAILED\n\r");
	}
}

s32 uart_be_intr_init(XScuGic *gic) {
	int Status;
	XScuGic_SetPriorityTriggerType(gic, XPAR_FABRIC_UARTLITE_0_VEC_ID, 0xA0, 0x3);
	Status = XScuGic_Connect(gic, XPAR_FABRIC_UARTLITE_0_VEC_ID,
				 (Xil_ExceptionHandler) XUartLite_InterruptHandler,
				 &uart1);
	if (Status != XST_SUCCESS) {
		return Status;
	}
	XScuGic_Enable(gic, XPAR_FABRIC_UARTLITE_0_VEC_ID);


	XUartLite_SetSendHandler(&uart1, SendHandler, &uart1);
	XUartLite_SetRecvHandler(&uart1, RecvHandler, &uart1);

	/*
	 * Enable the interrupt of the UartLite so that interrupts will occur.
	 */
	XUartLite_EnableInterrupt(&uart1);
	return XST_SUCCESS;
}

void uart_loopback_test() {
	int Index;
	/*
	 * Initialize the send buffer bytes with a pattern to send and the
	 * the receive buffer bytes to zero to allow the receive data to be
	 * verified.
	 */
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
		SendBuffer[Index] = Index;
		ReceiveBuffer[Index] = 0;
	}

	/*
	 * Start receiving data before sending it since there is a loopback.
	 */
	XUartLite_Recv(&uart1, ReceiveBuffer, TEST_BUFFER_SIZE);

	/*
	 * Send the buffer using the UartLite.
	 */
	XUartLite_Send(&uart1, SendBuffer, TEST_BUFFER_SIZE);

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
//	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
//		if (ReceiveBuffer[Index] != SendBuffer[Index]) {
//			return XST_FAILURE;
//		}
//	}

}

void test() {
	XUartLite_Recv(&uart1, ReceiveBuffer, TEST_BUFFER_SIZE);
}

void SendHandler(void *CallBackRef, unsigned int EventData)
{
	TotalSentCount = EventData;
	xil_printf("Called SendHandler: %d\n\r", TotalSentCount);

}

void RecvHandler(void *CallBackRef, unsigned int EventData)
{
	TotalReceivedCount = EventData;
	xil_printf("Called RecvHandler: %d\n\r", TotalSentCount);
//	xil_printf("ReceiveBuffer: \n\r %s\n\r", ReceiveBuffer);
}
