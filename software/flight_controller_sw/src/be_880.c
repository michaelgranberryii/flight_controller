#include "be_880.h"

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

void read_be_data() {
	u8 data1[100];
	s8 data_size = sizeof(data1);
	u8 Rx[data_size];

	int byteRcvd = 0;

	while(byteRcvd != data_size) {
		byteRcvd = byteRcvd + XUartLite_Recv(&uart1, &Rx[byteRcvd], data_size);
	}


	xil_printf("GPS data: \n\r");

	for(int i = 0 ; i < data_size; i++)
	{
		xil_printf("%c",Rx[i]);
	}
	xil_printf("\n\r\n\r");
}
