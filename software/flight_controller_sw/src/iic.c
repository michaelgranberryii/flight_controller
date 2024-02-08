#include "iic.h"

void iic_begin(IIC *InstancePtr, u32 IIC_Address, u8 Chip_Address) {
	InstancePtr->iic_config->BaseAddress = IIC_Address;
   InstancePtr->chipAddr = Chip_Address;
   iic_init(&InstancePtr->Iic);
   XIic_SetAddress(&InstancePtr->Iic, XII_ADDR_TO_SEND_TYPE, InstancePtr->chipAddr);
}

void iic_init(XIic *Iic) {
	int status = XIic_Initialize(Iic, Iic->BaseAddress);
	if(status == XST_SUCCESS)
		xil_printf("IIC Init Successful\n\r");
	else
		xil_printf("IIC Init Failed\n\r");
	 status = XIic_SelfTest(Iic);
	if(status == XST_SUCCESS) {
		xil_printf("IIC SELF TEST SUCCUSSFUL\n\r");
	} else {
		xil_printf("IIC SELF TEST FAILED\n\r");
	}

   /*
	* Start the IIC driver so that the device is enabled.
	*/
   XIic_Start(Iic);

   /*
    * Disable Global interrupt to use polled mode operation
    */
   XIic_IntrGlobalDisable(Iic);
}

void iic_write(IIC *InstancePtr, u8 reg, u8 *Data, int nData) {
   u8 out[10];
   out[0] = reg;
   out[1] = *Data;
   int Status;

   if (InstancePtr->currentRegister != reg) {
	   InstancePtr->currentRegister = reg;
   }
   Status = XIic_Start(&InstancePtr->Iic);
   if (Status != XST_SUCCESS) {
      return;
   }
   XIic_Send(XPAR_AXI_IIC_0_BASEADDR, InstancePtr->chipAddr, out, nData + 1, XIIC_STOP);

   Status = XIic_Stop(&InstancePtr->Iic);
   if (Status != XST_SUCCESS) {
      return;
   }
}

void iic_read(IIC *InstancePtr, u8 reg, u8 *Data, int nData) {
   int Status;
   Status = XIic_Start(&InstancePtr->Iic);
   if (Status != XST_SUCCESS) {
      return;
   }
   if (InstancePtr->currentRegister != reg) {

      XIic_Send(XPAR_AXI_IIC_0_BASEADDR, InstancePtr->chipAddr, &reg, 1, XII_REPEATED_START_OPTION);
      InstancePtr->currentRegister = reg;
   }
   XIic_Recv(XPAR_AXI_IIC_0_BASEADDR, InstancePtr->chipAddr, Data, nData, XIIC_STOP);

   Status = XIic_Stop(&InstancePtr->Iic);
   if (Status != XST_SUCCESS) {
      return;
   }
}
