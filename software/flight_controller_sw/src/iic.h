#ifndef IIC_H   /* prevent circular inclusions */
#define IIC_H

#include "xiic.h"


typedef struct IIC {
	XIic_Config *iic_config;
	XIic Iic;
	u8 chipAddr;
	u8 currentRegister;
	u8 recvbytes;
	u8 *recv;
} IIC;

void iic_begin(IIC *InstancePtr, u32 IIC_Address, u8 Chip_Address);
void iic_init(XIic *Iic);
void write_iic(IIC *InstancePtr, u8 reg, u8 *Data, int nData);
void read_iic(IIC *InstancePtr, u8 reg, u8 *Data, int nData);
#endif  /* end of protection macro */
