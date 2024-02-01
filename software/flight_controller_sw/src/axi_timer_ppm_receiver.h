#ifndef AXI_TIMER_PPM_RECEIVER_H   /* prevent circular inclusions */
#define AXI_TIMER_PPM_RECEIVER_H

#define NUM_OF_CH 9
u32 get_ch1();
u32 get_ch2();
u32 get_ch3();
u32 get_ch4();
void tmr_ppm_init();
void tmr_ppm_setup();
void gicInit();
void print_channel_time();
void tmrHandler();

#endif  /* end of protection macro */
