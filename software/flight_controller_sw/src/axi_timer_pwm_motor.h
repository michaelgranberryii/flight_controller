#ifndef AXI_TIMER_PWM_MOTOR_H   /* prevent circular inclusions */
#define AXI_TIMER_PWM_MOTOR_H

#include "xtmrctr.h"

// Tmr clk is set to 100 MHz in Vivado
#define ONE_mSec 1e6 // 1000000 nSec = 1 mSec duty cycle
#define PWM_PERIOD_nSEC 20e6 // 20 mSec pwm period
#define IN_MIN 990 // 0000
#define IN_MAX 2010 // 1111
#define OUT_MIN 1e6 // 1_000_000 nSec = 1 mSec
#define OUT_MAX 2e6 // 2_000_000 nSec = 2 mSec
#define NUM_OF_PWM_TIMERS 4

void tmr_pwm_init(XTmrCtr *tmr_pwm_ptr, u16 DeviceID);
void tmr_pwm_setup(XTmrCtr *tmr_pwm_ptr);
int map(float value, float in_min, float in_max, float out_min, float out_max);
void update_pwm(XTmrCtr *tmr_pwm, int input_channel_value);
void set_new_pwm(XTmrCtr *tmr_pwm, int pulse_width_nSec);
void trm_pwm_init_setup_all();
void update_pwm1(int input_channel_value);
void update_pwm2(int input_channel_value);
void update_pwm3(int input_channel_value);
void update_pwm4(int input_channel_value);

#endif  /* end of protection macro */
