/****************************************************************************
 *   $Id:: timer32.h 4785 2010-09-03 22:39:27Z nxp21346                     $
 *   Project: NXP LPC11xx software example
 *
 *   Description:
 *     This file contains definition and prototype for 32-bit timer 
 *     configuration.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#ifndef __TIMER32_H 
#define __TIMER32_H
#if CONFIG_ENABLE_DRIVER_TIMER32==1

/* The test is either MAT_OUT or CAP_IN. Default is MAT_OUT. */
#define TIMER_MATCH		0

#define EMC0	4
#define EMC1	6
#define EMC2	8
#define EMC3	10

#define MATCH0	(1<<0)
#define MATCH1	(1<<1)
#define MATCH2	(1<<2)
#define MATCH3	(1<<3)

#define TIME_INTERVAL	(SystemCoreClock/100 - 1)
/* depending on the SystemFrequency and SystemAHBFrequency setting, 
if SystemFrequency = 60Mhz, SystemAHBFrequency = 1/4 SystemAHBFrequency, 
10mSec = 150.000-1 counts */


typedef struct _T32_CONFIG{
  uint8_t 	T32_INTERRUPT;  // CR0_INT | MR3_INT | MR2_INT | MR1_INT | MR0_INT (Set and 0x1F)
  uint8_t 	T32_ENABLE;  // Starts the timer
  uint8_t 	T32_RESET;  // Restarts timer to zero
  uint32_t 	T32_PRESCALER; // Prescaler to divide the main clock value by + 1
  uint16_t 	T32_MATCH_CONTROL; // Match control register to decide what match registers are enabled and what their function is
  uint32_t 	T32_MATCH0; // Match register number 0, you can make something happen when the timer reaches this value using the Match Control Register
  uint32_t 	T32_MATCH1; // Match register number 1, you can make something happen when the timer reaches this value using the Match Control Register
  uint32_t 	T32_MATCH2; // Match register number 2, you can make something happen when the timer reaches this value using the Match Control Register
  uint32_t 	T32_MATCH3; // Match register number 3, you can make something happen when the timer reaches this value using the Match Control Register
  uint32_t 	T32_CAPTURE_CONTROL; // Can be used to configure capturing timer value to CR0 on an external event
  uint8_t 	T32_EXTERNAL_MATCH_CONTROL;  // Used to enable output of square waves
  uint32_t 	T32_EXTERNAL_MATCH_0;  //  Function of external match on pin CT32n_MAT0 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  uint32_t 	T32_EXTERNAL_MATCH_1;  //  Function of external match on pin CT32n_MAT1 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  uint32_t 	T32_EXTERNAL_MATCH_2;  //  Function of external match on pin CT32n_MAT2 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  uint32_t 	T32_EXTERNAL_MATCH_3;  //  Function of external match on pin CT32n_MAT3 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  uint8_t 	T32_COUNTER_CONTROL; // _CTCT Set to zero unless you want to use this to count an external signal
  uint8_t 	T32_PWM_CONTROL; // _PWMC  Set the appropriate bit high to enable PWM on the corresponding external match pin
} T32_CONFIG;


typedef struct _T32_IOCONFIG{
  uint8_t 	T32_CAP0; //
  uint8_t 	T32_MAT0; //
  uint8_t 	T32_MAT1; //
  uint8_t 	T32_MAT2; //
  uint8_t 	T32_MAT3; //
} T32_IOCONFIG;

void delay32Ms(uint8_t timer_num, uint32_t delayInMs);

#if CONFIG_TIMER32_DEFAULT_TIMER32_0_IRQHANDLER==1
extern volatile uint32_t timer32_0_counter;
#endif

#if CONFIG_TIMER32_DEFAULT_TIMER32_1_IRQHANDLER==1
extern volatile uint32_t timer32_1_counter;
#endif

void enable_timer32(uint8_t timer_num);
void disable_timer32(uint8_t timer_num);
void reset_timer32(uint8_t timer_num);
void init_timer32(uint8_t timer_num, T32_CONFIG conf_struct);
void init_timer32PWM(uint8_t timer_num, uint32_t period, uint8_t match_enable);
void setMatch_timer32PWM (uint8_t timer_num, uint8_t match_nr, uint32_t value);

#endif
#endif /* end __TIMER32_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
