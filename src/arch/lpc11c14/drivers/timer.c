/****************************************************************************
 *   $Id:: timer32.c 4785 2010-09-03 22:39:27Z nxp21346                     $
 *   Project: NXP LPC11xx 32-bit timer example
 *
 *   Description:
 *     This file contains 32-bit timer code example which include timer 
 *     initialization, timer interrupt handler, and related APIs for 
 *     timer setup.
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
#include <project/driver_config.h>

#include <scandal/timer.h>

#include <arch/timer.h>

/* ===================
 * CodeRed - Modified file to extract out interrupt handler related code,
 * which is really application project specific.
 * Set TIMER32_GENERIC_INTS to 1 to reenable original code.
 * =================== */



/******************************************************************************
** Function name:		TIMER32_0_IRQHandler
**
** Descriptions:		Timer/Counter 0 interrupt handler
**						executes each 10ms @ 60 MHz CPU Clock
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void TIMER32_0_IRQHandler(void)
{  
  LPC_TMR32B0->IR=0x1F; //Clear all timer 0 interrupts
  return;
}

/******************************************************************************
** Function name:		TIMER32_1_IRQHandler
**
** Descriptions:		Timer/Counter 1 interrupt handler
**						executes each 10ms @ 60 MHz CPU Clock
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void TIMER32_1_IRQHandler(void)
{  
  LPC_TMR32B0->IR=0x1F; //Clear all timer 1 interrupts
  return;
}

/******************************************************************************
** Function name:		enable_timer
**
** Descriptions:		Enable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void enable_timer32(uint8_t timer_num)
{
  if ( timer_num == 0 )
  {
    LPC_TMR32B0->TCR = 1;
  }
  else
  {
    LPC_TMR32B1->TCR = 1;
  }
  return;
}

/******************************************************************************
** Function name:		disable_timer
**
** Descriptions:		Disable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void disable_timer32(uint8_t timer_num)
{
  if ( timer_num == 0 )
  {
    LPC_TMR32B0->TCR = 0;
  }
  else
  {
    LPC_TMR32B1->TCR = 0;
  }
  return;
}

/******************************************************************************
** Function name:		reset_timer
**
** Descriptions:		Reset timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void reset_timer32(uint8_t timer_num)
{
  uint32_t regVal;

  if ( timer_num == 0 )
  {
    regVal = LPC_TMR32B0->TCR;
    regVal |= 0x02;
    LPC_TMR32B0->TCR = regVal;
  }
  else
  {
    regVal = LPC_TMR32B1->TCR;
    regVal |= 0x02;
    LPC_TMR32B1->TCR = regVal;
  }
  return;
}


/* TODO:Define function to retrieve TC for either timer 0 or 1 */


/* TODO:Define function to retrieve Capture Register 0 for either timer 0 or 1 */




/*
Name: timer32_ioconfig

Description: Sets the desired pins to the timer related functions

Parameters: 
  - time number (either 1 or 2 to specify which timer block is being used)
  - t32_ioconfig struct which specifies which pins are to be set with a nonzero value
  
  typedef struct _T32_IOCONFIG{
    uint8_t 	T32_CAP0; //
    uint8_t 	T32_MAT0; //
    uint8_t 	T32_MAT1; //
    uint8_t 	T32_MAT2; //
    uint8_t 	T32_MAT3; //
} T32_IOCONFIG;

Return: No return

*/

void timer32_ioconfig(uint8_t timer_number, T32_IOCONFIG ioconfig_struct)
{
  if(timer_number==0){/*  Timer0_32 I/O config */
    if(ioconfig_struct.T32_CAP0){
      LPC_IOCON->PIO1_5 &= ~0x07;	
      LPC_IOCON->PIO1_5 |= 0x02;	/* Timer0_32 CAP0 */
    }
    if(ioconfig_struct.T32_MAT0){
      LPC_IOCON->PIO1_6 &= ~0x07;
      LPC_IOCON->PIO1_6 |= 0x02;	/* Timer0_32 MAT0 */
    }
    
    if(ioconfig_struct.T32_MAT1){
      LPC_IOCON->PIO1_7 &= ~0x07;
      LPC_IOCON->PIO1_7 |= 0x02;	/* Timer0_32 MAT1 */
    }
    
    if(ioconfig_struct.T32_MAT2){
      LPC_IOCON->PIO0_1 &= ~0x07;	
      LPC_IOCON->PIO0_1 |= 0x02;	/* Timer0_32 MAT2 */
    }
    
    if(ioconfig_struct.T32_MAT3){
      LPC_IOCON->R_PIO0_11 &= ~0x07;	
      LPC_IOCON->R_PIO0_11 |= 0x03;	/* Timer0_32 MAT3 */
    }
  
    
  }else if(timer_number==1){/*  Timer1_32 I/O config */
    if(ioconfig_struct.T32_CAP0){
      LPC_IOCON->R_PIO1_0  &= ~0x07;	
      LPC_IOCON->R_PIO1_0  |= 0x03;	/* Timer1_32 CAP0 */
    }
    
    if(ioconfig_struct.T32_MAT0){
      LPC_IOCON->R_PIO1_1  &= ~0x07;	
      LPC_IOCON->R_PIO1_1  |= 0x03;	/* Timer1_32 MAT0 */
    }
    
    if(ioconfig_struct.T32_MAT1){
      LPC_IOCON->R_PIO1_2 &= ~0x07;
      LPC_IOCON->R_PIO1_2 |= 0x03;	/* Timer1_32 MAT1 */
    }
    
    if(ioconfig_struct.T32_MAT2){
      LPC_IOCON->SWDIO_PIO1_3  &= ~0x07;
      LPC_IOCON->SWDIO_PIO1_3  |= 0x03;	/* Timer1_32 MAT2 */
    }
    
    if(ioconfig_struct.T32_MAT3){
      LPC_IOCON->PIO1_4 &= ~0x07;
      LPC_IOCON->PIO1_4 |= 0x02;		/* Timer0_32 MAT3 */
    }
  }
  
}

/******************************************************************************
** Function name:		init_timer
**
** Descriptions:		Initialize timer, set timer interval, reset timer,
**						install timer interrupt handler
**
** parameters:			timer number and timer interval

  __IO uint32_t IR;                     /*!< Offset: 0x000 Interrupt Register (R/W) 
  __IO uint32_t TCR;                    /*!< Offset: 0x004 Timer Control Register (R/W) 
  __IO uint32_t TC;                     /*!< Offset: 0x008 Timer Counter Register (R/W) *
  __IO uint32_t PR;                     /*!< Offset: 0x00C Prescale Register (R/W) *
  __IO uint32_t PC;                     /*!< Offset: 0x010 Prescale Counter Register (R/W) *
  __IO uint32_t MCR;                    /*!< Offset: 0x014 Match Control Register (R/W) *
  __IO uint32_t MR0;                    /*!< Offset: 0x018 Match Register 0 (R/W) *
  __IO uint32_t MR1;                    /*!< Offset: 0x01C Match Register 1 (R/W) *
  __IO uint32_t MR2;                    /*!< Offset: 0x020 Match Register 2 (R/W) *
  __IO uint32_t MR3;                    /*!< Offset: 0x024 Match Register 3 (R/W) *
  __IO uint32_t CCR;                    /*!< Offset: 0x028 Capture Control Register (R/W) *
  __I  uint32_t CR0;                    /*!< Offset: 0x02C Capture Register 0 (R/ ) *
       uint32_t RESERVED1[3];
  __IO uint32_t EMR;                    /*!< Offset: 0x03C External Match Register (R/W) *
       uint32_t RESERVED2[12];
  __IO uint32_t CTCR;                   /*!< Offset: 0x070 Count Control Register (R/W) *
  __IO uint32_t PWMC;                   /*!< Offset: 0x074 PWM Control Register (R/W) *


  typedef struct _T32_CONFIG{
    uint8_t 	T32_INTERRUPT; _IR // CR0_INT | MR3_INT | MR2_INT | MR1_INT | MR0_INT (Set and 0x1F)
    uint8_t 	T32_ENABLE; _TCR // Starts the timer
    uint8_t 	T32_RESET; _TCR // Restarts timer to zero (Assert and de-assert)
    uint32_t 	T32_PRESCALER; // Prescaler to divide the main clock value by + 1
    uint16_t 	T32_MATCH_CONTROL; // Match control register to decide what match registers are enabled and what their function is
    uint32_t 	T32_MATCH0; // Match register number 0, you can make something happen when the timer reaches this value using the Match Control Register
    uint32_t 	T32_MATCH1; // Match register number 1, you can make something happen when the timer reaches this value using the Match Control Register
    uint32_t 	T32_MATCH2; // Match register number 2, you can make something happen when the timer reaches this value using the Match Control Register
    uint32_t 	T32_MATCH3; // Match register number 3, you can make something happen when the timer reaches this value using the Match Control Register
    uint32_t 	T32_CAPTURE_CONTROL; // Can be used to configure capturing timer value to CR0 on an external event
    uint8_t 	T32_EXTERNAL_MATCH_CONTROL; // Used to enable output of square waves
    uint32_t 	T32_EXTERNAL_MATCH_0; // Function of external match on pin CT32n_MAT0 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
    uint32_t 	T32_EXTERNAL_MATCH_1; // Function of external match on pin CT32n_MAT1 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
    uint32_t 	T32_EXTERNAL_MATCH_2; // Function of external match on pin CT32n_MAT2 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
    uint32_t 	T32_EXTERNAL_MATCH_3; // Function of external match on pin CT32n_MAT3 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
    uint8_t 	T32_COUNTER_CONTROL; // Set to zero unless you want to use this to count an external signal
    uint8_t 	T32_PWM_CONTROL; // Set the appropriate bit high to enable PWM on the corresponding external match pin
    
} T32_CONFIG;
/*

** Returned value:		None
** 
******************************************************************************/
void init_timer32(uint8_t timer_num, T32_CONFIG conf_struct) 
{
  if ( timer_num == 0 )
  {
    /* Some of the I/O pins need to be carefully planned if
    you use below module because JTAG and TIMER CAP/MAT pins are muxed. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);
    
    LPC_TMR32B0->IR = conf_struct.T32_INTERRUPT & 0x1F;
    //LPC_TMR32B0->TCR = c//Do not edit yet, we do this at the end when we want to start the timer
    //LPC_TMR32B0->TC //This is a read only and stores the timer counter, we don't want to write to this
    LPC_TMR32B0->PR = conf_struct.T32_PRESCALER;
    //LPC_TMR32B0->PC = //Do not edit - Read only and it's not initialisation related
    LPC_TMR32B0->MCR = conf_struct.T32_MATCH_CONTROL & 0xFFF;
    LPC_TMR32B0->MR0 = conf_struct.T32_MATCH0;
    LPC_TMR32B0->MR1 = conf_struct.T32_MATCH1;
    LPC_TMR32B0->MR2 = conf_struct.T32_MATCH2;
    LPC_TMR32B0->MR3 = conf_struct.T32_MATCH3;
    LPC_TMR32B0->CCR = conf_struct.T32_CAPTURE_CONTROL & 0x1; //Note, interrupts cannot be set, change this mask to allow interrupts on capture
    //LPC_TMR32B0->CR0 = //Do not edit - Read only and it's not initialisation related
    LPC_TMR32B0->EMR = ((conf_struct.T32_EXTERNAL_MATCH_3 & 0x3) << 10 ) | ((conf_struct.T32_EXTERNAL_MATCH_2 & 0x3) << 8 ) | ((conf_struct.T32_EXTERNAL_MATCH_1 & 0x3) << 6 ) | ((conf_struct.T32_EXTERNAL_MATCH_0 & 0x3) << 4 ) | (conf_struct.T32_EXTERNAL_MATCH_CONTROL & 0xF) ;
    LPC_TMR32B0->CTCR = conf_struct.T32_COUNTER_CONTROL & 0x3;
    LPC_TMR32B0->PWMC = conf_struct.T32_PWM_CONTROL & 0xF;
    
    
    LPC_TMR32B0->TCR = ((conf_struct.T32_RESET & 0x1) << 1) | (conf_struct.T32_ENABLE & 0x1);
    LPC_TMR32B0->TCR = (conf_struct.T32_ENABLE & 0x1);

    /* Enable the TIMER0 Interrupt - enable this once a handler has been written! */
    //NVIC_EnableIRQ(TIMER_32_0_IRQn);

  }
  else if ( timer_num == 1 )
  {
    /* Some of the I/O pins need to be clearfully planned if
    you use below module because JTAG and TIMER CAP/MAT pins are muxed. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);
#ifdef __JTAG_DISABLED
    LPC_IOCON->JTAG_TMS_PIO1_0  &= ~0x07;	/*  Timer1_32 I/O config */
    LPC_IOCON->JTAG_TMS_PIO1_0  |= 0x03;	/* Timer1_32 CAP0 */
    LPC_IOCON->JTAG_TDO_PIO1_1  &= ~0x07;	
    LPC_IOCON->JTAG_TDO_PIO1_1  |= 0x03;	/* Timer1_32 MAT0 */
    LPC_IOCON->JTAG_nTRST_PIO1_2 &= ~0x07;
    LPC_IOCON->JTAG_nTRST_PIO1_2 |= 0x03;	/* Timer1_32 MAT1 */
    LPC_IOCON->ARM_SWDIO_PIO1_3  &= ~0x07;
    LPC_IOCON->ARM_SWDIO_PIO1_3  |= 0x03;	/* Timer1_32 MAT2 */
#endif
    LPC_IOCON->PIO1_4 &= ~0x07;
    LPC_IOCON->PIO1_4 |= 0x02;		/* Timer0_32 MAT3 */

#if CONFIG_TIMER32_DEFAULT_TIMER32_1_IRQHANDLER==1
    timer32_1_counter = 0;
    timer32_1_capture = 0;
#endif //TIMER32_1_DEFAULT_HANDLER

    LPC_TMR32B1->MR0 = 0;
#if TIMER_MATCH
	LPC_TMR32B1->EMR &= ~(0xFF<<4);
	LPC_TMR32B1->EMR |= ((0x3<<4)|(0x3<<6)|(0x3<<8)|(0x3<<10));	/* MR0/1/2 Toggle */
#else
	/* Capture 0 on rising edge, interrupt enable. */
	LPC_TMR32B1->CCR = (0x1<<0)|(0x1<<2);
#endif
    LPC_TMR32B1->MCR = 3;			/* Interrupt and Reset on MR0 */

#if CONFIG_TIMER32_DEFAULT_TIMER32_1_IRQHANDLER==1
    /* Enable the TIMER1 Interrupt */
    NVIC_EnableIRQ(TIMER_32_1_IRQn);
#endif
  }
  return;
}




/******************************************************************************
** Function name:		pwm32_setMatch
**
** Descriptions:		Set the pwm32 match values
**
** parameters:			timer number, match numner and the value
**
** Returned value:		None
** 
******************************************************************************/
void setMatch_timer32PWM (uint8_t timer_num, uint8_t match_nr, uint32_t value)
{
	if (timer_num)
	{
		switch (match_nr)
		{
			case 0:
				LPC_TMR32B1->MR0 = value;
				break;
			case 1: 
				LPC_TMR32B1->MR1 = value;
				break;
			case 2:
				LPC_TMR32B1->MR2 = value;
				break;
			case 3: 
				LPC_TMR32B1->MR3 = value;
				break;
			default:
				break;
		}	

	}
	else 
	{
		switch (match_nr)
		{
			case 0:
				LPC_TMR32B0->MR0 = value;
				break;
			case 1: 
				LPC_TMR32B0->MR1 = value;
				break;
			case 2:
				LPC_TMR32B0->MR2 = value;
				break;
			case 3: 
				LPC_TMR32B0->MR3 = value;
				break;
			default:
				break;
		}	
	}

}

/* Scandal wrappers
 * *****************/

 //Scandal makes use of timer 0 of the LPC11C14 for timekeeping purposes
 
void sc_init_timer(void) {
  T32_CONFIG config_struct;
   
  config_struct.T32_INTERRUPT               = 0; // CR0_INT | MR3_INT | MR2_INT | MR1_INT | MR0_INT (Set and 0x1F)
  config_struct.T32_ENABLE                  = 1; // Starts the timer if set
  config_struct.T32_RESET                   = 1; // Restarts timer to zero if set
  config_struct.T32_PRESCALER               = (SystemCoreClock/1000-1); // Prescaler to divide the main clock value by + 1
  config_struct.T32_MATCH_CONTROL           = 0; // Match control register to decide what match registers are enabled and what their function is
  config_struct.T32_MATCH0                  = 0; // Match register number 0, you can make something happen when the timer reaches this value using the Match Control Register
  config_struct.T32_MATCH1                  = 0; // Match register number 1, you can make something happen when the timer reaches this value using the Match Control Register
  config_struct.T32_MATCH2                  = 0; // Match register number 2, you can make something happen when the timer reaches this value using the Match Control Register
  config_struct.T32_MATCH3                  = 0; // Match register number 3, you can make something happen when the timer reaches this value using the Match Control Register
  config_struct.T32_CAPTURE_CONTROL         = 0; // Can be used to configure capturing timer value to CR0 on an external event
  config_struct.T32_EXTERNAL_MATCH_CONTROL  = 0; // Used to enable output of square waves
  config_struct.T32_EXTERNAL_MATCH_0        = 0; // Function of external match on pin CT32n_MAT0 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  config_struct.T32_EXTERNAL_MATCH_1        = 0; // Function of external match on pin CT32n_MAT1 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  config_struct.T32_EXTERNAL_MATCH_2        = 0; // Function of external match on pin CT32n_MAT2 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  config_struct.T32_EXTERNAL_MATCH_3        = 0; // Function of external match on pin CT32n_MAT3 if enabled. 0:Do nothing, 1:Low on match, 2:High on match, 3:Toggle on match
  config_struct.T32_COUNTER_CONTROL         = 0; // Set to zero unless you want to use this to count an external signal
  config_struct.T32_PWM_CONTROL             = 0; // Set the appropriate bit high to enable PWM on the corresponding external match pin
  
	init_timer32(0,  config_struct);
	//enable_timer32(0); // Enabling the timer
}

void sc_set_timer(sc_time_t time) {
	//timer32_0_counter = (uint32_t)time;
  LPC_TMR32B0->TC = (uint32_t)time;
}

sc_time_t sc_get_timer(void) {
	return (sc_time_t)LPC_TMR32B0->TC; //TODO:Change this to use a proper interface function and not just access the memory directly
}

/* *******************
 * End Scandal wrappers
 */
