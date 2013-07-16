/****************************************************************************
 *   $Id:: wdt.c 4823 2010-09-07 18:39:49Z nxp21346                         $
 *   Project: NXP LPC11xx WDT(Watchdog timer) example
 *
 *   Description:
 *     This file contains WDT code example which include WDT 
 *     initialization, WDT interrupt handler, and APIs for WDT
 *     reading.
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
#include <arch/wdt.h>
#include <scandal/wdt.h>

volatile uint32_t wdt_counter;

#if CONFIG_WDT_DEFAULT_WDT_IRQHANDLER==1
/*****************************************************************************
** Function name:		WDTHandler
**
** Descriptions:		Watchdog timer interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WDT_IRQHandler(void)
{
  uint32_t i;

#if PROTECT_MODE
  /* For WDPROTECT test */
  /* Make sure counter is below WDWARNINT */
  wdt_counter = LPC_WDT->TV;
  while (wdt_counter >= 0x1FF)
  {
    wdt_counter = LPC_WDT->TV;
  }
  /* For WDPROTECT test */
  LPC_WDT->FEED = 0xAA;		/* Feeding sequence */
  LPC_WDT->FEED = 0x55;    
  /* Make sure feed sequence executed properly */
  for (i = 0; i < 0x80000; i++);

  /* Make sure counter is reloaded with WDTC */
  wdt_counter = LPC_WDT->TV;
  while (wdt_counter < 0x200)
  {
    wdt_counter = LPC_WDT->TV;
  }
#endif

  LPC_WDT->MOD &= ~WDTOF;		/* clear the time-out flag and interrupt flag */
  LPC_WDT->MOD &= ~WDINT;		/* clear the time-out flag and interrupt flag */
}
#endif

/*****************************************************************************
** Function name:		WDTInit
**
** Descriptions:		Initialize watchdog timer, install the
**						watchdog timer interrupt handler
**
** parameters:			wdt_timer_value: WDT reset period in milliseconds (rounded to 4ms in implementation)
** Returned value:		None
** 
*****************************************************************************/
void WDT_Init(uint32_t wdt_timer_value)
{
  uint32_t i;

  // Settings for LPC1114
  /* Enable clock to WDT */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);

  LPC_SYSCON->WDTOSCCTRL = (0x1 << 5) | 24; /* 10kHz WDT Osc */
  LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);

  
  /* Connecting the WDT Oscillator to the WDT peripheral without any division in the way */
  LPC_SYSCON->WDTCLKSEL = 0x02;		/* Select watchdog osc */
  
  /* Update clock source: Write zero and then a one to update */
  //LPC_SYSCON->WDTCLKUEN = 0x01;		
  LPC_SYSCON->WDTCLKUEN = 0x00;		/* Toggle update register once */
  LPC_SYSCON->WDTCLKUEN = 0x01;
  while ( !(LPC_SYSCON->WDTCLKUEN & 0x01) );		/* Wait until updated */
  
  /* 0 for Disabled clock, 1 to 255 for division ratio - Note this is again divided by 4 afterwards in the peripheral */
  LPC_SYSCON->WDTCLKDIV = 10; //Divide clock by 10 for 1kHz at WDT
  

  //NVIC_EnableIRQ(WDT_IRQn);

  //We divide wdt_timer_value by 4 to offset for the division by 4 of the main input clock
  LPC_WDT->TC = wdt_timer_value >> 2;	/* once WDEN is set, the WDT will start after feeding */


  LPC_WDT->MOD = WDEN | WDRESET; //Enable the WDT counter and also the ability for it to reset the MCU


  LPC_WDT->FEED = 0xAA;		/* Feeding sequence */
  LPC_WDT->FEED = 0x55;    
  /* Make sure feed sequence executed properly */
for (i = 0; i < 0x80000; i++);

    /* Enabling Brown out detect and reset */
    LPC_SYSCON->BODCTRL = 0x13; //Setting BOD Level (BODRSTLEV) to 3: Reset at 2.63, deassert at 2.71V and enabling the BOD circuitry
  return;
}

/*****************************************************************************
** Function name:		WDTFeed
**
** Descriptions:		Feed watchdog timer to prevent it from timeout
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WDT_Feed( void )
{
  LPC_WDT->FEED = 0xAA;		/* Feeding sequence */
  LPC_WDT->FEED = 0x55;
  return;
}
