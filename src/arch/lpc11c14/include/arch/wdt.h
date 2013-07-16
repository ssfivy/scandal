/****************************************************************************
 *   $Id:: wdt.h 4785 2010-09-03 22:39:27Z nxp21346                         $
 *   Project: NXP LPC11xx WDT example
 *
 *   Description:
 *     This file contains WDT code header definition.
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
#ifndef __WDT_H 
#define __WDT_H

#define WDEN              (0x1<<0)
#define WDRESET           (0x1<<1)
#define WDTOF             (0x1<<2)
#define WDINT             (0x1<<3)
#define WDPROTECT         (0x1<<4)

#if CONFIG_WDT_DEFAULT_WDT_IRQHANDLER==1
extern void WDT_IRQHandler(void);
#endif

void WDT_Init(uint32_t wdt_timer_value);
void WDT_Feed(void);

#endif /* end __WDT_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
