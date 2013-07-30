/****************************************************************************
 *   $Id:: adc.c 4785 2010-09-03 22:39:27Z nxp21346                         $
 *   Project: NXP LPC11xx ADC example
 *
 *   Description:
 *     This file contains ADC code example which include ADC 
 *     initialization, ADC interrupt handler, and APIs for ADC
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
#include <arch/adc.h>
#include <arch/gpio.h>

volatile uint32_t ADCValue[ADC_NUM];

#if CONFIG_ADC_DEFAULT_ADC_IRQHANDLER==1
volatile uint32_t ADCIntDone = 0;
#endif
volatile uint32_t OverRunCounter = 0;

#if BURST_MODE
volatile uint32_t channel_flag = 0; 
#endif

#if CONFIG_ADC_DEFAULT_ADC_IRQHANDLER==1
/******************************************************************************
** Function name:		ADC_IRQHandler
**
** Descriptions:		ADC interrupt handler
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void ADC_IRQHandler (void) 
{
  uint32_t regVal, i;

  regVal = LPC_ADC->STAT;		/* Read ADC will clear the interrupt */
  if ( regVal & 0x0000FF00 )	/* check OVERRUN error first */
  {
	OverRunCounter++;
	for ( i = 0; i < ADC_NUM; i++ )
	{
	  regVal = (regVal & 0x0000FF00) >> 0x08;
	  /* if overrun, just read ADDR to clear */
	  /* regVal variable has been reused. */
	  if ( regVal & (0x1 << i) )
	  {
		regVal = LPC_ADC->DR[i];
	  }
	}
	LPC_ADC->CR &= 0xF8FFFFFF;	/* stop ADC now */
	ADCIntDone = 1;
	return;
  }
    
  if ( regVal & ADC_ADINT )
  {
	for ( i = 0; i < ADC_NUM; i++ )
	{
	  if ( (regVal&0xFF) & (0x1 << i) )
	  {
		ADCValue[i] = ( LPC_ADC->DR[i] >> 6 ) & 0x3FF;
	  }
	}
#if CONFIG_ADC_ENABLE_BURST_MODE==1
	channel_flag |= (regVal & 0xFF);
	if ( (channel_flag & 0xFF) == 0xFF )
	{
	  /* All the bits in have been set, it indicates all the ADC 
	  channels have been converted. */
	  LPC_ADC->CR &= 0xF8FFFFFF;	/* stop ADC now */
	  channel_flag = 0; 
	  ADCIntDone = 1;
	}
#else
	LPC_ADC->CR &= 0xF8FFFFFF;	/* stop ADC now */ 
	ADCIntDone = 1;
#endif
  }
  return;
}
#endif

/*****************************************************************************
** Function name:		ADCInit
**
** Descriptions:		initialize ADC channel
**
** parameters:			ADC clock rate
** Returned value:		None
** 
*****************************************************************************/
void ADC_Init( uint32_t ADC_Clk ) {
	uint32_t i;

	/* Disable Power down bit to the ADC block. */  
	LPC_SYSCON->PDRUNCFG &= ~(0x1<<4);

	/* Enable AHB clock to the ADC. */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<13);

	for ( i = 0; i < ADC_NUM; i++ ) {
		ADCValue[i] = 0x0;
	}

	/* set the clock divider to just under 4.5MHz */
	LPC_ADC->CR = LPC_ADC->CR = (SystemCoreClock/((LPC_SYSCON->SYSAHBCLKDIV)*ADC_MAXCLK))<<8;
    
	/* enable the interrupt handler */
	NVIC_EnableIRQ(ADC_IRQn);
}

/* Enable an individual ADC channel */
void ADC_EnableChannel(uint32_t channel_num) {

	switch (channel_num) {
	 case 0:
		GPIO_SetFunction(PORT0, 11, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	 case 1:
		GPIO_SetFunction(PORT1, 0, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	 case 2:
		GPIO_SetFunction(PORT1, 1, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	 case 3:
		GPIO_SetFunction(PORT1, 2, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	 case 4:
		/* this pin is TMS */
		return;

	 case 5:
		GPIO_SetFunction(PORT1, 4, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	 case 6:
		GPIO_SetFunction(PORT1, 10, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	 case 7:
		GPIO_SetFunction(PORT1, 11, GPIO_FUNC1, GPIO_MODE_NONE);
		break;

	}

	LPC_ADC->CR |= 1 << channel_num;
	LPC_ADC->INTEN |= 1 << channel_num;
}

/*****************************************************************************
** Function name:		ADCRead
**
** Descriptions:		Read ADC channel
**
** parameters:			Channel number
** Returned value:		Value read, if interrupt driven, return channel #
** 
*****************************************************************************/
uint32_t ADC_Read( uint8_t channelNum )
{
#if CONFIG_ADC_ENABLE_ADC_IRQHANDLER!=1
  uint32_t regVal, ADC_Data;
#endif

  /* channel number is 0 through 7 */
  if ( channelNum >= ADC_NUM )
  {
	channelNum = 0;		/* reset channel number to 0 */
  }
  LPC_ADC->CR &= 0xFFFFFF00;
  LPC_ADC->CR |= (1 << 24) | (1 << channelNum);	
				/* switch channel,start A/D convert */
#if CONFIG_ADC_ENABLE_ADC_IRQHANDLER!=1
  while ( 1 )			/* wait until end of A/D convert */
  {
	regVal = *(volatile unsigned long *)(LPC_ADC_BASE 
			+ ADC_OFFSET + ADC_INDEX * channelNum);
	/* read result of A/D conversion */
	if ( regVal & ADC_DONE )
	{
	  break;
	}
  }	
        
  LPC_ADC->CR &= 0xF8FFFFFF;	/* stop ADC now */    
  if ( regVal & ADC_OVERRUN )	/* save data when it's not overrun, otherwise, return zero */
  {
	return ( 0 );
  }
  ADC_Data = ( regVal >> 6 ) & 0x3FF;
  return ( ADC_Data );	/* return A/D conversion value */
#else
  return ( channelNum );	/* if it's interrupt driven, the ADC reading is 
							done inside the handler. so, return channel number */
#endif
}

/*****************************************************************************
** Function name:		ADC0BurstRead
**
** Descriptions:		Use burst mode to convert multiple channels once.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void ADC_BurstRead( void )
{
  if ( LPC_ADC->CR & (0x7<<24) )
  {
	LPC_ADC->CR &= ~(0x7<<24);
  }
  /* Read all channels, 0 through 7. Be careful that if the ADCx pins is shared
  with SWD CLK or SWD IO. */
  LPC_ADC->CR |= (0xFF);
  LPC_ADC->CR |= (0x1<<16);		/* Set burst mode and start A/D convert */
  return;						/* the ADC reading is done inside the 
								handler, return 0. */
}
