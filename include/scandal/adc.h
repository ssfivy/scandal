/* -------------------------------------------------------------------------- 
	ADC Driver Header 
	 
	File name: adc.h 
	Author: David Snowdon 
	Date: 15/07/02 
    
    Copyright (C) David Snowdon, Robert Reid 2002.
 -------------------------------------------------------------------------- */  
 
#ifndef __ADC__ 
#define __ADC__ 
#include <scandal/types.h>

void init_adc(void); 
u16 sample_adc(u08 channel); 
void enable_adc(void); 
void disable_adc(void); 
#endif
