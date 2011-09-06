/* --------------------------------------------------------------------------
	Scandal Utilities
	File name: scandal_utils.c
	Author: BE, DS, MB

    Copyright (C) Bonne Eggleston, David Snowdon, Michael Blennerhassett, 2002. 
    
	Date: 07/06/2003
   -------------------------------------------------------------------------- */

/* 
 * This file is part of Scandal.
 * 
 * Scandal is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * Scandal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with Scandal.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <scandal/types.h>
#include <scandal/engine.h>
#include <scandal/utils.h>
#include <scandal/maths.h>
#include <scandal/error.h>
#include <scandal/message.h>

#include <project/scandal_config.h>

u08 scandal_integrate_trapz32(s32 *integral, s32 *timediff, s32 *old_val, s32 *pres_val, s32  *scaling)
{
	*integral = (*old_val + *pres_val) * (*timediff);
	*scaling = *scaling << 1;
 	scandal_div32(integral, scaling);
	return NO_ERR;
}

u08 scandal_get_scaled_value(u16 chan_num, s32 *value){
	
	*value *= scandal_get_m(chan_num);
	//	scandal_bitdiv(value, M32_SCALING_BITS - B32_SCALING_BITS);
	*value >>= M32_SCALING_BITS - B32_SCALING_BITS; // This loses a degree of accuracy due to integer division
	*value += scandal_get_b(chan_num);
	//	scandal_bitdiv(value, B32_SCALING_BITS);
	*value >>= B32_SCALING_BITS;
	return NO_ERR;
	
//	return scandal_div32(scandal_get_m(chan_num)*value + scandal_get_b(chan_num), 1000);
}

u08 scandal_get_unscaled_value(u16 chan_num, s32 *value){
	*value <<= B32_SCALING_BITS; 
	*value -= scandal_get_b(chan_num);
	*value /= scandal_get_m(chan_num); 
	*value <<= M32_SCALING_BITS - B32_SCALING_BITS; 
	return NO_ERR; 
}

u08 scandal_send_scaled_channel(u08 pri, u16 chan_num, s32 value){
	scandal_get_scaled_value(chan_num, &value);
	return(scandal_send_channel(pri, chan_num, value)); 
}

u08 scandal_get_scaleaverage(u16 chan_num, s32 *sum, s32 *n){
	*sum *= scandal_get_m(chan_num);
	*n <<= M32_SCALING_BITS - B32_SCALING_BITS; // This multiplies n by the difference between the b and m factors for accuracy in the diviusion. 
	scandal_div32(sum, n);
	*sum += scandal_get_b(chan_num);
	// scandal_bitdiv(sum, B32_SCALING_BITS); 
	*sum >>= B32_SCALING_BITS;
	return NO_ERR;
}

u08 scandal_send_scaleaverage_channel(u08 pri, u16 chan_num, s32 *sum, s32 *n) {
	scandal_get_scaleaverage(chan_num, sum, n);
	return(scandal_send_channel(pri, chan_num, *sum)); 
}

void scandal_ms_delay(sc_time_t delay){
	sc_time_t time = sc_get_timer();
	while(time + delay > sc_get_timer()) {
		handle_scandal();
	}
}

void scandal_delay(int num){
	sc_time_t time = sc_get_timer();
	while(time + num > sc_get_timer())
		;
}

void scandal_naive_delay(int num) {
	volatile int i; 
	for(i=num; i>0; i--)
		; 
}

#ifdef AVAILABLE_64

// This is intended for use with even positive denominators.
// Adds accuracy to division.
// This function has some serious problems. If you get random numbers use normal division instead.
// I seem to have fixed the random number problem, by puting the maths in different lines.

u08 scandal_integrate_trapz64(s64 *integral, s64 *timediff, s64 *old_val, s64 *pres_val, s64  *scaling)
{
	*integral = (*old_val + *pres_val) * (*timediff);
	*scaling = *scaling << 1;
	scandal_div64(old_val, scaling);
  	return NO_ERR;
} 

u08 scandal_get_scaleaverage64(u16 chan_num, s64 *sum, s64 *n){
    *sum *= scandal_get_m(chan_num);
    *n <<=  M64_SCALING_BITS - B64_SCALING_BITS;
    //*sum /= *n;
    scandal_div64(sum,n);
    *sum += scandal_get_b(chan_num);
    //  scandal_bitdiv(sum, B64_SCALING_BITS);
    *sum >>= B64_SCALING_BITS;

   return NO_ERR;
}

u08 scandal_get_scaled64_value(u16 chan_num, s64 *value){
	*value *= scandal_get_m(chan_num);
	*value >>= M64_SCALING_BITS - B64_SCALING_BITS;
	*value += scandal_get_b(chan_num);
	*value >>= B64_SCALING_BITS;
	return NO_ERR;
}

#endif
