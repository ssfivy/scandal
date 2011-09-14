/*
 *  scandal_utils.h
 *
 *  Created by David Snowdon, MB, BE on Sat 7th June, 2003.
 *  Copyright (c) 2003 David Snowdon, Michael Blennerhassett, Bonne Eggleston 2002. All rights reserved.
 *
 *  Utility functions.
 *
 */

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

#ifndef   __SCANDAL_UTILS__
#define   __SCANDAL_UTILS__

#include <scandal/timer.h>
#include <scandal/types.h>
#include <scandal/timer.h>

// These are the scale up factors for the scaled channels. 
// They are used to calculate scaling factors from calibration data.
#define M64_SCALING_BITS 20
#define B64_SCALING_BITS 10
#define M32_SCALING_BITS 10
#define B32_SCALING_BITS 10


u08	scandal_integrate_trapz32(s32 *integral, s32 *timediff, s32 *old_val, s32 *pres_val, s32 *scaling);
u08 	scandal_get_scaled_value(u16 chan_num, s32 *value);
u08     scandal_get_unscaled_value(u16 chan_num, s32 *value);
u08 	scandal_send_scaled_channel(u08 pri, u16 chan_num, s32 value);
u08 	scandal_get_scaleaverage(u16 chan_num, s32 *sum, s32 *n);
u08	scandal_send_scaleaverage_channel(u08 pri, u16 chan_num, s32 *sum, s32 *n);
u08     scandal_div32(s32 *numerator, s32 *denominator);
//u08 	scandal_bitdiv32(s32 *value, u08 bits);
void	scandal_ms_delay(sc_time_t delay);
void    scandal_delay(int num);

#ifdef AVAILABLE_64
u08	scandal_integrate_trapz64(s64 *integral, s64 *timediff, s64 *old_val, s64 *pres_val, s64 *scaling);
//u08 	scandal_bitdiv64(s64 *value, u08 bits);
u08    	scandal_div64(s64 *numerator, s64 *denominator);
u08 	scandal_get_scaleaverage64(u16 chan_num, s64 *sum, s64 *n);
u08 scandal_get_scaled64_value(u16 chan_num, s64 *value);
#endif

#endif /* __SCANDAL_UTILS__ */


