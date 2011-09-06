/*
 *  scandal_maths.h
 *
 *  Created by David Snowdon, MB, BE on Sat 7th June, 2003.
 *  Copyright (c) 2002 David Snowdon, Michael Blennerhassett, Bonne Eggleston. 
 *
 *  maths functions.
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


#ifdef    __SCANDAL_MATHS__
#define   __SCANDAL_MATHS__

#include <scandal/types.h>

s64      scandal_div64(s64 numerator, s64 denominator);
s64      scandal_div32(s32 numerator, s32 denominator);

s32      scandal_scale_value(s32 value, s32 m, s32 b);
s32      scandal_scaleaverage(s32 sum, s32 m, s32 b, s32 n);

#endif /* __SCANDAL_MATHS__ */
