/* --------------------------------------------------------------------------
	Scandal Maths
	File name: scandal_maths.c
	Author: David Snowdon, MB, BE

    Copyright (C) David Snowdon, Michael Blennerhassett, Bonne Eggleston, 2003. 
 
	Date: 7/6/2003
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
#include <scandal/error.h>
#include <scandal/maths.h>

u08 scandal_div32(s32 *numerator, s32 *denominator) {
	if (*denominator != 0) {
		if (*numerator >0) {
			*numerator += (*denominator >> 1);
			*numerator /= (*denominator);
	 	} else if (*numerator <0) {
			*numerator += (*denominator >> 1);
			*numerator /= (*denominator);
		}
	}
	return NO_ERR;
}

u08 scandal_div64(s64 *numerator, s64 *denominator){
	if (*denominator != 0){
		if (*numerator > 0){
			*numerator += (*denominator >> 1);
			*numerator /= (*denominator);
		} else if (*numerator <0){
			*numerator -= (*denominator >> 1);
			*numerator /= (*denominator);
		}
	}
	return NO_ERR;
}
