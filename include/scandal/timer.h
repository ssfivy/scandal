/*
 *  scandal_timer.h
 *
 *  Created by David Snowdon on Wed, 8 Aug, 2002.
 *  Copyright (c) 2002 David Snowdon.
 *
 *  Timer interface functions to be implemented by platform specific code.
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

#ifndef __SCANDAL_TIMER__
#define __SCANDAL_TIMER__

#include <scandal/types.h>

/* Global variables */ 
extern uint64_t timesync_offset; 

/* Time in milliseconds */
typedef u32 sc_time_t;

/* Function Prototypes */
void sc_init_timer(void);
void sc_set_timer(sc_time_t time);
sc_time_t sc_get_timer(void);

static inline uint64_t scandal_get_realtime(void){
	return (uint64_t)sc_get_timer() + timesync_offset;
}

static inline uint32_t scandal_get_realtime32(void){
	return (scandal_get_realtime()) & 0xFFFFFFFF;
}

static inline void scandal_set_realtime(uint64_t timestamp){
    uint32_t mytime = (uint32_t)sc_get_timer(); 
    timesync_offset = timestamp - mytime; 
}

#endif
