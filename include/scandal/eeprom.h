/* --------------------------------------------------------------------------
	Scandal EEPROM Interface
	File name: scandal_eeprom.h
	Author: David Snowdon
    Copyright (C) David Snowdon 2002. 
 
	Date: 14/08/02
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

#ifndef __SCANDAL_EEPROM__
#define __SCANDAL_EEPROM__

#include <scandal/types.h>
#include <scandal/engine.h>

#include <project/scandal_config.h>

#define SIZEOF_SCANDAL_CONFIG  ((NUM_IN_CHANNELS * SIZEOF_IN_CHAN_CONFIG) + \
				(NUM_OUT_CHANNELS * SIZEOF_OUT_CHAN_CONFIG) + \
				1 + 1)

typedef struct conf {
	u08					version;				/* Scandal version */
	u08					addr;					/* Node address */
	in_channel_config 	ins[NUM_IN_CHANNELS];	/* In channel configuration */
	out_channel_config 	outs[NUM_OUT_CHANNELS]; /* Out channel configuration */
} scandal_config;

/* Local Prototypes */
void sc_init_eeprom(void);
void sc_read_conf(scandal_config *conf);
void sc_write_conf(scandal_config *conf);
void sc_user_eeprom_read_block(u32 loc, u08 *data, u08 length);
void sc_user_eeprom_write_block(u32 loc, u08 *data, u08 length);

#endif
