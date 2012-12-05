/* scandal_manager.c
 
 Copyright (C) David Snowdon, 2009. 
 
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

#include <scandal/engine.h>
#include <scandal/types.h>

sc_channel_id scandal_mk_channel_id(u08 priority, u08 source, u16 channel_num){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |	
		((u32)CHANNEL_TYPE << TYPE_OFFSET) |
		((u32)(source & 0xFF) << CHANNEL_SOURCE_ADDR_OFFSET) |
		((u32)(channel_num & 0x03FF) << CHANNEL_NUM_OFFSET));
}    

u32	scandal_mk_config_id(u08 priority, u08 node, u08 parameter){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)CONFIG_TYPE << TYPE_OFFSET) | 
		((u32)(node & 0xFF) << CONFIG_NODE_ADDR_OFFSET) |
		((u32)(parameter & 0x03FF) << CONFIG_PARAM_OFFSET ));
}
