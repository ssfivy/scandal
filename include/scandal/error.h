/* --------------------------------------------------------------------------
	Scandal Error Handling
	File name: scandal_error.h
	Author: David Snowdon
    
    Copyright (C) 2002 David Snowdon

	Date: 31/07/02
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

#ifndef __SCANDAL_ERR__
#define __SCANDAL_ERR__

#include <scandal/types.h>

/* Scandal error codes */
#define NO_ERR           0
#define LEN_ERR          1
#define BUF_FULL_ERR     2
#define NO_MSG_ERR       3
#define STD_ID_ERR	 4

u08  scandal_get_last_scandal_error();
void scandal_do_scandal_err(u08 	err);

u08  scandal_get_last_user_error();
void scandal_do_user_err(u08 	err);

void scandal_do_fatal_err(u08 	err);

u32 scandal_get_num_errors(void);
#endif
