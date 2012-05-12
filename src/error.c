/* --------------------------------------------------------------------------
	Scandal Error Handling
	File name: scandal_error.c
	Author: David Snowdon
 
    Copyright (C) David Snowdon, 2009. 

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

#include <scandal/error.h>
#include <scandal/engine.h>
#include <scandal/types.h>
#include <scandal/message.h>

#include <project/scandal_config.h>

u08  last_scandal_error = 0;
u08  last_user_error = 0;
u32  num_errors = 0;

/* Scandal error */
u08  scandal_get_last_scandal_error(){
	return last_scandal_error;
}

void scandal_do_scandal_err(u08  err){
	last_scandal_error = err;
	scandal_send_scandal_error(err);
	num_errors++;
}

/* User error */
u08  scandal_get_last_user_error(){
	return last_user_error;
}

void scandal_do_user_err(u08  err){
	last_user_error = err;
	scandal_send_user_error(err);
	num_errors++;
}

u32 scandal_get_num_errors(void){
	return num_errors;
}

void do_fatal_error(u08 err){

}
