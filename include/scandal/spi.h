/*! ------------------------------------------------------------------------- 
	scandal_spi.h
 
    SPI Master Driver interface functions
	 
	Author: David Snowdon 
	Date: 19/06/02 
 
    Copyright (C) 2002 David Snowdon. 
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

#ifndef __SCANDAL_SPI__ 
#define __SCANDAL_SPI__ 

#include <scandal/types.h>

/* SPI Driver Prototypes */ 
u08 init_spi(void);  
u08 spi_select_device(u08	device);  
void spi_deselect_all(void); 
u08 spi_transfer(u08 out_data); 

#endif 
