/*
 *  scandal_types.h
 *
 *  Created by David Snowdon on Wed Jul 24, 2002.
 *  Copyright (c) 2002. All rights reserved.
 *  
 *  Defines a standard set of types for a variety of compilers. 
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

/* We assume that the standard C types will be provided by
 *  the compiler. 
 *  e.g. char, short, long, int, float 
 */

/*! \todo Define types expected of the compiler more formally */
/*! \todo Use uint8_t instead of u08, etc... */

#ifndef __SCANDAL_TYPES__
#define __SCANDAL_TYPES__

/* This #ifdef should really check for AVRGCC but I can't
   work out what the #define is for it */
#ifdef AVR
/* An AVR compiler is being used */
#include <inttypes.h>

/* Basic types */
typedef unsigned char     u08;
typedef          char     s08;
typedef unsigned short    u16;
typedef          short    s16;
typedef unsigned long     u32;
typedef          long     s32;
typedef          int64_t  s64;
typedef          uint64_t u64;

#define AVAILABLE_64

#else
#ifdef __PCM__
/*The CCS PCM PIC Compiler is being used */
#warning "compiler doesn't support 64bit stuff"
typedef unsigned int8  u08;
typedef signed int8    s08;
typedef unsigned int16 u16;
typedef signed int16   s16;
typedef unsigned int32 u32; 

#else
#ifdef MSP430
#include <sys/inttypes.h>
/* MSPGCC is being used */
typedef uint8_t        u08;
typedef int8_t         s08;
typedef uint16_t       u16;
typedef int16_t        s16;
typedef uint32_t       u32;
typedef int32_t        s32;
typedef int64_t        s64;
typedef uint64_t       u64;

#define AVAILABLE_64

#else
#ifdef LPC11C14
#include <arch/type.h>

typedef uint8_t        u08;
typedef int8_t         s08;
typedef uint16_t       u16;
typedef int16_t        s16;
typedef uint32_t       u32;
typedef int32_t        s32;
typedef int64_t        s64;
typedef uint64_t       u64;

#define AVAILABLE_64

#else
#if 0 
#warning "compiler doesn't support 64bit stuff"  
#endif
/* Basic types */
typedef unsigned char  u08;
typedef          char  s08;
typedef unsigned short u16;
typedef          short s16;
typedef unsigned long  u32;
typedef          long  s32;
typedef int64_t        s64;
typedef uint64_t       u64;

#endif
#endif
#endif
#endif

/* Generic types (Defined in terms of the above) */
//typedef u08            bool; 

#endif
