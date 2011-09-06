/*
 *  scandal_obligations.h
 *
 *  Created by David Snowdon on Thu Jul 18 2002.
 *  Copyright (c) David Snowdon, 2002.
 *
 *  Defines the functions to be called when implementing a node conforming to the scandal specification.
 *  These functions must be serviced in an appropriate fashion (as described in the comments supporting
 *  the prototypes in this file).
 *
 *  See scandal_requirements.h for a description of functions which must be made available by a board's
 *  software package in order to operate.
 */

#ifndef __SCANDAL_OBLIGATIONS__
#define __SCANDAL_OBLIGATIONS__

#include <scandal/types.h>
#include <scandal/can.h>

void scandal_reset_node(void);
void scandal_user_do_first_run(void);
u08 scandal_user_do_config(u08 param, s32 value, s32 value2);
u08 scandal_user_handle_message(can_msg *msg);
u08 scandal_user_handle_command(u08 command, u08 *data);
#endif
