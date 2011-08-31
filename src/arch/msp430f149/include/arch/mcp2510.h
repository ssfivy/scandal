/* --------------------------------------------------------------------------                                 
    MCP2515 Driver
    File name: mcp2510.h
    Author: David Snowdon, David Favaloro
    Description: Part of the USBCAN program

    Copyright (C) David Snowdon, 2002. 
    
    Date: 25-04-2002
   -------------------------------------------------------------------------- */

/* 
 * This file is part of Sunswift MCP2515 Driver.
 * 
 * Sunswift MCP2515 Driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * Sunswift MCP2515 Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with Sunswift MCP2515 Driver.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef __MCP2510__
#define __MCP2510__

#include <scandal_can.h>
#include <scandal_types.h>
#include <scandal_config.h>

/* -------------------------------------------------------------------------
   Function Prototypes
   ------------------------------------------------------------------------- */
/* MCP2510 Utility Functions */
void MCP2510_set_mode(u08 mode);
u08  MCP2510_get_mode(void);
u08  MCP2510_transmit_message(u32 id, u08*  buf, u08 size, u08 priority);
u08  MCP2510_receive_message(u32* id, u08* buf, u08* length);
/* MCP2510 Prototypes */
void MCP2510_reset(void);
void MCP2510_read(u08	addr, u08* buf, u08 num_bytes);
void MCP2510_write(u08	addr, u08* buf, u08 num_bytes);
void MCP2510_RTS(u08 buf_bitfield);
u08  MCP2510_read_status(void);
void MCP2510_bit_modify(u08 addr, u08 mask, u08 data);
u08  MCP2510_bit_timing(u08 mode);
u08  MCP2510_read_tx_errors(void);
u08  MCP2510_read_rx_errors(void);
void MCP2510_set_clkout_mode(u08 mode);

/* -------------------------------------------------------------------------
   MODES
   ------------------------------------------------------------------------- */
#define MCP2510_NORMAL_MODE		0x00
#define MCP2510_SLEEP_MODE		0x01
#define MCP2510_LOOPBACK_MODE		0x02
#define MCP2510_LISTEN_MODE            	0x03
#define MCP2510_CONFIGURATION_MODE	0x04

/* -------------------------------------------------------------------------
   CLKOUT modes 
   ------------------------------------------------------------------------- */

#define CANCTRL_CLKPRE_1   0
#define CANCTRL_CLKPRE_2   1
#define CANCTRL_CLKPRE_4   2
#define CANCTRL_CLKPRE_8   3

/* -------------------------------------------------------------------------
   RECEIVE BUFFER MODES
   -----------------------------------------------------------------------*/

#define MCP2510_RECEIVE_ALL		0x03
#define MCP2510_RECEIVE_EXTENDED	0x02
#define MCP2510_RECEIVE_STANDARD	0x01
#define MCP2510_RECEIVE_EXT_STD		0x00

/* -------------------------------------------------------------------------
   USEFUL MASKS
   ------------------------------------------------------------------------- */
#define TXBNCTRL_TXREQ_MASK		(1<<trTXREQ0)
#define TXBNCTRL_TXP_MASK		((1<<trTXP10) | (1<<trTXP00))
/* -------------------------------------------------------------------------
   TRANSMIT REGISTER OFFSETS
   Offset to a particular transmit register from the TXBNCTRL register
   ------------------------------------------------------------------------- */
#define TXBNCTRL_OFFSET	0x00
#define TXBNSIDH_OFFSET	0x01
#define TXBNSIDL_OFFSET	0x02
#define TXBNEID8_OFFSET	0x03
#define TXBNEID0_OFFSET	0x04
#define TXBNDLC_OFFSET	0x05
#define TXBND0_OFFSET	0x06
#define TXBND1_OFFSET	0x07
#define TXBND2_OFFSET	0x08
#define TXBND3_OFFSET	0x09
#define TXBND4_OFFSET	0x0A
#define TXBND5_OFFSET	0x0B
#define TXBND6_OFFSET	0x0C
#define TXBND7_OFFSET	0x0D

/* -------------------------------------------------------------------------
   COMMAND SET
   ------------------------------------------------------------------------- */
#define MCP2510_RESET_COMMAND	0xC0 	  /*! \def   MCP2510_RESET_COMMAND
					      \brief Reset the controller */
#define	MCP2510_READ_COMMAND	0x03      /*! \def   MCP2510_READ_COMMAND
					      \brief Read from the MCP2510 */
#define	MCP2510_WRITE_COMMAND	0x02      /*! \def   MCP2510_WRITE_COMMAND
					      \brief Write to the MCP2510 */

#define	MCP2510_RTS_COMMAND	0x80      /*! \def MCP2510_RTS_COMMAND
					      \brief Request to send command.

					      Sets the TXREQ bit for one of the
 					      three transmit buffers.
					      Select which buffer by setting one of
					      the last three bits */
#define	MCP2510_RTS0_COMMAND	0x81	  /*! Send buffer 1 */
#define	MCP2510_RTS1_COMMAND	0x82      /*! Send buffer 2 */
#define MCP2510_RTS2_COMMAND	0x84      /*! Send buffer 3 */
#define MCP2510_READSTATUS_COMMAND	0xA0	/* Polling command that outputs status
   					   bits for transmit/recieve functions */
#define MCP2510_BITMODIFY_COMMAND	0x05	/* Bit modify selected registers */



/* -------------------------------------------------------------------------
   REGISTER DEFINITIONS
   ------------------------------------------------------------------------- */
#define RXF0SIDH	0x00
#define RXF0SIDL	0x01
#define RXF0EID8	0x02
#define RXF0EID0	0x03
#define RXF1SIDH	0x04
#define RXF1SIDL	0x05
#define RXF1EID8	0x06
#define RXF1EID0	0x07
#define RXF2SIDH	0x08
#define RXF2SIDL	0x09
#define RXF2EID8	0x0A
#define RXF2EID0	0x0B
#define BFPCTRL	0x0C
#define TXRTSCTRL	0x0D
#define CANSTAT	0x0E
#define CANCTRL	0x0F

#define RXF3SIDH	0x10
#define RXF3SIDL	0x11
#define RXF3EID8	0x12
#define RXF3EID0	0x13
#define RXF4SIDH	0x14
#define RXF4SIDL	0x15
#define RXF4EID8	0x16
#define RXF4EID0	0x17
#define RXF5SIDH	0x18
#define RXF5SIDL	0x19
#define RXF5EID8	0x1A
#define RXF5EID0	0x1B
#define TEC	0x1C
#define REC         0x1D
#define CANSTAT1	0x1E
#define CANCTRL1	0x1F

#define RXM0SIDH	0x20
#define RXM0SIDL	0x21
#define RXM0EID8	0x22
#define RXM0EID0	0x23
#define RXM1SIDH	0x24
#define RXM1SIDL	0x25
#define RXM1EID8	0x26
#define RXM1EID0	0x27
#define CNF3	0x28
#define CNF2	0x29
#define CNF1	0x2A
#define CANINTE	0x2B
#define CANINTF	0x2C
#define EFLG	0x2D
#define CANSTAT2	0x2E
#define CANCTRL2	0x2F

#define TXB0CTRL	0x30
#define TXB0SIDH	0x31
#define TXB0SIDL	0x32
#define TXB0EID8	0x33
#define TXB0EID0	0x34
#define TXB0DLC	0x35
#define TXB0D0	0x36
#define TXB0D1	0x37
#define TXB0D2	0x38
#define TXB0D3	0x39
#define TXB0D4	0x3A
#define TXB0D5	0x3B
#define TXB0D6	0x3C
#define TXB0D7	0x3D
#define CANSTAT3	0x3E
#define CANCTRL3	0x3F

#define TXB1CTRL	0x40
#define TXB1SIDH	0x41
#define TXB1SIDL	0x42
#define TXB1EID8	0x43
#define TXB1EID0	0x44
#define TXB1DLC	0x45
#define TXB1D0	0x46
#define TXB1D1	0x47
#define TXB1D2	0x48
#define TXB1D3	0x49
#define TXB1D4	0x4A
#define TXB1D5	0x4B
#define TXB1D6	0x4C
#define TXB1D7	0x4D
#define CANSTAT4	0x4E
#define CANCTRL4	0x4F

#define TXB2CTRL	0x50
#define TXB2SIDH	0x51
#define TXB2SIDL	0x52
#define TXB2EID8	0x53
#define TXB2EID0	0x54
#define TXB2DLC	0x55
#define TXB2D0	0x56
#define TXB2D1	0x57
#define TXB2D2	0x58
#define TXB2D3	0x59
#define TXB2D4	0x5A
#define TXB2D5	0x5B
#define TXB2D6	0x5C
#define TXB2D7	0x5D
#define CANSTAT5	0x5E
#define CANCTRL5	0x5F

#define RXB0CTRL	0x60
#define RXB0SIDH	0x61
#define RXB0SIDL	0x62
#define RXB0EID8	0x63
#define RXB0EID0	0x64
#define RXB0DLC	0x65
#define RXB0D0	0x66
#define RXB0D1	0x67
#define RXB0D2	0x68
#define RXB0D3	0x69
#define RXB0D4	0x6A
#define RXB0D5	0x6B
#define RXB0D6	0x6C
#define RXB0D7	0x6D
#define CANSTAT6	0x6E
#define CANCTRL6	0x6F

#define RXB1CTRL	0x70
#define RXB1SIDH	0x71
#define RXB1SIDL	0x72
#define RXB1EID8	0x73
#define RXB1EID0	0x74
#define RXB1DLC	0x75
#define RXB1D0	0x76
#define RXB1D1	0x77
#define RXB1D2	0x78
#define RXB1D3	0x79
#define RXB1D4	0x7A
#define RXB1D5	0x7B
#define RXB1D6	0x7C
#define RXB1D7	0x7D
#define CANSTAT7	0x7E
#define CANCTRL7	0x7F

/* -------------------------------------------------------------------------
   BIT DEFINITIONS
   ------------------------------------------------------------------------- */

/* Bit definitions BFPCTRL */
#define trB1BFS		5
#define trB0BFS		4
#define trB1BFE		3
#define trB0BFE		2
#define trB1BFM		1
#define trB0BFM		0

/* Bit definitions TXRTSCTRL */
#define trB2RTS		5
#define trB1RTS		4
#define trB0RTS		3
#define trB2RTSM	2
#define trB1RTSM	1
#define trB0RTSM	0

/* Bit definitions CANSTAT */
#define trOPMOD2	7
#define trOPMOD1	6
#define trOPMOD0	5
#define trICOD2		3
#define trICOD1		2
#define trICOD0		1

/* Bit definitions CANCTRL */
#define trREQOP2	7
#define trREQOP1	6
#define trREQOP0	5
#define trABAT		4
#define trCLKEN		2
#define trCLKPRE1	1
#define trCLKPRE0	0

/* Bit definitions CNF3 */
#define trWAKFIL	6
#define trPHSEG22	2
#define trPHSEG21	1
#define trPHSEG20	0

/* Bit definitions CNF2 */
#define trBTLMODE	7
#define trSAM		6
#define trPHSEG12	5
#define trPHSEG11	4
#define trPHSEG10	3
#define trPHSEG2	2
#define trPHSEG1	1
#define trPHSEG0	0

/* Bit definitions CNF1 */
#define trSJW1		7
#define trSJW0		6
#define trBRP5		5
#define trBRP4		4
#define trBRP3		3
#define trBRP2		2
#define trBRP1		1
#define trBRP0		0

/* Bit definitions CANINTE */
#define trMERRE		7
#define trWAKIE		6
#define trERRIE		5
#define trTX2IE		4
#define trTX1IE		3
#define trTX0IE		2
#define trRX1IE		1
#define trRX0IE		0

/* Bit definitions CANINTF */
#define trMERRF		7
#define trWAKIF		6
#define trERRIF		5
#define trTX2IF		4
#define trTX1IF		3
#define trTX0IF		2
#define trRX1IF		1
#define trRX0IF		0

/* Bit definitions EFLG */
#define trRX1OVR	7
#define trRX0OVR	6
#define trTXB0		5
#define trTXEP		4
#define trRXEP		3
#define trTXWAR		2
#define trRXWAR		1
#define trEWARN		0

/* Bit definitions TXB0CTRL */
#define trABTF0		6
#define trMLOA0		5
#define trTXERR0	4
#define trTXREQ0	3
#define trTXP10		1
#define trTXP00		0

/* Bit definitions TXB1CTRL */
#define trABTF1		6
#define trMLOA1		5
#define trTXERR1	4
#define trTXREQ1	3
#define trTXP11		1
#define trTXP01		0

/* Bit definitions TXB2CTRL */
#define trABTF2		6
#define trMLOA2		5
#define trTXERR2	4
#define trTXREQ2	3
#define trTXP12		1
#define trTXP02		0

/* Bit definitions RXB0CTRL */
#define trRXM10		6
#define trRXM00		5
#define trRXRTR0	3
#define trBUKT01	2
#define trBUKT00	1
#define trFILHIT00	0

/* Bit definitions RXB1CTRL */
#define trRXM11		6
#define trRXM01		5
#define trRXRTR1	3
#define trFILHIT12	2
#define trFILHIT11	1
#define trFILHIT10	0
/* Bit definitions TXBNSIDL/RXNSIDL */
#define EXIDE           3

/* Bit definitions for word returned by READ_STATUS */
#define STATUS_RX0IF	0x00
#define STATUS_RX1IF	0x01
#define STATUS_TXB0CNTL	0x02
#define STATUS_TX0IF	0x03
#define STATUS_TXB1CNTL	0x04
#define STATUS_TX1IF	0x05
#define STATUS_TXB2CNTL	0x06
#define STATUS_TX2IF	0x07

#endif
