/****************************************************************************
 *   $Id:: can.c 3635 2010-06-02 00:31:46Z usb00423                         $
 *   Project: NXP LPC11xx CAN example
 *
 *   Description:
 *     This file contains CAN code example which include CAN 
 *     initialization, CAN interrupt handler, and APIs for CAN
 *     access.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/

/* CAN_ functions are non scandal specific.
 * This still could do with a bit of tidy. */

#include <project/driver_config.h>

#include <string.h> /* for memcpy */

#include <arch/can.h>
#include <arch/gpio.h>
#include <arch/timer32.h>

#ifdef CAN_UART_DEBUG
#warning CAN_UART debugging is enabled. If you need the UART for something else, you need to disable this in project_config.h.
#include <arch/uart.h>
#endif

#include <scandal/can.h>
#include <scandal/error.h>
#include <scandal/timer.h>

#define RECV_BUFF_DIVIDE 20 /* this gives 0-21 as recv buffers and 21-32 as tx buffers */

uint8_t recv_buf_used[MSG_OBJ_MAX]; /* this will be used to determine if a recv buffer is available */

#define CUSTOM_CONFIG 1
#define REORDER_DATA 1

/* statistics of all the interrupts */
volatile uint32_t BOffCnt = 0;
volatile uint32_t EWarnCnt = 0;
volatile uint32_t EPassCnt = 0;

uint32_t CANRxDone[MSG_OBJ_MAX];

message_object can_buff[MSG_OBJ_MAX];

#if CAN_DEBUG
uint32_t CANStatusLog[100];
uint32_t CANStatusLogCount = 0;
#endif

/* Scandal wrappers
 * *****************
 * for reference:
 * typedef struct can_mg {
 *   u32 id;
 *   u08 data[CAN_MSG_MAXSIZE];
 *   u08 length;
 * } can_msg;
 */

void init_can(void) {
	CAN_Init(BITRATE50K16MHZ);
}

/* Get a message from the CAN controller. */
u08 can_get_msg(can_msg* msg) {
	int i;
	uint8_t can_data[CAN_MSG_MAXSIZE/2];
	uint8_t can_timestamp[CAN_MSG_MAXSIZE/2];
	uint16_t priority;
	uint16_t msg_type;
	uint16_t node_address;
	uint16_t channel_num;

	for(i = 0; i < MSG_OBJ_MAX; i++) {
		if (CANRxDone[i] == TRUE) {
			int n = 0;
			CAN_decode_packet(i, (int32_t*)(&can_data), (uint32_t*)(&can_timestamp), &priority, &msg_type, &node_address, &channel_num);

			msg->id = can_buff[i].id;
			msg->length= CAN_MSG_MAXSIZE;
			/* ordering is important! */
			msg->data[7] = can_timestamp[0];
			msg->data[6] = can_timestamp[1];
			msg->data[5] = can_timestamp[2];
			msg->data[4] = can_timestamp[3];
			msg->data[3] = can_data[0];
			msg->data[2] = can_data[1];
			msg->data[1] = can_data[2];
			msg->data[0] = can_data[3];

#ifdef CAN_UART_DEBUG
			UART_printf("got a can message...\n\r");
			UART_printf(" id is               %d (0x%x)\n\r", can_buff[i].id, can_buff[i].id);
			UART_printf(" priority is         %d\n\r", priority);
			UART_printf(" node_address is     %d\n\r", node_address);
			UART_printf(" message type is     %d\n\r", msg_type);
			UART_printf(" channel_num is      %d\n\r", channel_num);
			UART_printf(" data value is:      %d (0x%x)\n\r", *((uint32_t*)&msg->data[0]), *((uint32_t*)&msg->data[0]));
			UART_printf(" timestamp value is: %d (0x%x)\n\r", *((uint32_t*)&msg->data[4]), *((uint32_t*)&msg->data[4]));
			for(n = 0; n < 8; n++)
				UART_printf("msg->data[%d] = %d (0x%x)\n\r", n, msg->data[n], msg->data[n]);
#endif

			return NO_ERR;
		}
	}
	return NO_MSG_ERR;
}

/* Send a message using the CAN controller */
u08 can_send_msg(can_msg *msg, u08 priority) {
	CAN_Send((uint16_t)priority, msg);
	return NO_ERR;
}

/* Register for a message type. Currently, each message that we want to register
 * for is given a specific message buffer. This limits the maximum number of in
 * channels to be 21 - 4 = 17. Look in scandal/engine.c for where this function
 * is called to see why. At the moment, I don't think there are any nodes with
 * large numbers of in channels. If we need to deal with this, it can be done 
 * in the future */
u08 can_register_id(u32 mask, u32 data, u08 priority) {
	int i;

	NVIC_DisableIRQ(CAN_IRQn);
	LPC_CAN->CNTL &= ~(CTRL_IE|CTRL_SIE|CTRL_EIE);

	for(i = 0; i < RECV_BUFF_DIVIDE; i++) {
		/* if we have run out of recv buffers, error out */
		if (i == RECV_BUFF_DIVIDE-1) {
			NVIC_EnableIRQ(CAN_IRQn);
			LPC_CAN->CNTL |= (CTRL_IE|CTRL_SIE|CTRL_EIE);
			return NO_ERR;

		/* find a free buffer and set up a filter */
		} else if (!recv_buf_used[i]) {
			CAN_set_up_filter(i, mask, data);
			recv_buf_used[i] = 1;
			NVIC_EnableIRQ(CAN_IRQn);
			LPC_CAN->CNTL |= (CTRL_IE|CTRL_SIE|CTRL_EIE);
#ifdef CAN_UART_DEBUG
			UART_printf("registering for a message type %d (0x%x)\n\r", data, data);
#endif
			return NO_ERR;
		}
	}
}

/* does nothing yet */
u08  can_baud_rate(u08 mode) {
	return 0;
}

/* this is not used on LPC11C14 */
void can_poll(void) {}

/* *******************
 * End Scandal wrappers
 */

void CAN_decode_packet(uint8_t msg_num, int32_t *data, uint32_t *timestamp,
	uint16_t *priority, uint16_t *type, uint16_t *node_address, uint16_t *channel_num) {

	*data         = (can_buff[msg_num].data[0] << 16) | (can_buff[msg_num].data[1]);
	*timestamp    = (can_buff[msg_num].data[2] << 16) | (can_buff[msg_num].data[3]);

	*channel_num  = ((can_buff[msg_num].id >> 0)  & 0x03FF);
	*node_address = ((can_buff[msg_num].id >> 10) & 0x00FF);
	*type         = ((can_buff[msg_num].id >> 18) & 0x00FF);
	*priority     = ((can_buff[msg_num].id >> 26) & 0x0007);

	CANRxDone[msg_num] = 0;
}

static inline void CAN_set_up_filter(uint8_t msg_id, uint32_t filter_mask, uint32_t filter_addr) {

	LPC_CAN->IF1_CMDMSK = WR|MASK|ARB|CTRL|DATAA|DATAB; //Configuring (Writing to) the message objects

	LPC_CAN->IF1_MSK1 = filter_mask & 0xFFFF; //Filtermask used to be ID_EXT_MASK
	LPC_CAN->IF1_MSK2 = MASK_MXTD | (filter_mask >> 16);

	LPC_CAN->IF1_ARB1 = (filter_addr) & 0xFFFF; //filteraddr used to be RX_EXT_MSG_ID + i
	LPC_CAN->IF1_ARB2 = ID_MVAL | ID_MTD | (filter_addr >> 16); // use this message object, extended
	LPC_CAN->IF1_ARB2 &= ~ID_DIR; // receive direction

	LPC_CAN->IF1_MCTRL = UMSK|RXIE|EOB|DLC_MAX;

	LPC_CAN->IF1_DA1 = 0x0000;
	LPC_CAN->IF1_DA2 = 0x0000;
	LPC_CAN->IF1_DB1 = 0x0000;
	LPC_CAN->IF1_DB2 = 0x0000;

	/* Transfer data to message RAM */
	LPC_CAN->IF1_CMDREQ = msg_id+1;

	while( LPC_CAN->IF1_CMDREQ & IFCREQ_BUSY );

}

/*****************************************************************************
** Function name:		MessageProcess
**
** Descriptions:		A valid message received, process message
**
** parameters:			Message Object number
** Returned value:		None
** 
*****************************************************************************/
void CAN_MessageProcess( uint8_t MsgNo )
{
  uint32_t MsgID;
  uint32_t *p_add;

#if BASIC_MODE
  LPC_CAN->IF2_CMDMSK = RD|MASK|ARB|TREQ|DATAA|DATAB;
  LPC_CAN->IF2_CMDREQ = IFCREQ_BUSY;    /* Start message transfer */
#else
  while ( LPC_CAN->IF2_CMDREQ & IFCREQ_BUSY );
  LPC_CAN->IF2_CMDMSK = RD|MASK|ARB|CTRL|INTPND|TREQ|DATAA|DATAB;	
  LPC_CAN->IF2_CMDREQ = MsgNo+1;    /* Start message transfer */
#endif	
  while ( LPC_CAN->IF2_CMDREQ & IFCREQ_BUSY );	/* Check new data bit */

  p_add = (uint32_t *)&can_buff[MsgNo];
  if( LPC_CAN->IF2_ARB2 & ID_MTD )	/* bit 28-0 is 29 bit extended frame */
  {
	/* mask off MsgVal and Dir */ 
	MsgID = (LPC_CAN->IF2_ARB1|((LPC_CAN->IF2_ARB2&0x5FFF)<<16));
  }
  else
  {
	/* bit 28-18 is 11-bit standard frame */
	MsgID = (LPC_CAN->IF2_ARB2 &0x1FFF) >> 2;
  }

  *p_add++ = MsgID;
  *p_add++ = LPC_CAN->IF2_MCTRL & 0x000F;	// Get Msg Obj Data length


#if REORDER_DATA
  uint32_t msghold;
  uint32_t msgleftshift;
  uint32_t msgrightshift;

	  msghold=LPC_CAN->IF2_DA1; //Stores the next two bytes in msghold
	  msgleftshift = (msghold & 0x000000FF)<<8; //Shifts the last byte left by 8 bits
	  msgrightshift = (msghold & 0x0000FF00)>>8; //Shifts the second last byte right by 8 bits
	  *p_add++ = msgleftshift | msgrightshift; //OR's the two shifted results to give the data in the right order

	  msghold=LPC_CAN->IF2_DA2; //Stores the next two bytes in msghold
	  msgleftshift = (msghold & 0x000000FF)<<8; //Shifts the last byte left by 8 bits
	  msgrightshift = (msghold & 0x0000FF00)>>8; //Shifts the second last byte right by 8 bits
	  *p_add++ = msgleftshift | msgrightshift; //OR's the two shifted results to give the data in the right order

	  msghold=LPC_CAN->IF2_DB1; //Stores the next two bytes in msghold
	  msgleftshift = (msghold & 0x000000FF)<<8; //Shifts the last byte left by 8 bits
	  msgrightshift = (msghold & 0x0000FF00)>>8; //Shifts the second last byte right by 8 bits
	  *p_add++ = msgleftshift | msgrightshift; //OR's the two shifted results to give the data in the right order

	  msghold=LPC_CAN->IF2_DB2; //Stores the next two bytes in msghold
	  msgleftshift = (msghold & 0x000000FF)<<8; //Shifts the last byte left by 8 bits
	  msgrightshift = (msghold & 0x0000FF00)>>8; //Shifts the second last byte right by 8 bits
	  *p_add++ = msgleftshift | msgrightshift; //OR's the two shifted results to give the data in the right order

#else
  *p_add++ = LPC_CAN->IF2_DA1;
  *p_add++ = LPC_CAN->IF2_DA2;
  *p_add++ = LPC_CAN->IF2_DB1;
  *p_add++ = LPC_CAN->IF2_DB2;
#endif
  return;
}

#if !POLLING
#if CONFIG_CAN_DEFAULT_CAN_IRQHANDLER==1
/*****************************************************************************
** Function name:		CAN_IRQHandler
**
** Descriptions:		Processing CAN interrupt
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void CAN_IRQHandler(void) 
{
  uint32_t canstat = canstat;
  uint32_t can_int, msg_no;

  while ( (can_int = LPC_CAN->INT) != 0 )
  {
	if ( can_int & CAN_STATUS_INTERRUPT )
	{
	  canstat = LPC_CAN->STAT;
#if CAN_DEBUG
	  CANStatusLog[CANStatusLogCount++] = canstat;
#endif
	  if ( canstat & STAT_EWARN )
	  {
		EWarnCnt++;
		return;
	  }
	  if ( canstat & STAT_BOFF )
	  {
		BOffCnt++;
		return;
	  }
	}
	else
	{
      if ( (canstat & STAT_LEC) == 0 ) 	/* NO ERROR */
	  {
		/* deal with RX only for now. */
		msg_no = can_int & 0x7FFF;
		if ( (msg_no >= 0x01) && (msg_no <= 0x20) )
		{
		  LPC_CAN->STAT &= ~STAT_RXOK;
		  CAN_MessageProcess( msg_no-1 ); //msg_no goes up from 1, msg_no ranges from 0
		  CANRxDone[msg_no-1] = TRUE;
		}
	  }
	}
  }	
  return;
}
#endif
#endif

/*****************************************************************************
** Function name:		CAN_Init
**
** Descriptions:		CAN clock, port initialization
**				
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void CAN_Init( uint32_t CANBitClk )
{
  LPC_SYSCON->PRESETCTRL |= (0x1<<3);
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<17);

  /* The USB D- and CAN RX share the dedicated pin. The USB D+ 
  and CAN TX share the dedicated pin. so, no IO configuration is 
  needed for CAN. */
  if ( !(LPC_CAN->CNTL & CTRL_INIT) )
  {
	/* If it's in normal operation already, stop it, reconfigure 
	everything first, then restart. */
	LPC_CAN->CNTL |= CTRL_INIT;		/* Default state */
  }

#if USE_DEFAULT_BIT_TIMING
  /* AHB clock is 48Mhz. The CAN clock divider is within CAN block, 
  set it to 8Mhz for now. Thus, default bit timing doesn't need to 
  be touched. */
  LPC_CAN->CLKDIV = 0x02;			/* Divided by 3 now for 16MHz */
   /* Start configuring bit timing */
  LPC_CAN->CNTL |= CTRL_CCE;
  LPC_CAN->BT = 0x2301;
  LPC_CAN->BRPE = 0x0000;
  /* Stop configuring bit timing */
  LPC_CAN->CNTL &= ~CTRL_CCE;
#else
  /* Be very careful with this setting because it's related to
  the input bitclock setting value in CANBitClk. */
  /* popular CAN clock setting assuming AHB clock is 48Mhz:
  CLKDIV = 1, CAN clock is 48Mhz/2 = 24Mhz
  CLKDIV = 2, CAN clock is 48Mhz/3 = 16Mhz
  CLKDIV = 3, CAN clock is 48Mhz/4 = 12Mhz
  CLKDIV = 5, CAN clock is 48Mhz/6 = 8Mhz */

  /* AHB clock is 48Mhz, the CAN clock is 1/3 AHB clock = 16Mhz */
  LPC_CAN->CLKDIV = 0x02;			/* Divided by 3 */
  
  /* Start configuring bit timing */
  LPC_CAN->CNTL |= CTRL_CCE;
  LPC_CAN->BT = CANBitClk;
  LPC_CAN->BRPE = 0x0000;
  /* Stop configuring bit timing */
  LPC_CAN->CNTL &= ~CTRL_CCE;
#endif

  /* Initialization finishes, normal operation now. */
  LPC_CAN->CNTL &= ~CTRL_INIT;
  while ( LPC_CAN->CNTL & CTRL_INIT );

#if (LOOPBACK_MODE | BASIC_MODE)
  LPC_CAN->CNTL |= CTRL_TEST;
  LPC_CAN->TEST &= ~((0x1<<2)|(0x1<<3)|(0x1<<4));
#if LOOPBACK_MODE
  LPC_CAN->TEST |= (0x1<<4);
#endif
#if BASIC_MODE
  LPC_CAN->TEST |= (0x1<<2);
#endif
#endif  

#if !POLLING
  /* Enable the CAN Interrupt */
  NVIC_EnableIRQ(CAN_IRQn);
	
  /* By default, auto TX is enabled, enable all related interrupts */
  LPC_CAN->CNTL |= (CTRL_IE|CTRL_SIE|CTRL_EIE);
#endif
  return;
}

void CAN_Send(uint16_t Pri, can_msg *msg) {
	uint32_t tx_addr = 0x1FFFFFFF & msg->id;
	uint8_t length = 8;
	uint8_t BufferPos; //Buffer Position (Counter variable to see if the buffer position is BUSY
	uint8_t BufferOffset=21; //The offset from which the buffer will start counting (Assumed 21 through 32)
	uint8_t BufferFree; //Status of the buffer being checked, 1 indicates busy
	uint32_t Buffers;

	/* Data is stored in can_msg->data[0-4], timestamp is stored in can_msg->data[4-7] */
	uint32_t can_data;
	uint32_t can_timestamp;
	memcpy(&can_data, msg->data, sizeof(uint32_t));
	memcpy(&can_timestamp, msg->data+sizeof(uint32_t), sizeof(uint32_t));

	/* MsgVal: 1, Mtd: 1, Dir: 1, ID = 0x200000 */
	LPC_CAN->IF1_ARB2 = ID_MVAL | ID_MTD | ID_DIR | (tx_addr >> 16);
	LPC_CAN->IF1_ARB1 = tx_addr & 0x0000FFFF;

	/* Mxtd: 1, Mdir: 1, Mask is 0x7FF */
	LPC_CAN->IF1_MSK2 = MASK_MXTD | MASK_MDIR | (ID_EXT_MASK >> 16);
	LPC_CAN->IF1_MSK1 = ID_EXT_MASK & 0x0000FFFF;

	LPC_CAN->IF1_MCTRL = UMSK|TXRQ|EOB|(length & DLC_MASK);

	/* put the data in the buffer register */
	LPC_CAN->IF1_DA1 = can_data & 0x0000FFFF;
	LPC_CAN->IF1_DA2 = ((can_data & 0xFFFF0000) >> 16);

	/* put the timestamp in the buffer register */
	LPC_CAN->IF1_DB1 = can_timestamp & 0x0000FFFF;
	LPC_CAN->IF1_DB2 = ((can_timestamp & 0xFFFF0000) >> 16);
	LPC_CAN->IF1_CMDMSK = WR|MASK|ARB|CTRL|TREQ|DATAA|DATAB;


	//Buffers=(((LPC_CAN->TXREQ2) & (0x0000FFFF)) << 16) | ((LPC_CAN->TXREQ1) & (0x0000FFFF));
	//BufferStat = BufferCheck(21);

	BufferPos=0;
	BufferFree=BufferCheck(BufferPos+BufferOffset);

	//BufferFree = (Buffer >> (20-1));

	//Spins here while BufferFree=1 (Current buffer is busy)

	while(BufferFree){
		BufferPos++;
		//BufferPos=BufferPos%13;

		if(BufferPos<12){ //Buffer Position should either be
			BufferFree=BufferCheck(BufferPos+BufferOffset);
		}else{
			BufferFree=1; //Force exit of the loop
		}

		//BufferFree=BufferCheck(BufferPos+BufferOffset);
	}


	if((BufferPos+BufferOffset)<=MSG_OBJ_MAX){ //Make sure we haven't gone beyond the maximum buffer size we're allowed to use.
		LPC_CAN->IF1_CMDREQ = (BufferPos+BufferOffset);
	}else{
		//Return some sort of error as all the buffers were full, maybe test by having an LED initially OFF
		//and turning it on here
	}
	while( LPC_CAN->IF1_CMDREQ & IFCREQ_BUSY ) {
	  ;/* Waits untill IF1 transfer thingy has done its duties */
	}
}

//Returns true of the buffer is BUSY so move on to the next buffer
uint8_t BufferCheck(uint8_t ToCheck){
	uint32_t BufferStatus;
	BufferStatus=(((LPC_CAN->TXREQ2) & (0x0000FFFF)) << 16) | ((LPC_CAN->TXREQ1) & (0x0000FFFF));

	//Return value to be 1 or 0 depending on if the buffer is free or not
	uint8_t ret = (((uint8_t) (BufferStatus >> (ToCheck -1))) & 0x01);

	return ret;
}
