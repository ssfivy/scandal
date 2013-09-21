/*****************************************************************************
 *   uart.c:  UART API file for NXP LPC11xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.12.07  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/

#include <project/driver_config.h>
#include <arch/uart.h>
#include <scandal/uart.h>
#include <scandal/leds.h>

#include <string.h>

#define UART_MAX_BUFFERS 4

volatile uint32_t UARTStatus;
volatile uint8_t  UARTTxEmpty = 1;
volatile char *UARTBuffer;
volatile uint32_t UARTCount = 0;

static struct UART_buffer_descriptor *current_buffer;
static int line_read = 0;

/*****************************************************************************
** Function name:		UART_IRQHandler
**
** Descriptions:		UART interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART_IRQHandler(void) {
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy = Dummy;

	IIRValue = LPC_UART->IIR;

	IIRValue >>= 1;			/* skip pending bit in IIR */
	IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */

	if (IIRValue == IIR_RLS) {		/* Receive Line Status */
		LSRValue = LPC_UART->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UARTStatus = LSRValue;
			Dummy = LPC_UART->RBR;	/* Dummy read on RX to clear 
					interrupt, then bail out */
			return;
		}

		if (LSRValue & LSR_RDR) { /* Receive Data Ready */			
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			if (current_buffer->write_pos == current_buffer->size) {
				current_buffer->overflow = 1;
				current_buffer->write_pos = 0;
				return;
			}
			current_buffer->buf[current_buffer->write_pos++] = LPC_UART->RBR;
		}

	} else if (IIRValue == IIR_RDA) {	/* Receive Data Available */
		/* Receive Data Available */
		if (current_buffer->write_pos == current_buffer->size) {
			current_buffer->overflow = 1;
			current_buffer->write_pos = 0;
			return;
		}
		current_buffer->buf[current_buffer->write_pos++] = LPC_UART->RBR;

	} else if (IIRValue == IIR_CTI) {	/* Character timeout indicator */
		/* Character Time-out indicator */
		UARTStatus |= 0x100;		/* Bit 9 as the CTI error */

	} else if (IIRValue == IIR_THRE) {	/* THRE, transmit holding register empty */
		/* THRE interrupt */
		LSRValue = LPC_UART->LSR;		/* Check status in the LSR to see if
					valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UARTTxEmpty = 1;
		} else {
			UARTTxEmpty = 0;
		}
	}
	return;
}

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART0 port, setup pin select,
**				clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
** 
*****************************************************************************/
void UART_Init(uint32_t baudrate) {
	UART_Init_11xx(baudrate);
}

void UART_Init_11xx(uint32_t baudrate) {
  uint32_t Fdiv;
  uint32_t regVal;

  UARTTxEmpty = 1;
  UARTCount = 0;
  
  NVIC_DisableIRQ(UART_IRQn);

  LPC_IOCON->PIO1_6 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_6 |= 0x01;     /* UART RXD */
  LPC_IOCON->PIO1_7 &= ~0x07;	
  LPC_IOCON->PIO1_7 |= 0x01;     /* UART TXD */

  /* Enable UART clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
  LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

  LPC_UART->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
  regVal = LPC_SYSCON->UARTCLKDIV;

  Fdiv = (((SystemCoreClock*LPC_SYSCON->SYSAHBCLKDIV)/regVal)/16)/baudrate ;	/*baud rate */

  LPC_UART->DLM = Fdiv / 256;							
  LPC_UART->DLL = Fdiv % 256;
  LPC_UART->LCR = 0x03;		/* DLAB = 0 */
  LPC_UART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

  /* Read to clear the line status. */
  regVal = LPC_UART->LSR;

  /* Ensure a clean start, no data in either TX or RX FIFO. */
// CodeRed - added parentheses around comparison in operand of &
  while (( LPC_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );

  while ( LPC_UART->LSR & LSR_RDR ) {
	regVal = LPC_UART->RBR;	/* Dump data from RX FIFO */
  }
 
  /* Enable the UART Interrupt */
  NVIC_EnableIRQ(UART_IRQn);

#if CONFIG_UART_ENABLE_INTERRUPT==1
#if CONFIG_UART_ENABLE_TX_INTERRUPT==1
  LPC_UART->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART interrupt */
#else
  LPC_UART->IER = IER_RBR | IER_RLS;	/* Enable UART interrupt */
#endif
#endif
  return;
}

/*****************************************************************************
** Function name:		UART_Send
**
** Descriptions:		Send a block of data to the UART 0 port based
**				on the data length
**
** parameters:		buffer pointer, and data length
** Returned value:	None
** 
*****************************************************************************/
void UART_Send(uint8_t *BufferPtr, uint32_t Length) {
  
  while ( Length != 0 )
  {
	  /* THRE status, contain valid data */
#if CONFIG_UART_ENABLE_TX_INTERRUPT==1
	  /* Below flag is set inside the interrupt handler when THRE occurs. */
      while ( !(UARTTxEmpty & 0x01) );
	  LPC_UART->THR = *BufferPtr;
      UARTTxEmpty = 0;	/* not empty in the THR until it shifts out */
#else
	  while ( !(LPC_UART->LSR & LSR_THRE) );
	  LPC_UART->THR = *BufferPtr;
#endif
      BufferPtr++;
      Length--;
  }
  return;
}

void UART_putchar(char c) {
	uint8_t c1 = c;
	UART_Send(&c1, 1);
}

/* If the write position is not the same as read position, a new character has been read. Note
that this function does not detect overflow, hence if the data is not read quickly enough then 
it may be lost.  */
u08 UART_is_received(void) {
	if(current_buffer->write_pos!= current_buffer->last_read_pos) { 
		return 1; 
	} else { 
		return 0; 	
	}
}

/* Scandal UART_ReceiveByte. Do not call this function without checking whether data is received through
UART_is_received function*/
u08  UART_ReceiveByte(void) {
	if(current_buffer->last_read_pos >= current_buffer->size) { 
 		current_buffer->last_read_pos=0; 
	}
	return current_buffer->buf[current_buffer->last_read_pos++]; 	 
}

/* Wait until a '\n' character comes, then return. The user should then
 * have the line in their buffer. This can suffer from overwrite if the user doesn't get
 * to the buffer in time or the characters are coming in too fast */
char *UART_readline(struct UART_buffer_descriptor *desc, char *buf, uint32_t size) {
	int i;
	desc->buf = buf;
	desc->write_pos = 0;
	desc->size = size;

	current_buffer = desc;

	/* check if we have a full line. Otherwise return -1 */
	for (i = 0; i <= current_buffer->write_pos; i++) {
		if (current_buffer->buf[i] == '\r') {
			current_buffer->write_pos = 0;
			return current_buffer->buf;
		} else {
			goto out;
		}
	}

out:
	return NULL;
}

void UART_init_double_buffer(struct UART_buffer_descriptor *desc_1, char *buf_1, uint32_t size_1,
	struct UART_buffer_descriptor *desc_2, char *buf_2, uint32_t size_2) {
	desc_1->buf = buf_1;
	desc_1->size = size_1;
	desc_1->write_pos = 0;
	desc_1->overflow = 0;
	desc_1->last_read_pos=0;

	desc_2->buf = buf_2;
	desc_2->size = size_2;
	desc_2->write_pos = 0;
	desc_2->overflow = 0;
	desc_2->last_read_pos=0;

	current_buffer = desc_1;
}

/* Wait until a '\n' character comes, then return. The user should then
 * have the line in their buffer. This can suffer from overwrite if the user doesn't get
 * to the buffer in time or the characters are coming in too fast */
char *UART_readline_double_buffer(struct UART_buffer_descriptor *desc_1, struct UART_buffer_descriptor *desc_2) {
	int i;

	if (line_read) {
		line_read = 0;

		if (current_buffer == desc_1) {
			desc_2->write_pos = 0;
			current_buffer = desc_2;
		} else {
			desc_1->write_pos = 0;
			current_buffer = desc_1;
		}
		memset(current_buffer->buf, 0, current_buffer->size);
	}

	/* check if we have a full line. */
	for (i = 0; i <= current_buffer->write_pos; i++) {
		if (current_buffer->buf[i] == '\n') {
			/* make it into a real string so we can printf it */
			current_buffer->buf[current_buffer->write_pos+1] = '\0';
			line_read = 1;
			return current_buffer->buf;
		}
	}
	return NULL;
}

/* 
	Initialise single buffer
*/
void UART_init_buffer(struct UART_buffer_descriptor *desc_1, char* buf_1, uint32_t size_1) {
	desc_1->buf = buf_1;
	desc_1->size = size_1;
	desc_1->write_pos = 0;
	desc_1->overflow = 0;
	desc_1->last_read_pos=0;
	
	current_buffer = desc_1; 	
}

