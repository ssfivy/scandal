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

#include <string.h>
#include <stdarg.h>

#if CONFIG_ENABLE_DRIVER_UART==1
#include <arch/uart.h>

volatile uint32_t UARTStatus;
volatile uint8_t  UARTTxEmpty = 1;
volatile uint8_t  UARTBuffer[UART_BUFSIZE];
volatile uint32_t UARTCount = 0;

#if CONFIG_UART_DEFAULT_UART_IRQHANDLER==1
/*****************************************************************************
** Function name:		UART_IRQHandler
**
** Descriptions:		UART interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART_IRQHandler(void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  IIRValue = LPC_UART->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if (IIRValue == IIR_RLS)		/* Receive Line Status */
  {
    LSRValue = LPC_UART->LSR;
    /* Receive Line Status */
    if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
    {
      /* There are errors or break interrupt */
      /* Read LSR will clear the interrupt */
      UARTStatus = LSRValue;
      Dummy = LPC_UART->RBR;	/* Dummy read on RX to clear 
								interrupt, then bail out */
      return;
    }
    if (LSRValue & LSR_RDR)	/* Receive Data Ready */			
    {
      /* If no error on RLS, normal ready, save into the data buffer. */
      /* Note: read RBR will clear the interrupt */
      UARTBuffer[UARTCount++] = LPC_UART->RBR;
      if (UARTCount == UART_BUFSIZE)
      {
        UARTCount = 0;		/* buffer overflow */
      }	
    }
  }
  else if (IIRValue == IIR_RDA)	/* Receive Data Available */
  {
    /* Receive Data Available */
    UARTBuffer[UARTCount++] = LPC_UART->RBR;
    if (UARTCount == UART_BUFSIZE)
    {
      UARTCount = 0;		/* buffer overflow */
    }
  }
  else if (IIRValue == IIR_CTI)	/* Character timeout indicator */
  {
    /* Character Time-out indicator */
    UARTStatus |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if (IIRValue == IIR_THRE)	/* THRE, transmit holding register empty */
  {
    /* THRE interrupt */
    LSRValue = LPC_UART->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
    if (LSRValue & LSR_THRE)
    {
      UARTTxEmpty = 1;
    }
    else
    {
      UARTTxEmpty = 0;
    }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:		ModemInit
**
** Descriptions:		Initialize UART0 port as modem, setup pin select.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void ModemInit( void )
{
  LPC_IOCON->PIO2_0 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_0 |= 0x01;     /* UART DTR */
  LPC_IOCON->PIO0_7 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO0_7 |= 0x01;     /* UART CTS */
  LPC_IOCON->PIO1_5 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_5 |= 0x01;     /* UART RTS */
#if 1 
  LPC_IOCON->DSR_LOC	= 0;
  LPC_IOCON->PIO2_1 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_1 |= 0x01;     /* UART DSR */

  LPC_IOCON->DCD_LOC	= 0;
  LPC_IOCON->PIO2_2 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_2 |= 0x01;     /* UART DCD */

  LPC_IOCON->RI_LOC	= 0;
  LPC_IOCON->PIO2_3 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO2_3 |= 0x01;     /* UART RI */

#else
  LPC_IOCON->DSR_LOC = 1;
  LPC_IOCON->PIO3_1 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO3_1 |= 0x01;     /* UART DSR */

  LPC_IOCON->DCD_LOC = 1;
  LPC_IOCON->PIO3_2 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO3_2 |= 0x01;     /* UART DCD */

  LPC_IOCON->RI_LOC = 1;
  LPC_IOCON->PIO3_3 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO3_3 |= 0x01;     /* UART RI */
#endif
  LPC_UART->MCR = 0xC0;          /* Enable Auto RTS and Auto CTS. */			
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
void UARTInit(uint32_t baudrate)
{
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
  while ( LPC_UART->LSR & LSR_RDR )
  {
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
** Function name:		UARTSend
**
** Descriptions:		Send a block of data to the UART 0 port based
**				on the data length
**
** parameters:		buffer pointer, and data length
** Returned value:	None
** 
*****************************************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
  
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
#endif

static void UART_putchar(int c) {
	uint8_t c1 = c;
	UARTSend(&c1, 1);
}

#define putchar(c) UART_putchar(c)

static void UART_printchar(char **str, int c)
{
	if (str) {
		**str = c;
		++(*str);
	} else
		(void)UART_putchar(c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int UART_prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr)
			++len;

		if (len >= width)
			width = 0;
		else
			width -= len;

		if (pad & PAD_ZERO)
			padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			UART_printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		UART_printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		UART_printchar (out, padchar);
		++pc;
	}

	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int UART_printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return UART_prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			UART_printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + UART_prints (out, s, width, pad);
}

/* A simple print float implementation. Takes the decimal part, prints it
 * as an int. Subtracts decimal part, and multiplies by 10^digits_past_point
 * and prints that as the fraction part.
 * Warning: There is probably going to be rounding error in here.
 */
static unsigned UART_printfloat(char **out, double dbl, unsigned frac_len)
{
	int len = 0;
	int frac = 0;
	int dec = (int)dbl;
	int i;
	int multiplier;

	if (dbl < 0.0)
		multiplier = -1;
	else
		multiplier = 1;

	len += UART_printf("%d", dec);

	for (i = 0; i < frac_len; i++)
		multiplier *= 10;

	frac = (int)((dbl - (double)dec) * multiplier);

	len += UART_printf(".%d", frac);

	return len;
}

static int UART_print(char **out, const char *format, va_list argp)
{
	int width, pad, dec_len, frac_len, frac;
	int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = dec_len = frac_len = frac = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; (*format >= '0' && *format <= '9') || *format == '.'; ++format) {
				width *= 10;
				width += *format - '0';

				if (*format == '.') {
					dec_len = width;
					width = 0;
					frac = 1;
				}

				if (frac == 1)
					frac_len = width;

			}
			if( *format == 's' ) {
				char *s = va_arg(argp, char *);
				pc += UART_prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				int i = va_arg(argp, int);
				pc += UART_printi (out, i, 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'f' ) {
				double f = va_arg(argp, double);
				pc += UART_printfloat (out, f, frac_len);
				continue;
			}
			if( *format == 'x' ) {
				int i = va_arg(argp, int);
				pc += UART_printi (out, i, 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				int i = va_arg(argp, int);
				pc += UART_printi (out, i, 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				unsigned int i = va_arg(argp, unsigned int);
				pc += UART_printi (out, (int)i, 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg(argp, int);
				scr[1] = '\0';
				pc += UART_prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			UART_printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	return pc;
}

/* assuming sizeof(void *) == sizeof(int) */

int UART_printf(const char *format, ...)
{
	va_list argp;
	int len;
	va_start(argp, format);
	len = UART_print(0, format, argp);
	va_end(argp);
	return len;
}

int UART_sprintf(char *out, const char *format, ...)
{
	va_list argp;
	int len;
	va_start(argp, format);
	len = UART_print(&out, format, argp);
	va_end(argp);
	return len;
}
/******************************************************************************
**                            End Of File
******************************************************************************/
