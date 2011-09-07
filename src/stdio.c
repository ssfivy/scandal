#include <scandal/stdio.h>
#include <arch/uart.h>

#include <string.h>
#include <stdarg.h>

#define putchar(c) UART_putchar(c)

static void UART_printchar(char **str, int c) {
	if (str) {
		**str = c;
		++(*str);
	} else
		(void)UART_putchar(c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int UART_prints(char **out, const char *string, int width, int pad) {
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

static int UART_printi(char **out, int i, int b, int sg, int width, int pad, int letbase) {
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
static unsigned UART_printfloat(char **out, double dbl, unsigned frac_len) {
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

static int UART_print(char **out, const char *format, va_list argp) {
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

int UART_printf(const char *format, ...) {
	va_list argp;
	int len;
	va_start(argp, format);
	len = UART_print(0, format, argp);
	va_end(argp);
	return len;
}

int UART_sprintf(char *out, const char *format, ...) {
	va_list argp;
	int len;
	va_start(argp, format);
	len = UART_print(&out, format, argp);
	va_end(argp);
	return len;
}
