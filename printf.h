/*
 * printf.h
 *
 * Local printf style function for openLRSng
 */
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include "serial.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Main entry point, note the format specifier
 */
extern void do_printf(LRS_Serial *ser, const char *format, ...) __attribute__ ((format (printf, 2, 3)));
extern void do_puts(LRS_Serial *ser, const char *str);

/*
 * printf() style macro, forces string constant to be in PROGMEM.
 */
#define lrs_printf(__s, __x, ...)      do_printf(__s, PSTR(__x), ##__VA_ARGS__)
#define lrs_printf_c(__s, __x, ...)    do_printf(__s, __x, ##__VA_ARGS__)
#define lrs_puts(__s, __p)             do_puts(__s, PSTR(__p))
#define lrs_puts_c(__s, __p)           do_puts(__s, __p)
#define lrs_putc(__s, __c)             LRS_SerialWrite(__s, __c)
#define lrs_getc(__s)                  LRS_SerialRead(__s)
#define lrs_inputPending(__s)          LRS_SerialAvailable(__s)

#ifdef __cplusplus
} // extern "C"
#endif
