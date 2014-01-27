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

extern SerialPort *stdIn;
extern SerialPort *stdOut;

/*
 * Main entry point, note the format specifier
 */
extern void do_snprintf(char *buf, size_t size, const char *format, ...) __attribute__ ((format (printf, 3, 4)));
extern void do_sprintf(char *buf, const char *format, ...) __attribute__ ((format (printf, 2, 3)));
extern void do_printf(const char *format, ...) __attribute__ ((format (printf, 1, 2)));
extern void do_puts(const char *str);

/*
 * printf() style macro, forces string constant to be in PROGMEM.
 */
#define lrs_snprintf(__s, __l, __x, ...)   do_snprintf(__s, __l, PSTR(__x), ##__VA_ARGS__)
#define lrs_sprintf(__s, __x, ...)         do_sprintf(__s, __l, PSTR(__x), ##__VA_ARGS__)
#define lrs_printf(__x, ...)               do_printf(PSTR(__x), ##__VA_ARGS__)
#define lrs_puts(__p)                      do_puts(PSTR(__p))

#define lrs_putc(__c)                      SerialWrite(stdOut, __c)
#define lrs_getc()                         SerialRead(stdOut)
#define lrs_inputPending(__s)              SerialAvailable(stdIn)

#ifdef __cplusplus
} // extern "C"
#endif
