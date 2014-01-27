/*
 * notify.c
 *
 * printf() style routine for openLRSng
 */
#include <string.h>

#include "printf.h"

#define PRINTF_UC   0x01 // upper case on hex output
#define PRINTF_ZS   0x02 // output zeroes (after initial digit)
#define PRINTF_HEX  0x04 // output hex
#define PRINTF_LONG 0x08 // 32bit number
#define PRINTF_LZ   0x10 // leading zeroes
#define PRINTF_HALF 0x20 // half word (byte)

static size_t
print_string(size_t (*pr_putc)(void *data, char ch), void *data,
             uint8_t flags, uint8_t w, char *p)
{
  size_t ret = 1;

  while (w-- > 0) {
    if (!(*pr_putc)(data, (flags&PRINTF_LZ) ? '0' : ' '))
      return 0;
  }

  char ch;
  while ((ch= *p++)) {
    if (!(*pr_putc)(data, ch))
      return 0;
  }

  return ret;
}

static size_t
print_number(size_t (*pr_putc)(void *data, char ch), void *data,
             uint8_t flags, int8_t w, int32_t n, uint8_t base)
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  w -= strlen(str);
  return print_string(pr_putc, data, flags, w, str);
}

void
format_output(size_t (*pr_putc)(void *data, char ch),
              void *data, const char *fmt, va_list va)
{
  uint32_t num;
  uint8_t flags;
  char ch, *p;

  while ((ch = pgm_read_byte(fmt++))) {
    flags = 0;
    if (ch != '%') {
      if (!(*pr_putc)(data, ch))
        return;
    }
    else {
      flags = 0;
      char w = 0;
      ch = pgm_read_byte(fmt++);
      if (ch == '0') {
        ch = pgm_read_byte(fmt++);
        flags |= PRINTF_LZ;
      }
      if (ch >= '0' && ch <= '9') {
        w = 0;
        while (ch >= '0' && ch <= '9') {
          w = (((w << 2) + w) << 1) + ch - '0';
          ch = pgm_read_byte(fmt++);
        }
      }
      if (ch == 'l') {
        ch = pgm_read_byte(fmt++);
        flags |= PRINTF_LONG;
      } else if (ch == 'h') {
        ch = pgm_read_byte(fmt++);
        flags |= PRINTF_HALF;
      }
      switch (ch) {
      case 0:
        goto error;
      case 'X' :
        flags |= PRINTF_UC;
      case 'x':
        flags |= PRINTF_HEX;
      case 'u':
      case 'd' :
        if (flags & PRINTF_LONG) {
          num = va_arg(va, uint32_t);
        } else {
          num = va_arg(va, uint16_t);
        }
        if (ch == 'd' && (int32_t)num < 0) {
          num = -(int32_t)num;
          if (!(*pr_putc)(data, '-'))
            goto error;
        }
        if (!print_number(pr_putc, data, flags, w, num, flags & PRINTF_HEX ? 16 : 10))
          goto error;
        break;
      case 'c' :
        if (!(*pr_putc)(data, (char)(va_arg(va, int))))
          return;
        break;
      case 's' :
        p = va_arg(va, char *);
        if (!print_string(pr_putc, data, flags,
                          w == 0 ? 0 : w - strlen(p), p))
          goto error;
        break;
      case '%' :
        if (!(*pr_putc)(data, '%'))
          goto error;
      default:
        break;
      }
    }
  }
error:
  va_end(va);
}

/*
 * Helper for buffer output
 */
static size_t maxBuf = 0;
static size_t index = 0;
static size_t
bufferPut(void *data, char ch)
{
  if (index >= (maxBuf - 1))
    return 0;
  ((char *)data)[index] = ch;
  index++;
  return 1;
}

/* BUFFER FAMILY */
void
do_vsnprintf(char *buf, size_t size, const char *fmt, va_list va)
{
  /* Have to keep room for the trailing NUL */
  if (size == 1) {
    buf[0] = 0;
    return;
  }
  maxBuf = size;
  index = 0;
  format_output(bufferPut, buf, fmt, va);
  buf[index] = 0;
}

void
do_snprintf(char *buf, size_t size, const char *fmt, ...)
{
  va_list va;

  /* Have to keep room for the trailing NUL */
  if (size == 1) {
    buf[0] = 0;
    return;
  }
  va_start(va, fmt);
  maxBuf = size;
  index = 0;
  format_output(bufferPut, buf, fmt, va);
  buf[index] = 0;
  va_end(va);
}

void
do_vsprintf(char *buf, const char *fmt, va_list va)
{
  maxBuf = (size_t)-1;
  index = 0;
  format_output(bufferPut, buf, fmt, va);
  buf[index] = 0;
}

void
do_sprintf(char *buf, const char *fmt, ...)
{
  va_list va;

  maxBuf = (size_t)-1;
  index = 0;
  va_start(va, fmt);
  format_output(bufferPut, buf, fmt, va);
  buf[index] = 0;
  va_end(va);
}

void
do_vprintf(SerialPort *ser, const char *fmt, va_list va)
{
  format_output((size_t (*)(void *, char))SerialWrite, ser, fmt, va);
}

void
do_printf(SerialPort *ser, const char *fmt, ...)
{
  va_list va;

  va_start(va,fmt);
  format_output((size_t (*)(void *, char))SerialWrite, ser, fmt, va);
  va_end(va);
}

void
do_puts(SerialPort *ser, const char *str)
{
    do_printf(ser, str);
    SerialWrite(ser, '\r');
    SerialWrite(ser, '\n');
}

