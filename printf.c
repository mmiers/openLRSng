/*
 * notify.c
 *
 * printf() style routine for openLRSng
 */
#include "printf.h"

#define PRINTF_UC   0x01 // upper case on hex output
#define PRINTF_ZS   0x02 // output zeroes (after initial digit)
#define PRINTF_HEX  0x04 // output hex
#define PRINTF_LONG 0x08 // 32bit number
#define PRINTF_LZ   0x10 // leading zeroes
#define PRINTF_HALF 0x20 // half word (byte)

static char* bf;
static char buf[12];
static uint32_t num;
static uint8_t  flags;

extern void notify_putc(char);

static size_t (*putch)(SerialPort *, uint8_t) = SerialWrite;

static void
out(char c)
{
  *bf++ = c;
}

static void
outDgt(char dgt)
{
  out(dgt + (dgt < 10 ? '0' : ((flags & PRINTF_UC) ? 'A' : 'a')-10));
  flags |= PRINTF_ZS;
}

static void
divOut(uint32_t div)
{
  unsigned char dgt=0;
  while (num >= div) {
    num -= div;
    dgt++;
  }
  if ((flags & PRINTF_ZS) || dgt > 0)
    outDgt(dgt);
}

void
do_printf(SerialPort *ser, const char *fmt, ...)
{
  va_list va;
  char ch;
  char* p;

  va_start(va,fmt);

  while ((ch = pgm_read_byte(fmt++))) {
    if (ch != '%') {
      putch(ser, ch);
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
      bf = buf;
      p = bf;
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
          out('-');
        }
        if (flags & PRINTF_HEX) {
          if (flags & PRINTF_LONG) {
            divOut(0x10000000);
            divOut(0x1000000);
            divOut(0x100000);
            divOut(0x10000);
            divOut(0x1000);
            divOut(0x100);
          } else if (!(flags & PRINTF_HALF)) {
            divOut(0x1000);
            divOut(0x100);
          }
          divOut(0x10);
        } else {
          if (flags & PRINTF_LONG) {
            divOut(1000000000);
            divOut(100000000);
            divOut(10000000);
            divOut(1000000);
            divOut(100000);
            divOut(10000);
            divOut(1000);
          } else if (!(flags & PRINTF_HALF)) {
            divOut(10000);
            divOut(1000);
          }
          divOut(100);
          divOut(10);
        }
        outDgt(num);
        break;
      case 'c' :
        out((char)(va_arg(va, int)));
        break;
      case 's' :
        p = va_arg(va, char*);
        break;
      case '%' :
        out('%');
      default:
        break;
      }
      *bf = 0;
      bf = p;
      while (*bf++ && w > 0)
        w--;
      while (w-- > 0)
        putch(ser, (flags&PRINTF_LZ) ? '0' : ' ');
      while ((ch= *p++))
        putch(ser, ch);
    }
  }
error:
  va_end(va);
}

void
do_puts(SerialPort *ser, const char *str)
{
    do_printf(ser, str);
    putch(ser, '\r');
    putch(ser, '\n');
}

