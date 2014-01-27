/*
 * Serial iplementation for openLRSng
 *
 * Based on FastSerial from APM, with the following license:
 *
 * Interrupt-driven serial transmit/receive library.
 *
 *      Copyright (c) 2010 Michael Smith. All rights reserved.
 *
 * Receive and baudrate calculations derived from the Arduino
 * HardwareSerial driver:
 *
 *      Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * Transmit algorithm inspired by work:
 *
 *      Code Jose Julio and Jordi Munoz. DIYDrones.com
 *
 *      This library is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU Lesser General Public
 *      License as published by the Free Software Foundation; either
 *      version 2.1 of the License, or (at your option) any later
 *      version.
 *
 *      This library is distributed in the hope that it will be
 *      useful, but WITHOUT ANY WARRANTY; without even the implied
 *      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *      PURPOSE.  See the GNU Lesser General Public License for more
 *      details.
 *
 *      You should have received a copy of the GNU Lesser General
 *      Public License along with this library; if not, write to the
 *      Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 *      Boston, MA 02110-1301 USA
 */
#if !defined(SERIAL_H)
#define SERIAL_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#if !MALLOC_RING
#define RING_SIZE    64 ///< Same size as Arduino
#endif

///
/// We only want to define interrupt handlers for serial ports that
/// are actually used, so we force our users to define them using
/// a macro.
///
/// SerialPort(<port name>, <port number>)
///
/// <port name> is the name of the object that will be created by the
/// macro.  <port number> is the 0-based number of the port that will
/// be managed by the object.
///

///
/// Transmit/receive buffer descriptor.
///
typedef struct
{
  volatile uint16_t head, tail; ///< head and tail pointers
  volatile uint16_t overflow;   ///< Incremented every time the buffer can't fit a character.
  uint16_t          mask;       ///< buffer size mask for pointer wrap
#if MALLOC_RING
  uint8_t           *bytes;     ///< pointer to allocated buffer
#else
  uint8_t           bytes[RING_SIZE]; ///< buffer
#endif
}
SerialBuffer;

typedef enum
{
  SERIAL_TTY,
  SERIAL_USB
}
SerialType;

///
/// The SerialPort data structure definition
///
typedef struct
{
  SerialType _type;

  // register accessors
  volatile uint8_t * _ubrrh;
  volatile uint8_t * _ubrrl;
  volatile uint8_t * _ucsra;
  volatile uint8_t * _ucsrb;

  // register magic numbers
  uint8_t            _u2x;
  uint8_t            _portEnableBits;  ///< rx, tx and rx interrupt enables
  uint8_t            _portTxBits;      ///< tx data and completion interrupt enables

  // ring buffers
  SerialBuffer * _rxBuffer;
  SerialBuffer * _txBuffer;
  bool            _open;

  // whether writes to the port should block waiting for enough space to appear
  bool            _nonblocking_writes;
}
SerialPort;

/// Allocates a buffer of the given size
///
/// @param      buffer    The buffer descriptor for which the buffer will
///                       will be allocated.
/// @param      size      The desired buffer size.
/// @returns              True if the buffer was allocated successfully.
///
bool SerialAllocBuffer(SerialBuffer *buffer, unsigned int size);

/// Frees the allocated buffer in a descriptor
///
/// @param      buffer    The descriptor whose buffer should be freed.
///
void SerialFreeBuffer(SerialBuffer *buffer);

/// default receive buffer size
#define DEFAULT_RX_BUFFER_SIZE 128

/// default transmit buffer size
#define DEFAULT_TX_BUFFER_SIZE 16

/// maxium tx/rx buffer size
/// @note if we could bring the max size down to 256, the mask and head/tail
///       pointers in the buffer could become uint8_t.
///
#if defined MALLOC_RING
#define MAX_BUFFER_SIZE 512
#else
#define MAX_BUFFER_SIZE 128
#endif

//
// LRS serial API
//
void     SerialSetup(SerialPort *ser, const uint8_t portNumber,
                     SerialType type,
                     volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                     volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                     const uint8_t u2x, const uint8_t portEnableBits,
                     const uint8_t portTxBits);

/// Serial API
void     SerialBegin(SerialPort *, long baud);
void     SerialBeginExt(SerialPort *, long baud,
                        unsigned int rxSpace, unsigned int txSpace);
void     SerialEnd(SerialPort *);
int      SerialAvailable(SerialPort *);
int      SerialRxSpace(SerialPort *);
int      SerialTxSpace(SerialPort *);
int      SerialBufferSize(SerialPort *);
int      SerialRead(SerialPort *);
int      SerialPeek(SerialPort *);
void     SerialFlush(SerialPort *);
size_t   SerialWrite(SerialPort *, uint8_t c);

uint16_t SerialRxOverflowCounter(SerialPort *);
bool     SerialGetInitialized(uint8_t port);
void     SerialSetBlockingWrites(SerialPort *, bool);

//
// Forward decl for the I/O buffers
//
extern SerialBuffer __Serial__rxBuffer[];
extern SerialBuffer __Serial__txBuffer[];

///
/// Generic Rx/Tx vectors for a serial port - needs to know magic numbers
///
#define SerialHandler(_PORT, _RXVECTOR, _TXVECTOR, _UDR, _UCSRB, _TXBITS)    \
ISR(_RXVECTOR, ISR_BLOCK)                                                    \
{                                                                            \
  uint8_t c;                                                                 \
  uint16_t i;                                                                \
                                                                             \
  /* read the byte as quickly as possible */                                 \
  c = _UDR;                                                                  \
  /* work out where the head will go next */                                 \
  i = (__Serial__rxBuffer[_PORT].head + 1) & __Serial__rxBuffer[_PORT].mask; \
  /* decide whether we have space for another byte */                        \
  if (i != __Serial__rxBuffer[_PORT].tail) {                                 \
    /* we do, move the head */                                               \
    __Serial__rxBuffer[_PORT].bytes[__Serial__rxBuffer[_PORT].head] = c;     \
    __Serial__rxBuffer[_PORT].head = i;                                      \
  }                                                                          \
  else                                                                       \
  {                                                                          \
     __Serial__rxBuffer[_PORT].overflow++;                                   \
  }                                                                          \
}                                                                            \
ISR(_TXVECTOR, ISR_BLOCK)                                                    \
{                                                                            \
  /* if there is another character to send */                                \
  if (__Serial__txBuffer[_PORT].tail != __Serial__txBuffer[_PORT].head) {    \
    _UDR = __Serial__txBuffer[_PORT].bytes[__Serial__txBuffer[_PORT].tail];  \
    /* increment the tail */                                                 \
    __Serial__txBuffer[_PORT].tail =                                         \
      (__Serial__txBuffer[_PORT].tail + 1) & __Serial__txBuffer[_PORT].mask; \
  } else {                                                                   \
    /* there are no more bytes to send, disable the interrupt */             \
    if (__Serial__txBuffer[_PORT].head == __Serial__txBuffer[_PORT].tail)    \
      _UCSRB &= ~_TXBITS;                                                    \
  }                                                                          \
}                                                                            \
struct hack

//
// Portability; convert various older sets of defines for U(S)ART0 up
// to match the definitions for the 1280 and later devices.
//
#if !defined(USART0_RX_vect)
# if defined(USART_RX_vect)
#  define USART0_RX_vect        USART_RX_vect
#  define USART0_UDRE_vect      USART_UDRE_vect
# elif defined(UART0_RX_vect)
#  define USART0_RX_vect        UART0_RX_vect
#  define USART0_UDRE_vect      UART0_UDRE_vect
# endif
#endif

#if !defined(USART1_RX_vect)
# if defined(UART1_RX_vect)
#  define USART1_RX_vect        UART1_RX_vect
#  define USART1_UDRE_vect      UART1_UDRE_vect
# endif
#endif

#if !defined(UDR0)
# if defined(UDR)
#  define UDR0                  UDR
#  define UBRR0H                UBRRH
#  define UBRR0L                UBRRL
#  define UCSR0A                UCSRA
#  define UCSR0B                UCSRB
#  define U2X0                  U2X
#  define RXEN0                 RXEN
#  define TXEN0                 TXEN
#  define RXCIE0                RXCIE
#  define UDRIE0                UDRIE
# endif
#endif

///
/// Macro defining a FastSerial port instance.
///
#define SerialPort(_name, _num)                                  \
  SerialPort _name;                                              \
  SerialHandler(_num,                                            \
                   USART##_num##_RX_vect,                        \
                   USART##_num##_UDRE_vect,                      \
                   UDR##_num,                                    \
                   UCSR##_num##B,                                \
                   _BV(UDRIE##_num))
#define SerialConstruct(_name, _num)                             \
  SerialSetup(_name,                                             \
              _num, SERIAL_TTY,                                  \
              &UBRR##_num##H,                                    \
              &UBRR##_num##L,                                    \
              &UCSR##_num##A,                                    \
              &UCSR##_num##B,                                    \
              U2X##_num,                                         \
              (_BV(RXEN##_num) | _BV(TXEN##_num) | _BV(RXCIE##_num)), \
              (_BV(UDRIE##_num)));                               \

#if BOARD_TYPE == 6
# define USB_MAX_PORTS  1
#else
# define USB_MAX_PORTS  0
#endif

#if   defined(UDR3)
# define FS_MAX_PORTS   (4 + USB_MAX_PORTS)
#elif defined(UDR2)
# define FS_MAX_PORTS   (3 + USB_MAX_PORTS)
#elif defined(UDR1)
# define FS_MAX_PORTS   (2 + USB_MAX_PORTS)
#else
# define FS_MAX_PORTS   (1 + USB_MAX_PORTS)
#endif

#define USBSerialPort(_name)                                     \
  SerialPort _name;
#define USBSerialConstruct(_name)                                \
  SerialSetup(_name, FS_MAX_PORTS - 1, SERIAL_USB,               \
                  0, 0, 0, 0, 0, 0, 0);                          \

#if defined(__cplusplus)
}
#endif

#endif

