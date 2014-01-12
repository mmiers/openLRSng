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
#include "serial.h"

LRS_SerBuffer __LRS__rxBuffer[FS_MAX_PORTS];
LRS_SerBuffer __LRS__txBuffer[FS_MAX_PORTS];

/// Bit mask for initialized ports
static uint8_t _LRS_SerialInitialized;

/// Tell if the serial port has been initialized
static bool LRS_SerialGetInitialized(uint8_t port)
{
  return (1<<port) & _LRS_SerialInitialized;
}

/// Set if the serial port has been initialized
static void LRS_SerialSetInitialized(uint8_t port)
{
  _LRS_SerialInitialized |= (1<<port);
}

// ask for writes to be blocking or non-blocking
void LRS_SerialSetBlockingWrites(LRS_Serial *ser, bool blocking)
{
  ser->_nonblocking_writes = !blocking;
}

//
// Constructor /////////////////////////////////////////////////////////////////
//
void LRS_SerialSetup(LRS_Serial *ser, const uint8_t portNumber,
                     volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                     volatile uint8_t *ucsra, volatile uint8_t *ucsrb, const uint8_t u2x,
                     const uint8_t portEnableBits, const uint8_t portTxBits)
{
  ser->_ubrrh = ubrrh;
  ser->_ubrrl = ubrrl;
  ser->_ucsra = ucsra;
  ser->_ucsrb = ucsrb;
  ser->_u2x   = u2x;
  ser->_portEnableBits = portEnableBits;
  ser->_portTxBits     = portTxBits;
  ser->_rxBuffer = &__LRS__rxBuffer[portNumber];
  ser->_txBuffer = &__LRS__txBuffer[portNumber];

  LRS_SerialSetInitialized(portNumber);
  LRS_SerialBegin(ser, 57600);
}

void LRS_SerialBegin(LRS_Serial *ser, long baud)
{
  LRS_SerialBeginExt(ser, baud, 0, 0);
}

void LRS_SerialBeginExt(LRS_Serial *ser, long baud,
                        unsigned int rxSpace, unsigned int txSpace)
{
  uint16_t ubrr;
  bool use_u2x = true;

  // if we are currently open...
  if (ser->_open) {
    // If the caller wants to preserve the buffer sizing, work out what
    // it currently is...
    if (0 == rxSpace)
      rxSpace = ser->_rxBuffer->mask + 1;
    if (0 == txSpace)
      txSpace = ser->_txBuffer->mask + 1;

    // close the port in its current configuration, clears _open
    LRS_SerialEnd(ser);
  }

  // allocate buffers
  if (!LRS_SerialAllocBuffer(ser->_rxBuffer, rxSpace ? : DEFAULT_RX_BUFFER_SIZE) ||
      !LRS_SerialAllocBuffer(ser->_txBuffer, txSpace ? : DEFAULT_TX_BUFFER_SIZE)) {
    LRS_SerialEnd(ser);
    return; // couldn't allocate buffers - fatal
  }

  // reset buffer pointers
  ser->_txBuffer->head = ser->_txBuffer->tail = 0;
  ser->_rxBuffer->head = ser->_rxBuffer->tail = 0;

  // mark the port as open
  ser->_open = true;

  // If the user has supplied a new baud rate, compute the new UBRR value.
  if (baud > 0) {
#if F_CPU == 16000000UL
    // hardcoded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600)
      use_u2x = false;
#endif

    if (use_u2x) {
      *ser->_ucsra = 1 << ser->_u2x;
      ubrr = (F_CPU / 4 / baud - 1) / 2;
    } else {
      *ser->_ucsra = 0;
      ubrr = (F_CPU / 8 / baud - 1) / 2;
    }

    *ser->_ubrrh = ubrr >> 8;
    *ser->_ubrrl = ubrr;
  }

  *ser->_ucsrb |= ser->_portEnableBits;
}

void LRS_SerialEnd(LRS_Serial *ser)
{
  *ser->_ucsrb &= ~(ser->_portEnableBits | ser->_portTxBits);

  LRS_SerialFreeBuffer(ser->_rxBuffer);
  LRS_SerialFreeBuffer(ser->_txBuffer);
  ser->_open = false;
}

uint16_t LRS_SerialRxOverflowCounter(LRS_Serial *ser)
{
  if (!ser->_open)
    return 0;
  return ser->_rxBuffer->overflow;
}

int LRS_SerialAvailable(LRS_Serial *ser)
{
  if (!ser->_open)
    return (-1);
  return ((ser->_rxBuffer->head - ser->_rxBuffer->tail) & ser->_rxBuffer->mask);
}

int LRS_SerialTxSpace(LRS_Serial *ser)
{
  if (!ser->_open)
    return (-1);
  return ((ser->_txBuffer->mask+1) - ((ser->_txBuffer->head - ser->_txBuffer->tail) & ser->_txBuffer->mask));
}

int LRS_SerialRead(LRS_Serial *ser)
{
  uint8_t c;

  // if the head and tail are equal, the buffer is empty
  if (!ser->_open || (ser->_rxBuffer->head == ser->_rxBuffer->tail))
    return (-1);

  // pull character from tail
  c = ser->_rxBuffer->bytes[ser->_rxBuffer->tail];
  ser->_rxBuffer->tail = (ser->_rxBuffer->tail + 1) & ser->_rxBuffer->mask;

  return (c);
}

int LRS_SerialPeek(LRS_Serial *ser)
{

  // if the head and tail are equal, the buffer is empty
  if (!ser->_open || (ser->_rxBuffer->head == ser->_rxBuffer->tail))
    return (-1);

  // pull character from tail
  return (ser->_rxBuffer->bytes[ser->_rxBuffer->tail]);
}

void LRS_SerialFlush(LRS_Serial *ser)
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of _rxBuffer->head but before writing
  // the value to _rxBuffer->tail; the previous value of head
  // may be written to tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of head but before writing
  // the value to tail; the previous value of rx_buffer_head
  // may be written to tail, making it appear as if the buffer
  // were full, not empty.
  ser->_rxBuffer->head = ser->_rxBuffer->tail;

  // don't reverse this or there may be problems if the TX interrupt
  // occurs after reading the value of _txBuffer->tail but before writing
  // the value to _txBuffer->head.
  ser->_txBuffer->tail = ser->_txBuffer->head;
}

size_t LRS_SerialWrite(LRS_Serial *ser, uint8_t c)
{
  uint16_t i;

  if (!ser->_open) // drop bytes if not open
    return 0;

  // wait for room in the tx buffer
  i = (ser->_txBuffer->head + 1) & ser->_txBuffer->mask;

  // if the port is set into non-blocking mode, then drop the byte
  // if there isn't enough room for it in the transmit buffer
  if (ser->_nonblocking_writes && i == ser->_txBuffer->tail) {
    return 0;
  }

  while (i == ser->_txBuffer->tail)
    ;

  // add byte to the buffer
  ser->_txBuffer->bytes[ser->_txBuffer->head] = c;
  ser->_txBuffer->head = i;

  // enable the data-ready interrupt, as it may be off if the buffer is empty
  *ser->_ucsrb |= ser->_portTxBits;

  // return number of bytes written (always 1)
  return 1;
}

// Buffer management ///////////////////////////////////////////////////////////
bool LRS_SerialAllocBuffer(LRS_SerBuffer *buffer, unsigned int size)
{
  uint16_t  mask;
  uint8_t    shift;

  // init buffer state
  buffer->head = buffer->tail = 0;

  // Compute the power of 2 greater or equal to the requested buffer size
  // and then a mask to simplify wrapping operations.  Using __builtin_clz
  // would seem to make sense, but it uses a 256(!) byte table.
  // Note that we ignore requests for more than BUFFER_MAX space.
  for (shift = 1; (1U << shift) < (MAX_BUFFER_SIZE < size ? MAX_BUFFER_SIZE : size); shift++)
    ;
  mask = (1 << shift) - 1;

  // If the descriptor already has a buffer allocated we need to take
  // care of it.
  if (buffer->bytes) {

    // If the allocated buffer is already the correct size then
    // we have nothing to do
    if (buffer->mask == mask)
      return true;

    // Dispose of the old buffer.
    free(buffer->bytes);
  }
  buffer->mask = mask;

  // allocate memory for the buffer - if this fails, we fail.
  buffer->bytes = (uint8_t *)malloc(buffer->mask + 1);

  return (buffer->bytes != NULL);
}

void LRS_SerialFreeBuffer(LRS_SerBuffer *buffer)
{
  buffer->head = buffer->tail = 0;
  buffer->mask = 0;
  if (NULL != buffer->bytes) {
    free(buffer->bytes);
    buffer->bytes = NULL;
  }
}

