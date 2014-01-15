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

SerialBuffer __Serial__rxBuffer[FS_MAX_PORTS];
SerialBuffer __Serial__txBuffer[FS_MAX_PORTS];

/// Bit mask for initialized ports
static uint8_t _SerialInitialized;

/// Set if the serial port has been initialized
static void SerialSetInitialized(uint8_t port)
{
  _SerialInitialized |= (1<<port);
}

/// Tell if the serial port has been initialized
bool SerialGetInitialized(uint8_t port)
{
  return (1<<port) & _SerialInitialized;
}

// ask for writes to be blocking or non-blocking
void SerialSetBlockingWrites(SerialPort *ser, bool blocking)
{
  ser->_nonblocking_writes = !blocking;
}

#if BOARD_TYPE == 6
/*
** Copyright (c) 2011, Peter Barrett  
**  
** Permission to use, copy, modify, and/or distribute this software for  
** any purpose with or without fee is hereby granted, provided that the  
** above copyright notice and this permission notice appear in all copies.  
** 
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR  
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES  
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,  
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS  
** SOFTWARE.  
*/
#include "usbapi.h"
#include "usbcore.h"
#include "usbdesc.h"

#include <avr/wdt.h>
#include <avr/pgmspace.h>

typedef struct
{
    uint32_t  dwDTERate;
    uint8_t   bCharFormat;
    uint8_t   bParityType;
    uint8_t   bDataBits;
    uint8_t   lineState;
}
LineInfo;

static volatile LineInfo _usbLineInfo = { 57600, 0x00, 0x00, 0x00, 0x00 };

#define WEAK __attribute__ ((weak))

extern const CDCDescriptor _cdcInterface PROGMEM;
const CDCDescriptor _cdcInterface =
{
    D_IAD(0,2,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,1),

    //    CDC communication interface
    D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
    D_CDCCS(CDC_HEADER,0x10,0x01),                                // Header (1.10 bcd)
    D_CDCCS(CDC_CALL_MANAGEMENT,1,1),                            // Device handles call management (not)
    D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6),                // SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
    D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE),    // Communication interface is master, data interface is slave 0
    D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),USB_ENDPOINT_TYPE_INTERRUPT,0x10,0x40),

    //    CDC data interface
    D_INTERFACE(CDC_DATA_INTERFACE,2,CDC_DATA_INTERFACE_CLASS,0,0),
    D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),USB_ENDPOINT_TYPE_BULK,0x40,0),
    D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN ),USB_ENDPOINT_TYPE_BULK,0x40,0)
};

int WEAK CDC_GetInterface(uint8_t* interfaceNum)
{
    interfaceNum[0] += 2;    // uses 2
    return USB_SendControl(TRANSFER_PGM,&_cdcInterface,sizeof(_cdcInterface));
}

bool WEAK CDC_Setup(Setup* setup)
{
    uint8_t r = setup->bRequest;
    uint8_t requestType = setup->bmRequestType;

    if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
    {
        if (CDC_GET_LINE_CODING == r)
        {
            USB_SendControl(0,(void*)&_usbLineInfo,7);
            return true;
        }
    }

    if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
    {
        if (CDC_SET_LINE_CODING == r)
        {
            USB_RecvControl((void*)&_usbLineInfo,7);
            return true;
        }

        if (CDC_SET_CONTROL_LINE_STATE == r)
        {
            _usbLineInfo.lineState = setup->wValueL;

            // auto-reset into the bootloader is triggered when the port, already 
            // open at 1200 bps, is closed.  this is the signal to start the watchdog
            // with a relatively long period so it can finish housekeeping tasks
            // like servicing endpoints before the sketch ends
            if (1200 == _usbLineInfo.dwDTERate) {
                // We check DTR state to determine if host port is open (bit 0 of lineState).
                if ((_usbLineInfo.lineState & 0x01) == 0) {
                    *(uint16_t *)0x0800 = 0x7777;
                    wdt_enable(WDTO_120MS);
                } else {
                    // Most OSs do some intermediate steps when configuring ports and DTR can
                    // twiggle more than once before stabilizing.
                    // To avoid spurious resets we set the watchdog to 250ms and eventually
                    // cancel if DTR goes back high.
    
                    wdt_disable();
                    wdt_reset();
                    *(uint16_t *)0x0800 = 0x0;
                }
            }
            return true;
        }
    }
    return false;
}


int _serialPeek = -1;

void USBSerial_Accept(void) 
{
    // NOTE!  We're only good for ONE USB serial port.
    SerialBuffer *buffer = &__Serial__rxBuffer[FS_MAX_PORTS-1];

    int i = (unsigned int)(buffer->head+1) & buffer->mask;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.

    // while we have room to store a byte
    while (i != buffer->tail) {
        int c = USB_Get(CDC_RX);
        if (c == -1)
            break;    // no more data
        buffer->bytes[buffer->head] = c;
        buffer->head = i;
        i = (unsigned int)(buffer->head+1) & buffer->mask;
    }
}

void USBSerial_Flush(void)
{
    USB_Flush(CDC_TX);
}

size_t USBSerial_Write(uint8_t c)
{
    /* only try to send bytes if the high-level CDC connection itself 
     is open (not just the pipe) - the OS should set lineState when the port
     is opened and clear lineState when the port is closed.
     bytes sent before the user opens the connection or after
     the connection is closed are lost - just like with a UART. */
    
    // TODO - ZE - check behavior on different OSes and test what happens if an
    // open connection isn't broken cleanly (cable is yanked out, host dies
    // or locks up, or host virtual serial port hangs)
    if (_usbLineInfo.lineState > 0)    {
        int r = USB_Send(CDC_TX,&c,1);
        if (r > 0) {
            return r;
        } else {
            return 0;
        }
    }
    return 0;
}

#endif

//
// Constructor /////////////////////////////////////////////////////////////////
//
void SerialSetup(SerialPort *ser, const uint8_t portNumber,
                 SerialType type,
                 volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                 volatile uint8_t *ucsra, volatile uint8_t *ucsrb, const uint8_t u2x,
                 const uint8_t portEnableBits, const uint8_t portTxBits)
{
  ser->_type = type;
  ser->_ubrrh = ubrrh;
  ser->_ubrrl = ubrrl;
  ser->_ucsra = ucsra;
  ser->_ucsrb = ucsrb;
  ser->_u2x   = u2x;
  ser->_portEnableBits = portEnableBits;
  ser->_portTxBits     = portTxBits;
  ser->_rxBuffer = &__Serial__rxBuffer[portNumber];
  ser->_txBuffer = &__Serial__txBuffer[portNumber];

  SerialSetInitialized(portNumber);
  SerialBegin(ser, 57600);
}

void SerialBegin(SerialPort *ser, long baud)
{
  SerialBeginExt(ser, baud, 0, 0);
}

void SerialBeginExt(SerialPort *ser, long baud,
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
    SerialEnd(ser);
  }

  // allocate buffers
  if (!SerialAllocBuffer(ser->_rxBuffer, rxSpace ? : DEFAULT_RX_BUFFER_SIZE) ||
      !SerialAllocBuffer(ser->_txBuffer, txSpace ? : DEFAULT_TX_BUFFER_SIZE)) {
    SerialEnd(ser);
    return; // couldn't allocate buffers - fatal
  }

  // reset buffer pointers
  ser->_txBuffer->head = ser->_txBuffer->tail = 0;
  ser->_rxBuffer->head = ser->_rxBuffer->tail = 0;

  // mark the port as open
  ser->_open = true;

#if BOARD_TYPE == 6
  // Short circuit for USB
  if (ser->_type == SERIAL_USB)
    return;
#endif

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

void SerialEnd(SerialPort *ser)
{
  *ser->_ucsrb &= ~(ser->_portEnableBits | ser->_portTxBits);

  SerialFreeBuffer(ser->_rxBuffer);
  SerialFreeBuffer(ser->_txBuffer);
  ser->_open = false;
}

uint16_t SerialRxOverflowCounter(SerialPort *ser)
{
  if (!ser->_open)
    return 0;
  return ser->_rxBuffer->overflow;
}

int SerialAvailable(SerialPort *ser)
{
  if (!ser->_open)
    return (-1);
  return ((ser->_rxBuffer->head - ser->_rxBuffer->tail) & ser->_rxBuffer->mask);
}

int SerialTxSpace(SerialPort *ser)
{
  if (!ser->_open)
    return (-1);
  return ((ser->_txBuffer->mask+1) - ((ser->_txBuffer->head - ser->_txBuffer->tail) & ser->_txBuffer->mask));
}

int SerialRead(SerialPort *ser)
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

int SerialPeek(SerialPort *ser)
{

  // if the head and tail are equal, the buffer is empty
  if (!ser->_open || (ser->_rxBuffer->head == ser->_rxBuffer->tail))
    return (-1);

  // pull character from tail
  return (ser->_rxBuffer->bytes[ser->_rxBuffer->tail]);
}

void SerialFlush(SerialPort *ser)
{
#if BOARD_TYPE == 6
  // Short circuit on USB
  if (ser->_type == SERIAL_USB) {
    USBSerial_Flush();
    return;
  }
#endif

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

size_t SerialWrite(SerialPort *ser, uint8_t c)
{
  uint16_t i;

  if (!ser->_open) // drop bytes if not open
    return 0;

#if BOARD_TYPE == 6
  if (ser->_type == SERIAL_USB) {
    USBSerial_Write(c);
    return 0;
  }
#endif

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
bool SerialAllocBuffer(SerialBuffer *buffer, unsigned int size)
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

void SerialFreeBuffer(SerialBuffer *buffer)
{
  buffer->head = buffer->tail = 0;
  buffer->mask = 0;
  if (NULL != buffer->bytes) {
    free(buffer->bytes);
    buffer->bytes = NULL;
  }
}
