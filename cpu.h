//
// CPU definitions
// Lifted from Arduino: pins_arduino.h (standard, leonardo)
//
//  Copyright (c) 2007 David A. Mellis
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General
//  Public License along with this library; if not, write to the
//  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
//  Boston, MA  02111-1307  USA
//

// Atmel pins
#if __AVR_ATmega328P__ == 1

#define SS           10
#define MOSI         11
#define MISO         12
#define SCK          13

#define SDA          18
#define SCL          19
#define LED_BUILTIN  13

#define A0           14
#define A1           15
#define A2           16
#define A3           17
#define A4           18
#define A5           19
#define A6           20
#define A7           21

#elif __AVR_ATmega32U4__ == 1

#define SDA          2
#define SCL          3

// Map SPI port to 'new' pins D14..D17
#define SS           17
#define MOSI         16
#define MISO         14
#define SCK          15

// Mapping of analog pins as digital I/O
// A6-A11 share with digital pins
#define A0          18
#define A1          19
#define A2          20
#define A3          21
#define A4          22
#define A5          23
#define A6          24   // D4
#define A7          25   // D6
#define A8          26   // D8
#define A9          27   // D9
#define A10         28   // D10
#define A11         29   // D12

#else

#error "You don't have a proper MCU defined.  Build flags are bad."

#endif // CPU type

