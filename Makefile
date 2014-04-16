##################################################################
#
# Makefile for OpenLRSng
#

#
# If your Arduino is in a weird place, you'll need to change this.
#
ARDUINO_PATH=/usr/share/arduino

#
# Board type can be one of 6 values:
# 0 - Flytron M1 TX
# 1 - Flytron M1 RX
# 2 - Flytron M2, M3 TX, HK Orange-TX
# 3 - Flytron V2 RX, Hawkeye RX, HK Orange-RX
# 4 - Hawkeye TX, OpenLRSng TX
# 5 - DTF 4ch RX
# 6 - Deluxe TX
#
BOARD_TYPE=3

#
# You can compile all TX as TX, and all RX as either RX or TX.
# You cannot currently compile TX as RX.
# This flag controls what primary function the board will have
#
COMPILE_TX=1

#
# No real user options below here.
##################################################################

#
# You don't want to change this unless you really know that you
# need to.  CPU clock.
#
CLOCK=16000000L

#
# Board type 6 requires a different Arduino target
#
ifeq ($(BOARD_TYPE),6)
CPU=atmega32u4
USB_VID=0x2341
USB_PID=0x8036
VARIANT=leonardo
else
CPU=atmega328p
USB_VID=null
USB_PID=null
VARIANT=standard
endif

#
# C preprocessor defines
#
DEFINES=-DBOARD_TYPE=$(BOARD_TYPE)
ifeq ($(COMPILE_TX),1)
DEFINES:=$(DEFINES) -DCOMPILE_TX
endif

#
# AVR GCC info
#
EXEPREFIX=avr-
ifneq (,$(wildcard $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-gcc))
	EXEPATH=$(ARDUINO_PATH)/hardware/tools/avr/bin
else ifneq (,$(wildcard /usr/bin/avr-gcc))
	EXEPATH=/usr/bin
endif

#
# AVR gcc and binutils
#
CC=$(EXEPATH)/$(EXEPREFIX)gcc
CXX=$(EXEPATH)/$(EXEPREFIX)g++
AR=$(EXEPATH)/$(EXEPREFIX)ar
SIZE=$(EXEPATH)/$(EXEPREFIX)size
OBJCOPY=$(EXEPATH)/$(EXEPREFIX)objcopy

RM=rm

#
# Styling
#
ASTYLE=astyle
ASTYLEOPTIONS=--style=1tbs --indent=spaces=2 --suffix=none

#
# Compile flags
#
COPTFLAGS= -g -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
	   -fno-inline-small-functions -Wl,--relax -mcall-prologues

CFLAGS=-Wall -ffunction-sections -fdata-sections -mmcu=$(CPU) -DF_CPU=$(CLOCK) -MMD \
	-DUSB_VID=$(USB_VID) -DUSB_PID=$(USB_PID) -DARDUINO=105 -D__PROG_TYPES_COMPAT__ $(DEFINES)
CXXFLAGS=-fno-exceptions

#
# Master include path
#
INCLUDE=-I.

#
# Target object files
#
OBJS=openLRSng.o printf.o serial.o usbcore.o

#
# Master target
#
all: openLRSng.hex

#
# From here down are build rules
#
define ino-command
	@$(CXX) -c $(COPTFLAGS) $(CXXFLAGS) $(CFLAGS) $(INCLUDE) -o $@ -x c++ $<
endef
define cc-command
	@$(CC) --std=c99 -c $(COPTFLAGS) $(CFLAGS) $(INCLUDE) -o $@ $<
endef
define cxx-command
	@$(CXX) -c $(COPTFLAGS) $(CXXFLAGS) $(CFLAGS) $(INCLUDE) -o $@ $<
endef

.PHONY: all clean upload astyle 433 868 915 allfw

%.o: %.ino
	$(ino-command)

%.o: %.c
	$(cc-command)

%.o: %.cpp
	$(cxx-command)

#
# Other targets
#
clean:
	rm -f *.[aod] *.elf *.eep *.d *.hex

openLRSng.hex: $(OBJS)
	@$(CC) -Os -Wl,--gc-sections -mmcu=$(CPU) -o openLRSng.elf $(OBJS) -Llibraries -lm
	@$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load \
		--no-change-warnings --change-section-lma .eeprom=0 \
		openLRSng.elf openLRSng.eep
	@$(OBJCOPY) -O ihex -R .eeprom openLRSng.elf openLRSng.hex
	@echo "NOTE: Deployment size is text + data."
	@$(SIZE) openLRSng.elf

astyle:
	$(ASTYLE) $(ASTYLEOPTIONS) openLRSng.ino *.h

433:
	mkdir -p out
	rm -f out/*.hex
	make -s COMPILE_TX= BOARD_TYPE=3 clean all && cp openLRSng.hex out/RX-3.hex
	make -s COMPILE_TX= BOARD_TYPE=5 clean all && cp openLRSng.hex out/RX-5.hex
	make -s COMPILE_TX=1 BOARD_TYPE=2 clean all && cp openLRSng.hex out/TX-2.hex
	make -s COMPILE_TX=1 BOARD_TYPE=3 clean all && cp openLRSng.hex out/TX-3.hex
	make -s COMPILE_TX=1 BOARD_TYPE=4 clean all && cp openLRSng.hex out/TX-4.hex
	make -s COMPILE_TX=1 BOARD_TYPE=5 clean all && cp openLRSng.hex out/TX-5.hex
	make -s COMPILE_TX=1 BOARD_TYPE=6 clean all && cp openLRSng.hex out/TX-6.hex
	ls -l out

868:
	mkdir -p out/868
	rm -f out/868/*.hex
	make -s RFMXX_868=1 COMPILE_TX= BOARD_TYPE=3 clean all && cp openLRSng.hex out/868/RX-3.hex
	make -s RFMXX_868=1 COMPILE_TX= BOARD_TYPE=5 clean all && cp openLRSng.hex out/868/RX-5.hex
	make -s RFMXX_868=1 COMPILE_TX=1 BOARD_TYPE=2 clean all && cp openLRSng.hex out/868/TX-2.hex
	make -s RFMXX_868=1 COMPILE_TX=1 BOARD_TYPE=3 clean all && cp openLRSng.hex out/868/TX-3.hex
	make -s RFMXX_868=1 COMPILE_TX=1 BOARD_TYPE=4 clean all && cp openLRSng.hex out/868/TX-4.hex
	make -s RFMXX_868=1 COMPILE_TX=1 BOARD_TYPE=5 clean all && cp openLRSng.hex out/868/TX-5.hex
	make -s RFMXX_868=1 COMPILE_TX=1 BOARD_TYPE=6 clean all && cp openLRSng.hex out/868/TX-6.hex
	ls -l out/868

915:
	mkdir -p out/915
	rm -f out/915/*.hex
	make -s RFMXX_915=1 COMPILE_TX= BOARD_TYPE=3 clean all && cp openLRSng.hex out/915/RX-3.hex
	make -s RFMXX_915=1 COMPILE_TX= BOARD_TYPE=5 clean all && cp openLRSng.hex out/915/RX-5.hex
	make -s RFMXX_915=1 COMPILE_TX=1 BOARD_TYPE=2 clean all && cp openLRSng.hex out/915/TX-2.hex
	make -s RFMXX_915=1 COMPILE_TX=1 BOARD_TYPE=3 clean all && cp openLRSng.hex out/915/TX-3.hex
	make -s RFMXX_915=1 COMPILE_TX=1 BOARD_TYPE=4 clean all && cp openLRSng.hex out/915/TX-4.hex
	make -s RFMXX_915=1 COMPILE_TX=1 BOARD_TYPE=5 clean all && cp openLRSng.hex out/915/TX-5.hex
	make -s RFMXX_915=1 COMPILE_TX=1 BOARD_TYPE=6 clean all && cp openLRSng.hex out/915/TX-6.hex
	ls -l out/868

allfw:  433 868 915
	ls -lR out

