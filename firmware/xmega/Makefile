###############################################################################
# Makefile 
###############################################################################

## General Flags
PROJECT = flightcontrol
MCU = atxmega128a1
TARGET = $(PROJECT).elf
CC = avr-gcc

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON) -D__AVR_ATxmega128a1__
CFLAGS += -D AVRGCC -Wall -gdwarf-2  -Os -fsigned-char
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-Map=$(PROJECT).map,--cref

LIBS = -lm

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0


## Include Directories
INCLUDES = -I /usr/lib/avr/include/

SOURCES = test.c usart_driver.c uart.c adc.c
## Objects that must be built in order to link
OBJECTS = $(SOURCES:.c=.o)

## Objects explicitly added by the user
LINKONLYOBJECTS =

## Build
all: $(TARGET) $(PROJECT).hex $(PROJECT).eep
## Compile
.c.o:
	$(CC) $(INCLUDES) $(CFLAGS) -c  $< -o $@

#.c.o: 
#	$(CC) $(CFLAGS) -c $< -o $@
	
##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@  || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

## Program
program: all
	#avrdude -c osuisp2 -p t861 -e -U flash:w:$(PROJECT).hex	-B 4
	#avrdude -P /dev/ttyUSB0 -p x128a3 -c avr109 -b 9600 -e -U flash:w:$(PROJECT).hex -B 4 	
	avrdude -c avrispmkII -P usb -p x128a1 -e -U flash:w:$(PROJECT).hex	-B 4
	#avrdude -p x128a3 -c avr911 -P /dev/ttyUSB1 -b 57600 -e -U flash:w:$(PROJECT).hex
	#avrdude -p x128a3 -c buspirate -P /dev/ttyUSB1 -e -U flash:w:$(PROJECT).hex	-F

## Clean target
.PHONY: clean
clean:
	-rm -rf $(PROJECT).hex $(PROJECT).eep $(PROJECT).map $(PROJECT).elf *.o dep

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

