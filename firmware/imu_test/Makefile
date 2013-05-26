# Makefile for Medulla programs that use the medulla library
# Written by: Kit Morton

# MCU Options
MCU = atxmega128a1
F_CPU = 32000000

# Output file name (without extension)
OUTPUT = imu_test
OUTPUT_DIR = .

# Source Files
SOURCES = imu_test.c

# Medulla_lib stuff
MEDULLA_LIB_DIR = ../../../medulla/firmware/medulla_lib

# Build Options
OBJDIR = .
INCLUDE_DIRS = $(MEDULLA_LIB_DIR)/include
OPT = 2

# AVR Dude options
PROGRAMMER = avrisp2
PORT = usb

# Create the compiler flags
CFLAGS  = -DF_CPU=$(F_CPU)UL
CFLAGS += -O$(OPT)
CFLAGS += -Wall
CFLAGS += -include stdint.h
CFLAGS += $(patsubst %,-I%,$(INCLUDE_DIRS))
CFLAGS += -mmcu=$(MCU)
CFLAGS += -std=gnu99
CFLAGS += -lm
CFLAGS += -D__DELAY_BACKWARD_COMPATIBLE__   # The new implementation of _delay_us(double __us) with __builtin_avr_delay_cycles(unsigned long) support is not backward compatible. This get us a backward compatible delay.

# Define all object files
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SOURCES))

# Define the programs
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AR = avr-ar rcs
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp

# Define rules

all: $(OUTPUT).hex

clean:
	$(REMOVE) $(OBJS)
	$(REMOVE) $(OUTPUT_DIR)/$(OUTPUT).elf
	$(REMOVE) $(OUTPUT_DIR)/$(OUTPUT).hex

program: all
	$(AVRDUDE) -p $(MCU) -P $(PORT) -c $(PROGRAMMER) -U flash:w:$(OUTPUT).hex

%.hex : %.elf
	@echo
	@echo Linking $<
	$(OBJCOPY) -O ihex $(OUTPUT_DIR)/$< $(OUTPUT_DIR)/$@

$(OBJDIR)/%.o : %.c
	@echo
	@echo Building $<
	$(CC) -c $(CFLAGS) $< -o $@

.SECONDARY : $(OUTPUT).elf
.PRECIOUS : $(OBJS)
%.elf : $(OBJS) $(MEDULLA_LIB_DIR)/lib/medulla_lib.a
	@echo
	@echo Linking $@
	$(CC) $(CFLAGS) $(OBJS) --output $(OUTPUT_DIR)/$@ $(LDFLAGS) $(MEDULLA_LIB_DIR)/lib/medulla_lib.a

# Make build directories
$(shell mkdir $(OBJDIR) 2>/dev/null)
$(shell mkdir $(OUTPUT_DIR) 2>/dev/null)

# List phony targets
.PHONY : all clean program size

