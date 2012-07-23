# Makefile for Medulla programs that use the medulla library
# Written by: Kit Morton

# MCU Options
MCU = atxmega128a1
F_CPU = 32000000

# Output file name (without extension)
OUTPUT = medulla_lib_test
OUTPUT_DIR = .

# Source Files
SOURCES = medulla_lib_test.c

# Medulla_lib stuff
MEDULLA_LIB_DIR = ../medulla_lib

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

all: $(OUTPUT).hex size

clean:
	$(REMOVE) $(OBJS)
	$(REMOVE) $(OUTPUT_DIR)/$(OUTPUT).elf
	$(REMOVE) $(OUTPUT_DIR)/$(OUTPUT).hex

program: all
	$(AVRDUDE) -p $(MCU) -P $(PORT) -c $(PROGRAMMER) -U flash:w:$(OUTPUT).hex

size:
	@if test -f $(OUTPUT_DIR)/$(OUTPUT).elf; then echo; echo Size of: $(OUTPUT).elf; $(SIZE) --mcu=$(MCU) --format=avr $(OUTPUT_DIR)/$(OUTPUT).elf; \
	2>/dev/null; echo; fi

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

