# Makefile for Medulla programs that use the medulla library
# Written by: Kit Morton

# MCU Options
MCU = atxmega128a1
F_CPU = 32000000

# Output file name (without extension)
OUTPUT = medulla
OUTPUT_DIR = .

# Source Files
SOURCES = medulla.c medulla_leg.c medulla_hip.c medulla_boom.c medulla_imu.c amplifier.c crc.c
SRCDIR = src

# Medulla_lib stuff
MEDULLA_LIB_DIR = ../../../medulla/firmware/medulla_lib
DEBUG_LEVEL = -DDEBUG_LOW
#DEBUG_LEVEL = -DDEBUG_HIGH
ENABLE_ECAT = -DENABLE_ECAT
ENABLE_IMU = -DENABLE_IMU

# Build Options
OBJDIR = build
INCLUDE_DIRS = $(MEDULLA_LIB_DIR)/include include ../../robot_definitions
OPT = 2

# AVR Dude options
PROGRAMMER = avrisp2
PORT = usb

# Create the compiler flags
CFLAGS  = -DF_CPU=$(F_CPU)UL
CFLAGS += $(DEBUG_LEVEL)
CFLAGS += $(ENABLE_ECAT)
CFLAGS += $(ENABLE_IMU)
CFLAGS += -O$(OPT)
CFLAGS += -fshort-enums
CFLAGS += -Wall
CFLAGS += -include stdint.h
CFLAGS += $(patsubst %,-I%,$(INCLUDE_DIRS))
CFLAGS += -mmcu=$(MCU)
CFLAGS += -std=gnu99
CFLAGS += -lm

# avr-libc version > 1.7.0?
# TODO(yoos): This doesn't actually work.
ifeq "$(shell expr `gcc -dumpversion` \> 1.7.0)" "1"
	CFLAGS += -D__DELAY_BACKWARD_COMPATIBLE__
endif

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
CREATEDIR = mkdir -p
COPY = cp

# Define rules

all: $(OUTPUT).hex

clean:
	$(REMOVE) $(OBJS)
	$(REMOVE) $(OBJDIR)/$(OUTPUT).elf
	$(REMOVE) $(OUTPUT).hex

program: all
	$(AVRDUDE) -p $(MCU) -P $(PORT) -c $(PROGRAMMER) -U flash:w:$(OUTPUT).hex

crc_test:
	gcc crc_test.c src/crc.c -Iinclude -o crc_test

%.hex : %.elf
	@echo
	@echo Linking $<
	$(OBJCOPY) -O ihex $(OBJDIR)/$< $(OUTPUT_DIR)/$@

$(OBJDIR)/%.o : $(SRCDIR)/%.c
	@echo
	@echo Building $<
	$(CC) -c $(CFLAGS) $< -o $@

.SECONDARY : $(OUTPUT).elf
.PRECIOUS : $(OBJS)
%.elf : $(OBJS) $(MEDULLA_LIB_DIR)/lib/medulla_lib.a
	@echo
	@echo Linking $@
	$(CC) $(CFLAGS) $(OBJS) --output $(OBJDIR)/$@ $(LDFLAGS) $(MEDULLA_LIB_DIR)/lib/medulla_lib.a

# Make build directories
$(shell mkdir $(OBJDIR) 2>/dev/null)
$(shell mkdir $(OUTPUT_DIR) 2>/dev/null)

# List phony targets
.PHONY : all clean program

