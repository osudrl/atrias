# Makefile for the Medulla library
# Written by: Kit Morton

# MCU Options
MCU = atxmega128a1
F_CPU = 32000000

# Source Files
SOURCES = uart.c spi.c cpu.c ethercat.c io_pin.c pwm.c limit_switch.c estop.c biss_encoder.c ssi_encoder.c quadrature_encoder.c dzralte_comm.c adc.c

# Output file and directory
OUTPUT = medulla_lib
OUTPUT_DIR = ../lib

# GCC Options
OBJDIR = ../build
INCLUDE_DIRS = ../include
OPT = 2

# Create the compiler options
CFLAGS  = -DF_CPU=$(F_CPU)UL
CFLAGS += -O$(OPT)
CFLAGS += -Wall
CFLAGS += -include stdint.h
CFLAGS += $(patsubst %,-I%,$(INCLUDE_DIRS))
CFLAGS += -mmcu=$(MCU)
CFLAGS += -std=gnu99

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

all: $(OUTPUT).a

clean:
	$(REMOVE) $(OBJS)
	$(REMOVE) $(OUTPUT_DIR)/$(OUTPUT).a

$(OBJDIR)/%.o : %.c
	@echo
	@echo Building $<
	$(CC) -c $(CFLAGS) $< -o $@

.SECONDARY : $(OUTPUT).a
.PRECIOUS : $(OBJS)
%.a : $(OBJS)
	@echo
	@echo Linking $@
	$(AR) $(OUTPUT_DIR)/$@ $(OBJS)

# Make build directories
$(shell mkdir $(OBJDIR) 2>/dev/null)
$(shell mkdir $(OUTPUT_DIR) 2>/dev/null)

# List phony targets
.PHONY : all clean

