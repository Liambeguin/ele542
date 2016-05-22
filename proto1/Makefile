## Notes:
#
# ' $< ' = Make variable refering to a single dependency
# ' $^ ' = Make variable refering to all dependencies
# ' $@ ' = Make variable refering to the target of a rule
#
# Converting a .c file to a .o file is done implicitly by Make via OBJECTS


# Compiler settings
CC = avr-gcc
OBJCOPY = avr-objcopy
OPT = s

TARGET = main
SOURCES = $(wildcard src/*.c)
OBJECTS = $(SOURCES:.c=.o)
HEADERS = inc/*.h

# MCU settings
MCU = atmega32
F_CPU = 16000000UL
BAUD = 9600UL
AVRDUDE_PORT = /dev/ttyUSB1


# Used implicitly
TARGET_ARCH = -mmcu=$(MCU)
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.

CFLAGS = -g -Wall -std=gnu99 -O$(OPT) \
		 -ffunction-sections -fdata-sections \
		 -Werror


LDFLAGS = -Wl,-Map,$(TARGET).map -Wl,--gc-sections



## Targets and rules
all: flash

flash: $(TARGET).hex
	avrdude -eqV -c stk500 -p m32 -U flash:w:$< -P $(AVRDUDE_PORT)

%.hex: %.elf
	     $(OBJCOPY) -j .text -j .data -O ihex $< $@

%.elf: $(OBJECTS)
	    $(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

clean:
	    rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map

reset:
	avrdude -qV -c stk500 -p m32 -P $(AVRDUDE_PORT)
erase:
	avrdude -eqV -c stk500 -p m32 -P $(AVRDUDE_PORT)
