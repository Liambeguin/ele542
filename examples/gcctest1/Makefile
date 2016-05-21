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

TARGET = gcctest1
OBJECTS = $(TARGET).o

# MCU settings
MCU = atmega16
# F_CPU = 16000000UL
F_CPU = 160000UL
BAUD = 9600UL
AVRDUDE_PORT = /dev/ttyUSB1


# Used implicitly
TARGET_ARCH = -mmcu=$(MCU)
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.

CFLAGS = -g -Werror -Wall -std=gnu99 -O$(OPT) \
		 -ffunction-sections -fdata-sections


LDFLAGS = -Wl,-Map,$(TARGET).map -Wl,--gc-sections



## Targets and rules
all: flash

flash: $(TARGET).hex
	avrdude -eqV -c stk500 -p m32 -U flash:w:$(TARGET).hex -P $(AVRDUDE_PORT)

%.hex: %.elf
	     $(OBJCOPY) -j .text -j .data -O ihex $< $@

%.elf: $(OBJECTS)
	    $(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

clean:
	    rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map
