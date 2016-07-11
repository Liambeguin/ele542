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
OPT = 0

TARGET = main
SOURCES = $(wildcard src/*.c)
OBJECTS = $(SOURCES:.c=.o)
HEADERS = inc/*.h

# MCU settings
MCU = atmega32
F_CPU = 16000000
BAUD = 9600
SERIAL_PORT = COM1

# AVRDUDE Settings
AVRDUDE_FLAGS = -qV \
		-c stk500 \
		-p m32


# Used implicitly
TARGET_ARCH = -mmcu=$(MCU)
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.

# Detail:
#  -Wall show all warnings
#  -Wextra show even more warnings
#  -Werror all warnings are errors
#  -Wno-cpp exclude #warning statments

CFLAGS = -g -Wall -std=gnu99 -O$(OPT) \
		 -ffunction-sections -fdata-sections

LDFLAGS = -Wl,-Map,$(TARGET).map -Wl,--gc-sections



## Targets and rules
all: clean flash

proto: $(TARGET).hex
	:

demo:
	avrdude -e $(AVRDUDE_FLAGS) -U flash:w:demo.hex

flash: $(TARGET).hex
	avrdude -e $(AVRDUDE_FLAGS) -U flash:w:$<
reset:
	avrdude $(AVRDUDE_FLAGS)
erase:
	avrdude -e $(AVRDUDE_FLAGS)
serial:
	picocom --omap crcrlf -b $(BAUD) $(SERIAL_PORT)



%.hex: %.elf
	     $(OBJCOPY) -j .text -j .data -O ihex $< $@

%.elf: $(OBJECTS)
	    $(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

clean:
	    rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map
