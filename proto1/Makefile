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
SERIAL_PORT = /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0

# AVARICE Settings
AVARICE_FLAGS = -g -j usb

# Used implicitly
TARGET_ARCH = -mmcu=$(MCU)
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.

# Detail:
#  -Wall show all warnings
#  -Wextra show even more warnings
#  -Werror all warnings are errors
#  -Wno-cpp exclude #warning statments

CFLAGS = -g -Wall -std=gnu99 -O$(OPT) \
		 -ffunction-sections -fdata-sections \
		 -Werror -Wextra -Wno-cpp

LDFLAGS = -Wl,-Map,$(TARGET).map -Wl,--gc-sections



## Targets and rules
all: flash

proto: $(TARGET).hex
	:

flash: $(TARGET).hex
	avarice $(AVARICE_FLAGS) --erase --program --file $<
reset:
	avarice $(AVARICE_FLAGS)
erase:
	avarice $(AVARICE_FLAGS) --erase
serial:
	picocom --omap crcrlf -b $(BAUD) $(SERIAL_PORT)



%.hex: %.elf
	     $(OBJCOPY) -j .text -j .data -O ihex $< $@

%.elf: $(OBJECTS)
	    $(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

clean:
	    rm -f $(TARGET).elf $(TARGET).hex $(TARGET).map
