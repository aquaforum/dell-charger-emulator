PROJECT=dell-charger-emulator
GCC_MCU?=atmega328p

OBJDUMP = avr-objdump
PREFIX=/home/vanessa/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin

CFLAGS?=-Os -g
CFLAGS+=-mmcu=$(GCC_MCU) -DF_CPU=32000000

AVRDUDE=/home/vanessa/.arduino15/packages/MiniCore/tools/avrdude/7.2-arduino.1/bin/avrdude
AVRDUDE_FLAGS?=-C/home/vanessa/.arduino15/packages/MiniCore/hardware/avr/3.0.0/avrdude.conf -patmega328p -curclock -P/dev/ttyUSB0 -b57600

# -v -patmega328p -carduino -P/dev/ttyUSB1  -D -Uflash:w:/tmp/arduino_build_812219/Blink.ino.hex:i 

FUSES?=-U lfuse:w:0xd2:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m

SRC=$(wildcard *.c)

all: $(PROJECT).hex

$(PROJECT).hex: $(PROJECT).elf
	$(PREFIX)/avr-objcopy -Oihex $< $@

$(PROJECT).elf: $(SRC) Makefile
	$(PREFIX)/avr-gcc -o $@ $(CFLAGS) $(SRC)
	$(PREFIX)/avr-size $(PROJECT).elf

clean:
	rm -rfv $(PROJECT).hex $(PROJECT).elf

load: load_rom load_eeprom

load_rom: $(PROJECT).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$<

load_eeprom: eeprom-data.hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U eeprom:w:$<

fuses:
	avrdude $(AVRDUDE_FLAGS) $(FUSES)

lss:  $(PROJECT).lss


%.lss: %.elf
	$(PREFIX)/$(OBJDUMP) -h -S $< > $@

