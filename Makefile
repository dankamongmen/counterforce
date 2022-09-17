.PHONY: all clean

MEGAHEX:=$(addsuffix .hex, fan4pwm geiger)
ESPHEX:=$(addsuffix .hex, fanmgr)
HEX:=$(MEGAHEX) $(ESPHEX)

ACLI:=arduino-cli

all: $(HEX)

fanmgr.hex: $(addprefix esp32/fanmgr/, EspMQTTConfig.h fanmgr.ino)
	$(ACLI) compile -b heltec:esp32:wifi_lora_32_V2 -v --output-dir . esp32/fanmgr

geiger.hex: $(addprefix mega2560/geiger/, geiger.ino)
	$(ACLI) compile -b arduino:avr:mega -v --output-dir . mega2560/geiger

fan4pwm.hex: $(addprefix mega2560/fan4pwm/, fan4pwm.ino)
	$(ACLI) compile -b arduino:avr:mega -v --output-dir . mega2560/fan4pwm

clean:
	rm -f $(HEX)
