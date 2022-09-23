.PHONY: all clean

OUT:=out
MEGAHEX:=$(addsuffix .hex, fan4pwm geiger)
ESPHEX:=$(addsuffix .hex, fanmgr)
UNOHEX:=$(addsuffix .hex, codi6)
HEX:=$(addprefix $(OUT)/, $(MEGAHEX) $(ESPHEX) $(UNOHEX))
BIN:=$(addprefix $(OUT)/, counterforce)

ACLI:=arduino-cli

all: $(HEX) $(BIN)

$(OUT)/counterforce: pi/counterforce.c
	@mkdir -p $(@D)
	$(CC) -o $@ $< -lnotcurses $(shell pkg-config --libs notcurses)

$(OUT)/codi6.hex: $(addprefix codi6/, codi6.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:uno -v --output-dir $(@D) codi6

$(OUT)/fanmgr.hex: $(addprefix esp32/fanmgr/, EspMQTTConfig.h fanmgr.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b heltec:esp32:wifi_lora_32_V2 -v --output-dir $(@D) esp32/fanmgr

$(OUT)/geiger.hex: $(addprefix mega2560/geiger/, geiger.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:mega -v --output-dir $(@D) mega2560/geiger

$(OUT)/fan4pwm.hex: $(addprefix mega2560/fan4pwm/, fan4pwm.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:mega -v --output-dir $(@D) mega2560/fan4pwm

clean:
	rm -rf $(OUT)
