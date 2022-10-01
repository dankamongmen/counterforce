.PHONY: all clean

OUT:=out
MEGAHEX:=$(addprefix mega2560/, $(addsuffix .hex, fan4pwm geiger))
ESPHEX:=$(addprefix esp32/, $(addsuffix .hex, fanmgr))
UNOHEX:=$(addprefix codi6/, $(addsuffix .hex, external internal))
HEX:=$(addprefix $(OUT)/, $(MEGAHEX) $(ESPHEX) $(UNOHEX))
BIN:=$(addprefix $(OUT)/, counterforce)

ACLI:=arduino-cli

all: $(HEX) $(BIN)

$(OUT)/counterforce: pi/counterforce.c
	@mkdir -p $(@D)
	$(CC) -o $@ $< -lnotcurses $(shell pkg-config --libs notcurses)

$(OUT)/codi6/external.hex: $(addprefix codi6/external/, external.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:uno -v --output-dir $(@D) codi6/external

$(OUT)/codi6/internal.hex: $(addprefix codi6/internal/, internal.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:uno -v --output-dir $(@D) codi6/internal

$(OUT)/esp32/fanmgr.hex: $(addprefix esp32/fanmgr/, EspMQTTConfig.h fanmgr.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b heltec:esp32:wifi_lora_32_V2 -v --output-dir $(@D) esp32/fanmgr

$(OUT)/mega2560/geiger.hex: $(addprefix mega2560/geiger/, geiger.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:mega -v --output-dir $(@D) mega2560/geiger

$(OUT)/mega2560/fan4pwm.hex: $(addprefix mega2560/fan4pwm/, fan4pwm.ino)
	@mkdir -p $(@D)
	$(ACLI) compile -b arduino:avr:mega -v --output-dir $(@D) mega2560/fan4pwm

clean:
	rm -rf $(OUT)
