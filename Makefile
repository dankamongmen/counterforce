.PHONY: all clean

OUT:=out
MEGAHEX:=$(addprefix mega2560/, $(addsuffix .hex, fan4pwm geiger))
ESP32HEX:=$(addprefix esp32/, $(addsuffix .hex, fanmgr))
ESP8266HEX:=$(addprefix esp8266/, $(addsuffix .hex, fanmgr))
ESPCOMMON:=$(addprefix espcommon/fanmgr/, common.h EspMQTTConfig.h)
UNOHEX:=$(addprefix codi6/, $(addsuffix .hex, external internal))
HEX:=$(addprefix $(OUT)/, $(MEGAHEX) $(ESP32HEX) $(ESP8266HEX) $(UNOHEX))
BIN:=$(addprefix $(OUT)/, counterforce)
CFLAGS:=

ACLI:=arduino-cli

all: $(HEX) $(BIN)

$(OUT)/counterforce: pi/counterforce.c
	@mkdir -p $(@D)
	$(CC) -o $@ $< -lnotcurses $(shell pkg-config --libs notcurses)

$(OUT)/codi6/mora.hex: $(addprefix codi6/mora/, mora.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/mora

$(OUT)/codi6/external.hex: $(addprefix codi6/external/, external.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/external

$(OUT)/codi6/internal.hex: $(addprefix codi6/internal/, internal.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/internal

$(OUT)/esp8266/fanmgr.hex: $(addprefix esp8266/fanmgr/, fanmgr.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b esp8266:esp8266:nodemcuv2 -v --output-dir $(@D) esp8266/fanmgr

$(OUT)/esp32/fanmgr.hex: $(addprefix esp32/fanmgr/, fanmgr.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b Heltec-esp32:esp32:wifi_lora_32_V2 -v --output-dir $(@D) esp32/fanmgr

$(OUT)/mega2560/geiger.hex: $(addprefix mega2560/geiger/, geiger.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:mega -v --output-dir $(@D) mega2560/geiger

$(OUT)/mega2560/fan4pwm.hex: $(addprefix mega2560/fan4pwm/, fan4pwm.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:mega -v --output-dir $(@D) mega2560/fan4pwm

clean:
	rm -rf $(OUT)
