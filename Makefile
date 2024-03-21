.PHONY: all clean

OUT:=out
MEGAHEX:=$(addprefix mega2560/, $(addsuffix .ino.hex, fan4pwm geiger))
ESP32WROOM2HEX:=$(addprefix esp32-wroom2/, $(addsuffix .ino.hex, fanmgr))
ESPCOMMON:=$(addprefix espcommon/fanmgr/, common.h EspMQTTConfig.h)
UNOHEX:=$(addprefix codi6/, $(addsuffix .ino.hex, external internal mora))
HEX:=$(addprefix $(OUT)/, $(MEGAHEX) $(UNOHEX))
BIN:=$(addprefix $(OUT)/, counterforce)
CFLAGS:=

ACLI:=arduino-cli

all: $(HEX) $(BIN)

$(OUT)/counterforce: pi/counterforce.c
	@mkdir -p $(@D)
	$(CC) -o $@ $< -lnotcurses $(shell pkg-config --libs notcurses)

$(OUT)/codi6/mora.ino.hex: $(addprefix codi6/mora/, mora.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/mora

$(OUT)/codi6/external.ino.hex: $(addprefix codi6/external/, external.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/external

$(OUT)/codi6/internal.ino.hex: $(addprefix codi6/internal/, internal.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/internal

$(OUT)/esp32-wroom2/fanmgr.ino.hex: $(addprefix esp32-wroom2/fanmgr/, fanmgr.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b esp32:esp32:node32s -v --output-dir $(@D) esp32-wroom2/fanmgr

$(OUT)/esp32-wroom2/simple.ino.hex: $(addprefix esp32-wroom2/simple/, simple.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b esp32:esp32:node32s -v --output-dir $(@D) esp32-wroom2/simple

$(OUT)/mega2560/geiger.ino.hex: $(addprefix mega2560/geiger/, geiger.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:mega -v --output-dir $(@D) mega2560/geiger

$(OUT)/mega2560/fan4pwm.ino.hex: $(addprefix mega2560/fan4pwm/, fan4pwm.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:mega -v --output-dir $(@D) mega2560/fan4pwm

clean:
	rm -rf $(OUT)
