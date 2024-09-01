.PHONY: all clean

OUT:=out
MEGAHEX:=$(addprefix mega2560/, $(addsuffix .ino.hex, geiger))
ESP32WROVERHEX:=$(addprefix esp32-wrover/, $(addsuffix .ino.elf, s-mora r-mora bambux1c))
ESPCOMMON:=$(addprefix espcommon/, fanmgr/common.h espcommon.h fanmgr/EspMQTTConfig.h)
UNOHEX:=$(addprefix codi6/, $(addsuffix .ino.hex, external internal mora))
UNO4HEX:=$(addprefix unor4/, $(addsuffix .ino.hex, airmon))
HEX:=$(addprefix $(OUT)/, $(MEGAHEX) $(UNOHEX) $(UNO4HEX) $(ESP32WROVERHEX))
BIN:=$(addprefix $(OUT)/, counterforce)
CFLAGS:=

ACLI:=arduino-cli

all: $(HEX) $(BIN)

$(OUT)/counterforce: pi/counterforce.c
	@mkdir -p $(@D)
	$(CC) $(shell pkg-config --cflags notcurses) -o $@ $< -lnotcurses $(shell pkg-config --libs notcurses)

$(OUT)/codi6/mora.ino.hex: $(addprefix codi6/mora/, mora.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/mora

$(OUT)/codi6/external.ino.hex: $(addprefix codi6/external/, external.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/external

$(OUT)/codi6/internal.ino.hex: $(addprefix codi6/internal/, internal.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:uno -v --output-dir $(@D) codi6/internal

$(OUT)/esp32-wrover/s-mora.ino.elf: $(addprefix esp32-wrover/s-mora/, s-mora.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b esp32:esp32:node32s -v --output-dir $(@D) esp32-wrover/s-mora

$(OUT)/esp32-wrover/r-mora.ino.elf: $(addprefix esp32-wrover/r-mora/, r-mora.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b esp32:esp32:node32s -v --output-dir $(@D) esp32-wrover/r-mora

$(OUT)/esp32-wrover/bambux1c.ino.elf: $(addprefix esp32-wrover/bambux1c/, bambux1c.ino) $(ESPCOMMON)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b esp32:esp32:node32s -v --output-dir $(@D) esp32-wrover/bambux1c

$(OUT)/mega2560/geiger.ino.hex: $(addprefix mega2560/geiger/, geiger.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:avr:mega -v --output-dir $(@D) mega2560/geiger

$(OUT)/unor4/airmon.ino.hex: $(addprefix unor4/airmon/, airmon.ino)
	@mkdir -p $(@D)
	$(ACLI) compile $(CFLAGS) -b arduino:renesas_uno:unor4wifi -v --output-dir $(@D) unor4/airmon

clean:
	rm -rf $(OUT)
