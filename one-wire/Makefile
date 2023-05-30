CC = sdcc

BUILD_DIR = build
INCLUDE_DIR = ../lcd
SOURCE_DIR = ../lcd

EXTRASRCS = \
	$(SOURCE_DIR)/lcd1602-8.c

RELS = $(EXTRASRCS:.c=.rel)

INCLUDES = -I $(INCLUDE_DIR)
CFLAGS   = -lstm8 -mstm8 --out-fmt-ihx --std-sdcc11

all: clean one-wire flash

one-wire: one-wire.c $(RELS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(INCLUDES) $(CFLAGS) one-wire.c $(wildcard $(BUILD_DIR)/*.rel) -o$(BUILD_DIR)/


.c.rel:
	@mkdir -p $(BUILD_DIR)
	$(CC) -c $(INCLUDES) $(CFLAGS) $< -o$(BUILD_DIR)/

.SUFFIXES: .c .rel

.PHONY:	clean flash

clean:
	rm -rf $(BUILD_DIR)

flash:
	sudo stm8flash -c stlinkv21 -p stm8s207k8 -w $(BUILD_DIR)/one-wire.ihx