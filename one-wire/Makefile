CC = sdcc

ODIR = bin
IDIR = ../lcd
SDIR = ../lcd

EXTRASRCS = \
	$(SDIR)/lcd1602-8.c

RELS = $(EXTRASRCS:.c=.rel)

INCLUDES = -I$(IDIR)
CFLAGS   = -lstm8 -mstm8 --out-fmt-ihx --std-sdcc11

all: clean one-wire flash

one-wire: one-wire.c $(RELS)
	@mkdir -p $(ODIR)
	$(CC) $(INCLUDES) $(CFLAGS) $(LIBS) one-wire.c $(wildcard $(ODIR)/*.rel) -o$(ODIR)/


.c.rel:
	@mkdir -p $(ODIR)
	$(CC) -c $(INCLUDES) $(CFLAGS) $(LIBS) $< -o$(ODIR)/

.SUFFIXES: .c .rel

.PHONY:	clean flash

clean:
	rm -rf $(ODIR)

flash:
	sudo stm8flash -c stlinkv21 -p stm8s207k8 -w $(ODIR)/one-wire.ihx