CC=sdcc
CFLAGS=-lstm8 -mstm8 --out-fmt-ihx --std-sdcc11 

all: lcd1602-8 flash

lcd1602-8:
	$(CC) $(CFLAGS) lcd1602-8.c

flash:
	sudo stm8flash -c stlinkv21 -p stm8s207k8 -w lcd1602-8.ihx
