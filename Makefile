CC = gcc

BASEFLAGS = -Wall -std=c99
DEBUG_FLAGS = -g
LIBS = -lwiringPi

BINS = libbno055_uart.so

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

release: CFLAGS = $(BASEFLAGS) $(LIBS)
release: $(BINS)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS) $(LIBS)
debug: $(BINS)

libbno055_uart.so: bno055_uart.c bno055_uart.h
	$(CC) $(CFLAGS) -fPIC -shared -o $@ bno055_uart.c -lc

clean:
	rm -f $(OBJS)
	rm -f *~
	rm -f $(BINS)

install: libbno055_uart.so
    install -m 644 libbno055_uart.so $(PREFIX)/lib/
	install -m 644 bno055_uart.h $(PREFIX)/include/