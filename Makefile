CC = gcc

BASEFLAGS = -Wall
DEBUG_FLAGS = -g
LIBS = -lc -lwiringPi

BINS = libbno055_uart.so

ifeq ($(PREFIX),)
    PREFIX := /usr
endif

release: CFLAGS = $(BASEFLAGS)
release: $(BINS)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(BINS)

libbno055_uart.so: bno055_uart.c bno055_uart.h
	$(CC) $(CFLAGS) -fPIC -shared -o $@ bno055_uart.c $(LIBS)

clean:
	rm -f $(BINS)

install: libbno055_uart.so
	install -m 644 libbno055_uart.so $(PREFIX)/lib/
	install -m 644 bno055_uart.h $(PREFIX)/include/
	rm -f $(BINS)