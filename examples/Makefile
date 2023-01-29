CC = gcc

BASEFLAGS = -Wall
DEBUG_FLAGS = -g
LIBS = -lwiringPi -lbno055_uart

$(EXE) bno055_uart_test bno055_uart_system_test

release: CFLAGS = $(BASEFLAGS)
release: $(EXE)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(EXE)

bno055_uart_test: bno055_uart_test.c
	$(CC) $(CFLAGS) bno055_uart_test.c -o bno055_uart_test $(LIBS)

bno055_uart_system_test: bno055_uart_system_test.c 
	$(CC) $(CFLAGS) bno055_uart_system_test.c  -o bno055_uart_system_test $(LIBS)

clean:
	rm -f $(EXE)