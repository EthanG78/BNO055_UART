CC = gcc

BASEFLAGS = -Wall
DEBUG_FLAGS = -g
LIBS = -lwiringPi -lbno055_uart

EXE = bno055_uart_init_test bno055_uart_system_test bno055_uart_cal_test

release: CFLAGS = $(BASEFLAGS)
release: $(EXE)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(EXE)

bno055_uart_init_test: bno055_uart_init_test.c
	$(CC) $(CFLAGS) bno055_uart_init_test.c -o bno055_uart_init_test $(LIBS)

bno055_uart_system_test: bno055_uart_system_test.c 
	$(CC) $(CFLAGS) bno055_uart_system_test.c  -o bno055_uart_system_test $(LIBS)

bno055_uart_cal_test: bno055_uart_cal_test.c 
	$(CC) $(CFLAGS) bno055_uart_cal_test.c  -o bno055_uart_cal_test $(LIBS)

clean:
	rm -f $(EXE)