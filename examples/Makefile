CC = gcc

BASEFLAGS = -Wall
DEBUG_FLAGS = -g
LIBS = -lwiringPi -lbno055_uart

SOURCE = bno055_uart_init_test.c

EXE = bno055_uart_test

release: CFLAGS = $(BASEFLAGS)
release: $(EXE)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(EXE)

$(EXE): $(SOURCE)
	$(CC) $(CFLAGS) $(SOURCE) -o $(EXE) $(LIBS)

clean:
	rm -f $(EXE)