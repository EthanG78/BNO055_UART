CC = gcc

BASEFLAGS = -Wall -std=c99
DEBUG_FLAGS = -g

OBJS = bno055_uart.o

EXE = bno055_uart

release: CFLAGS = $(BASEFLAGS)
release: $(EXE)

debug: CFLAGS = $(BASEFLAGS) $(DEBUG_FLAGS)
debug: $(EXE)

$(EXE): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(EXE)

bno055_uart.o: bno055_uart.c bno055_uart.h
	$(CC) $(CFLAGS) -c bno055_uart.c

clean:
	rm -f $(OBJS)
	rm -f *~
	rm -f $(EXE)

run:
	./$(EXE)