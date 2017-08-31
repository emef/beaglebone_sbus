CC = gcc
CFLAGS = -g -Wall
HEADERS = ../sbus.h

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

all: example_writer example_reader

example_writer: example_writer.o
	$(CC) $^ -Wall -o $@

example_reader: example_reader.o
	$(CC) $^ -Wall -o $@

clean:
	-rm -f *.o
	-rm -f example_writer example_reader
