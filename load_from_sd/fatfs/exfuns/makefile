CFLAGS=-g
all: exfuns.o  fattester.o

%.o: %.c
	$(CC) $(CFLAGS) -I../../../inc/ -I../src -c $<


clean:
	rm -rf *.o
