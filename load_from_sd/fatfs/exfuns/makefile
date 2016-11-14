all: exfuns.o  fattester.o

%.o: %.c
	$(CC) -I../../../inc/ -I../src -c $<


clean:
	rm -rf *.o
