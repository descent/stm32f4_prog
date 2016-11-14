all: cc936.o  syscall.o  unicode.o

%.o: %.c
	$(CC) -I../../ -I../../../inc/ -I../src -c $<


clean:
	rm -rf *.o
