include ../../../rule.mk
#CFLAGS=-g -m32
all: cc936.o  syscall.o  unicode.o

%.o: %.c
	$(CC) $(CFLAGS) -I../../ -I../../../../inc/ -I../src -c $<


clean:
	rm -rf *.o
