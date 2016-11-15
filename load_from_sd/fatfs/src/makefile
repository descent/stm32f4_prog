include ../../rule.mk
#CFLAGS=-g -m32
all: diskio.o  ff.o

%.o: %.c
	$(CC) $(CFLAGS) -I../../ -I../../../inc/ -I../src -c $<


clean:
	rm -rf *.o
