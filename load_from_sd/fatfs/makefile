libfat_objs =  ./exfuns/fattester.o ./exfuns/exfuns.o  ./src/diskio.o ./src/ff.o src/option/cc936.o  src/option/syscall.o  src/option/unicode.o 
libfat.a: src_objs exfuns_objs opt_objs
	$(AR) rcs -o $@ $(libfat_objs)
exfuns_objs:  ./exfuns/fattester.o ./exfuns/exfuns.o
	(cd exfuns; make)
src_objs: ./src/diskio.o ./src/ff.o 
	(cd src; make)
opt_objs: src/option/cc936.o  src/option/syscall.o  src/option/unicode.o
	(cd src/option; make)
clean:
	(cd exfuns; make clean)
	(cd src; make clean)
	(cd src/option; make clean)
	rm -rf libfat.a
