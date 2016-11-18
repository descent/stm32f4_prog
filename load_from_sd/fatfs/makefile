include ../rule.mk
libfat_objs =  ./exfuns/fattester.o ./exfuns/exfuns.o  ./src/diskio.o ./src/ff.o src/option/cc950.o  src/option/syscall.o  src/option/unicode.o 

TARGET=libfat.a
ifeq ($(ARCH), cm3) 
TARGET=libfat.cm3.a
else
endif
$(TARGET): $(libfat_objs)
	$(AR) rcs -o $@ $(libfat_objs)
exfuns/fattester.o:
	(cd exfuns; make fattester.o)
exfuns/exfuns.o:
	(cd exfuns; make exfuns.o)
src/diskio.o:
	(cd src; make diskio.o)
src/ff.o:
	(cd src; make ff.o)

src/option/cc936.o:
	(cd src/option; make cc936.o)
src/option/cc950.o:
	(cd src/option; make cc950.o)
src/option/syscall.o:
	(cd src/option; make syscall.o)
src/option/unicode.o:
	(cd src/option; make unicode.o)
clean:
	(cd exfuns; make clean)
	(cd src; make clean)
	(cd src/option; make clean)
	rm -rf libfat.a
