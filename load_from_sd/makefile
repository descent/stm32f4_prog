MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CFLAGS=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -I../myur_168M/ -I../inc/ -std=c99 -g

all: load_elf

spi_sdcard.bin: spi_sdcard.elf
	arm-none-eabi-objcopy -Obinary $< $@

spi_sdcard.elf: spi_sdcard.o stm32f4xx_spi.o mmc_sd.o
	arm-none-eabi-gcc $(MYCFLAGS) -o $@ $^

spi_sdcard.o: spi_sdcard.c
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 
sd.o: sd.c
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 
mmc_sd.o: mmc_sd.c mmc_sd.h
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 

stm32f4xx_spi.o: stm32f4xx_spi.c
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 
load_elf: load_elf.c
	gcc -I../inc -DTEST_MAIN -o $@ $<
clean:
	rm -rf *.o *.bin *.elf
