CFLAGS=-g
MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -I../myur_168M/
#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CFLAGS=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -I../myur_168M/

rcc.bin: rcc.elf
	arm-none-eabi-objcopy -Obinary $< $@

rcc.elf: rcc.c
	arm-none-eabi-gcc -DTEST_MAIN $(MYCFLAGS) -o $@ $^
rcc.o: rcc.c
	arm-none-eabi-gcc $(CFLAGS) -c $< 

clean:
	rm -rf *.o *.bin *.elf
