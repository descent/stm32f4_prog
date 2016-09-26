CFLAGS=-g
MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CFLAGS=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -I../myur_168M/
my_timer.bin: my_timer.elf
	arm-none-eabi-objcopy -Obinary $< $@
my_timer.elf: my_timer.o
	arm-none-eabi-gcc $(MYCFLAGS) -o $@ $< 
my_timer.o: my_timer.c
	arm-none-eabi-gcc $(CFLAGS) -c $< 


clean:
	rm -rf *.o *.bin *.elf
