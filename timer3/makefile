CFLAGS=-g
MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CFLAGS=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -I../myur_168M/

timer3.bin: timer3.elf
	arm-none-eabi-objcopy -Obinary $< $@

timer3.elf: timer3.o stm32f4xx_tim.o
	arm-none-eabi-gcc $(MYCFLAGS) -o $@ $^
timer3.o: timer3.c
	arm-none-eabi-gcc $(CFLAGS) -c $< 

stm32f4xx_tim.o: stm32f4xx_tim.c
	arm-none-eabi-gcc $(CFLAGS) -c $< 

clean:
	rm -rf *.o *.bin *.elf
