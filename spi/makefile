CFLAGS=-g
MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CFLAGS=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -I../myur_168M/


my_spi.bin: my_spi.elf
	arm-none-eabi-objcopy -Obinary $< $@

my_spi.elf: my_spi.o stm32f4xx_spi.o
	arm-none-eabi-gcc $(MYCFLAGS) -o $@ $^

my_spi.o: my_spi.c
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 

spi1.bin: spi1.elf
	arm-none-eabi-objcopy -Obinary $< $@

spi1.elf: spi1.o stm32f4xx_spi.o
	arm-none-eabi-gcc $(MYCFLAGS) -o $@ $^

spi1.o: spi1.c
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 

stm32f4xx_spi.o: stm32f4xx_spi.c
	arm-none-eabi-gcc $(CFLAGS) $(INC) -c $< 
clean:
	rm -rf *.o *.bin *.elf
