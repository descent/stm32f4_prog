CFLAGS=-g
MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CFLAGS=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb

myur_168M.o: myur_168M.cpp
	arm-none-eabi-g++ $(CFLAGS) $(INC) -c $< 

myur_168M.bin: myur_168M.elf
	arm-none-eabi-objcopy -Obinary $< $@
myur_168M.elf: myur_168M.o
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -o $@ $< 

clean:
	rm -rf *.o *.bin *.elf
