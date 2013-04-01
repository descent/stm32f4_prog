CFLAGS=-g
MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb

factorial.bin: factorial.elf
	arm-none-eabi-objcopy -O binary $< $@

factorial.elf: factorial.o
	arm-none-eabi-ld -Ttext 0x0 -o $@ $<
	
factorial.o: factorial.S
	arm-none-eabi-as -g -mcpu=cortex-m3 -mthumb -o $@ $<
	#arm-none-eabi-gcc $(CFLAGS) -mcpu=cortex-m3 -mthumb -nostartfiles -c hello.c

hello.bin: hello.elf
	arm-none-eabi-objcopy -O binary hello.elf hello.bin

hello.o: hello.c stm32.h
	arm-none-eabi-gcc $(CFLAGS) -mcpu=cortex-m3 -mthumb -nostartfiles -c hello.c

hello.elf: hello.o
	arm-none-eabi-ld -T stm32.ld -o hello.elf hello.o

mymain.bin: mymain.elf
	arm-none-eabi-objcopy -Obinary $< $@

mymain.elf: mymain.c
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -o $@ $< 
clean:
	rm -rf *.o *.bin *.elf
