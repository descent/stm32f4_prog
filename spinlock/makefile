CFLAGS=-g
MYCFLAGS=-fno-common -O0 -g -mcpu=cortex-m3 -mthumb
MYCFLAGS_NO_LD=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
LD_FLAGS=-Wl,-T./stm32.ld -nostartfiles


%.o:%.c
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -c $< 
clean:
	rm -rf *.o *.bin *.elf
