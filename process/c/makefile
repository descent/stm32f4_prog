CFLAGS=-g
MYCFLAGS=-fno-common -O0 -g -mcpu=cortex-m3 -mthumb
MYCFLAGS_NO_LD=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
LD_FLAGS=-Wl,-T./stm32.ld -nostartfiles

all: cs.bin

%.bin:%.elf
	arm-none-eabi-objcopy -R .data -O binary $< $@

cs.elf: cs.o stm32.o
	arm-none-eabi-gcc $(LD_FLAGS) -o $@ $^

%.o:%.S
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -c $< 

%.o:%.c
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -c $< 
cs.o: cs.c stm32.h

clean:
	rm -rf *.o *.bin *.elf
