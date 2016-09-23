CFLAGS=-g
MYCFLAGS=-fno-common -O0 -g -mcpu=cortex-m3 -mthumb
MYCFLAGS_NO_LD=-nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb
LD_FLAGS=-Wl,-T./stm32.ld -nostartfiles

all: mygpio_led.bin

%.o:%.S
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -c $< 
	#arm-none-eabi-gcc $(MYCFLAGS_NO_LD) $(INC) -c $< 

%.o:%.c
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -c $< 


%.bin:%.elf
	arm-none-eabi-objcopy -R .data -O binary $< $@


mygpio_led.bin: mygpio_led.elf
	arm-none-eabi-objcopy -Obinary $< $@
mygpio_led.elf: mygpio_led.o
	arm-none-eabi-gcc $(LD_FLAGS) $(INC) -o $@ $< 
mygpio_led.o: mygpio_led.c stm32.h 
	arm-none-eabi-gcc $(MYCFLAGS) $(INC) -c $< 
clean:
	rm -rf *.o *.bin *.elf
