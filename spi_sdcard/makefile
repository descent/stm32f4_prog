CFLAGS=-g
#MYCFLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-common -O0 -g -mcpu=cortex-m3 -mthumb

CFLAGS=-fno-common -O0 -g -mcpu=cortex-m3 -mthumb -fno-builtin-function -fno-builtin -ffreestanding -nostartfiles -fno-common
MYCXXFLAGS = -fno-exceptions -fno-rtti

LD_FLAGS=-Wl,-T./stm32.ld -nostartfiles -fno-builtin-function -fno-builtin -ffreestanding

#MYCFLAGS=-Wl,-T./stm32.ld -g -O2 -mlittle-endian -mthumb -mcpu=cortex-m4 -ffreestanding -nostdlib 

CXXFLAGS += $(MYCXXFLAGS) $(CFLAGS)

myur_168M.bin: myur_168M.elf
	arm-none-eabi-objcopy -Obinary $< $@

myur_168M.o: myur_168M.cpp stm32.h stm32f4xx_spi.h stm32f4xx.h core_cm4.h \
 core_cmInstr.h core_cmFunc.h core_cm4_simd.h stm32f4xx_usart.h \
  stm32f4xx_rcc.h stm32f4xx_gpio.h
	arm-none-eabi-g++ $(CXXFLAGS) $(INC) -c $< 

myur_168M.elf: myur_168M.o
	arm-none-eabi-gcc $(LD_FLAGS) $(INC) -o $@ $< 

clean:
	rm -rf *.o *.bin *.elf
