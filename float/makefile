include rule.mk

CFLAGS += $(MYCFLAGS)
CXXFLAGS += $(MYCXXFLAGS) $(CFLAGS)

float.bin: float.elf
	arm-none-eabi-objcopy -Obinary $< $@
float.elf: float.o k_stdio.o
	$(CXX) $(LD_FLAGS) -o $@ $^ -lgcc
	#$(CXX) $(LD_FLAGS) -o $@ $^

sources = float.cpp  k_stdio.cpp

include $(sources:.cpp=.dpp)

clean:
	rm -rf *.o *.elf *.bin *.dpp *.dpp.*
distclean: clean
	find . -type l -exec rm -f {} \;
