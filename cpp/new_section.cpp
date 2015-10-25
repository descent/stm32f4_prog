// extern int __exidx_end, __exidx_start;

__attribute__((section("s1")))
int new_s1(int i)
{
  const char* s="i am s1";
  int a=5, b=6, c;
  c = a + b;
  return c;
}



#if 0
 using exception, will add 


        __exidx_start = .;
        .ARM.exidx (NOLOAD) : AT(_etext)
        {
                *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        } > SRAM
        __exidx_end = .;

if no the ld script
will get the error message:
/usr/lib/gcc/arm-none-eabi/4.8/libgcc.a(unwind-arm.o): In function `get_eit_entry':
/build/gcc-arm-none-eabi-DEMfr1/gcc-arm-none-eabi-11/build/arm-none-eabi/libgcc/../../../gcc-4.8.3/libgcc/unwind-arm-common.inc:221: undefined reference to `__exidx_end'
/build/gcc-arm-none-eabi-DEMfr1/gcc-arm-none-eabi-11/build/arm-none-eabi/libgcc/../../../gcc-4.8.3/libgcc/unwind-arm-common.inc:221: undefined reference to `__exidx_start'




bfd/elf32-arm.c:14750:  return (CONST_STRNEQ (name, ELF_STRING_ARM_unwind)
bfd/elf32-arm.c:14751:	  || CONST_STRNEQ (name, ELF_STRING_ARM_unwind_once));
gas/config/tc-arm.c:21158:      prefix = ELF_STRING_ARM_unwind;
gas/config/tc-arm.c:21159:      prefix_once = ELF_STRING_ARM_unwind_once;
gas/config/tc-arm.c:21164:      prefix = ELF_STRING_ARM_unwind_info;
gas/config/tc-arm.c:21165:      prefix_once = ELF_STRING_ARM_unwind_info_once;
include/elf/arm.h:328:#define ELF_STRING_ARM_unwind           ".ARM.exidx"
include/elf/arm.h:329:#define ELF_STRING_ARM_unwind_info      ".ARM.extab"
include/elf/arm.h:330:#define ELF_STRING_ARM_unwind_once      ".gnu.linkonce.armexidx."
include/elf/arm.h:331:#define ELF_STRING_ARM_unwind_info_once ".gnu.linkonce.armextab."
#endif


// arm-none-eabi-g++ -fno-common -O0 -g -mcpu=cortex-m3 -mthumb -fomit-frame-pointer  -c new_section.cpp 

#if 1
int throw_test()
{ 
  int i=0;
  i+=2;
  try 
  {
    if (5 > 0) 
    {
      throw "something wrong...";
    }
  }
  catch (const char* message) 
  {
    "got it";
  }
  return i;
}
#endif
