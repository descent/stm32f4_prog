#include "elf.h"
#include "type.h"
#include "fatfs/exfuns/exfuns.h"


#include <stdio.h>
#include <stdint.h>


void s32_memcpy(u8 *dest, const u8 *src, u32 n)
{
  for (int i=0; i < n ; ++i)
    *dest++ = *src++;
}

void print_packet(u8 *packet, u32 len)
{
  u32 i;

  printf("len: %d\n", len);

  for (i=0 ; i < len ; ++i)
  {
    if (i%16 == 0)
      printf("\n");

    printf("%02x ", packet[i]);
  }
  printf("\n");
}

#define BUF_SIZE 640*1024
#define LOAD_SIZE 64*1024
u8 buf[BUF_SIZE];
int load_elf()
{
  Elf32Ehdr elf_header = *((Elf32Ehdr*)buf);
  //Elf32Ehdr *elf_header = (Elf32Ehdr*)buf;
  printf("sizeof Elf32Ehdr: %d\n", sizeof(Elf32Ehdr));
  printf("elf_header.e_phoff: %d\n",  elf_header.e_phoff);

  Elf32Phdr elf_pheader = *((Elf32Phdr*)((u8 *)buf + elf_header.e_phoff));
  //Elf32Phdr *elf_pheader = ((Elf32Phdr*)((u8 *)buf + elf_header.e_phoff));
  printf("sizeof Elf32Phdr: %d\n", sizeof(Elf32Phdr));
  // the load elf address maybe overlay entry ~ elf code size, so save entry first
  u32 entry = elf_header.e_entry; 
  printf("entry: %#x\n", entry);
  printf("elf_header.e_phnum: %d\n", elf_header.e_phnum);

  for (int i=0 ; i < elf_header.e_phnum; ++i)
  {
    //printf("index: %d\n", i);
    if (elf_pheader.p_type == PT_LOAD)
    {
      //printf("offset: %#x size: %d\n", (elf_pheader->p_offset), elf_pheader->p_filesz);
      printf("p_vaddr: %#x offset: %#x size: %d\n", elf_pheader.p_vaddr, elf_pheader.p_offset, elf_pheader.p_filesz);
      u32 load_sector = elf_pheader.p_offset/512;
      u32 load_sector_num = elf_pheader.p_filesz/512 + 1;
      printf("load_sector: %d\n", load_sector);
      printf("load_sector_num: %d\n", load_sector_num);
      #if 0
      if (elf_pheader.p_offset >= LOAD_SIZE)
      {
        printf("exceed load size\n");
      }
      #endif
#ifdef TEST_MAIN
      print_packet(buf+elf_pheader.p_offset, elf_pheader.p_filesz);
#else
      s32_memcpy((u8*)elf_pheader->p_vaddr, buf, elf_pheader->p_filesz);
#endif
    }
    //elf_pheader = (Elf32Phdr*)((u8 *)buf + sizeof(Elf32Phdr) + elf_header.e_phoff);
    uintptr_t prog_header_offset = (uintptr_t)(i*(sizeof(Elf32Phdr)) + elf_header.e_phoff);
    //printf("prog_header_offset: %#x\n", prog_header_offset);
    elf_pheader = *((Elf32Phdr*)((u8 *)buf + prog_header_offset));
    //++elf_pheader;
  }
}



#ifdef TEST_MAIN
int main(int argc, char *argv[])
{
  u32 total,free;

  exfuns_init(); // 配置 fatfs 相關變數所使用的記憶體
  if (FR_INVALID_DRIVE == f_mount(fs[0],"0:",1)) // 掛載SD卡 
  {
    printf("f_mount fail\n");
    return -1;
  }

  while(exf_getfree("0",&total,&free))    //得到SD卡的總容量和剩餘容量
  {
  }

  printf("total: %d, free: %d\n", total, free);


#if 0
  FILE *fs;
  fs = fopen("/home/descent/git/jserv-course/stm32f4_prog/load_from_sd/loaded_prog/myur_168M.elf", "r");
  //fs = fopen("/home/descent/git/simple_compiler/c_parser", "r");
  fread(buf, 1, BUF_SIZE, fs);
  
  load_elf();
  fclose(fs);
#endif
  return 0;
}
#endif
