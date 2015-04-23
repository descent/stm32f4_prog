#include "stm32.h"

#include <vector>

int print(int i)
{
  i+=1;
}

int print(char c)
{
 c-=1;
}



void *dso_handle_;
void *__dso_handle;

extern "C" void _exit()
{
}

char brk_area[1024];

extern "C" char *_sbrk(char *increment)
{
  char *ptr = brk_area;
  return ptr;
}

extern "C"
int _kill(int a, int b)
{
  return a;
}

extern "C"
int _getpid()
{
  int i;
  return i;
}

extern "C"
int _write(int fd, const void *buf, int count)
{
}

extern "C"
int open(const char *pathname, int flags, int mode)
{
}

extern "C"
int _isatty(int fd)
{
}


extern "C"
int _close(int fd)
{
}

extern "C"
int _fstat(int fd, struct stat *buf)
{
}

extern "C"
int _read(int fd, void *buf, int count)
{
}

extern "C"
int _lseek(int fd, int offset, int whence)
{
}


void mymain(void)
{
#if 0
  std::vector<int> vec;
  vec.push_back(1);
  print(35);
  print('A');
  while(1);
#endif 
}

