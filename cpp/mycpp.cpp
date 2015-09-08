#include "stm32.h"

#include "mem.h"

#include <vector>

int print(int i)
{
  i+=1;
}

int print(char c)
{
 c-=1;
}

int print(const char *str)
{
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
int _open(const char *pathname, int flags, int mode)
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

static char memarea[1024];

template <class T>
class my_allocator
{
public:
  typedef int    size_type;
  typedef int difference_type;
  typedef T*        pointer;
  typedef const T*  const_pointer;
  typedef T&        reference;
  typedef const T&  const_reference;
  typedef T         value_type;

  my_allocator() {}
  my_allocator(const my_allocator&) {}



  pointer   allocate(size_type n, const void * = 0) {
              T* t = (T*) mymalloc(n * sizeof(T));
              print("used my_allocator to allocate   at address ");
              print((int)t);
              print("\r\n");

              return t;
            }
  
  void      deallocate(void* p, size_type) {
              if (p) {
                myfree(p);
                print("used my_allocator to deallocate at address ");
                print((int)p);
                print("\r\n");
              } 
            }

  pointer           address(reference x) const { return &x; }
  const_pointer     address(const_reference x) const { return &x; }
  my_allocator<T>&  operator=(const my_allocator&) { return *this; }
  void              construct(pointer p, const T& val) 
                    { new ((T*) p) T(val); }
  void              destroy(pointer p) { p->~T(); }

  size_type         max_size() const { return int(-1); }

  template <class U>
  struct rebind { typedef my_allocator<U> other; };

  template <class U>
  my_allocator(const my_allocator<U>&) {}

  template <class U>
  my_allocator& operator=(const my_allocator<U>&) { return *this; }
};

void mymain(void)
{
  std::vector<char, my_allocator<char> > vec;
  for (int i=0 ; i < 5 ; ++i)
  {
    vec.push_back(i);
  }
#if 0
  char a, b, c , d, e, f;
  //std::vector<int, __gnu_cxx::new_allocator<int> > vec;
  std::vector<char, my_allocator<char> > vec;
  print(35);
  print('A');
  a = vec[0];
  b = vec[1];
  c = vec[2];
  d = vec[3];
  e = vec[4];

  char z=a+b+c+d;

  while(1);
#endif 
}

