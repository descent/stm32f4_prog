// ref: http://docs.roguewave.com/sourcepro/11.1/html/toolsug/11-6.html
// I modify something

//#include <rw/tvdlist.h>
//#include <rw/cstring.h>
#include <iostream>
#include <vector>

#include <cstdlib>


using namespace std;

template <class T>
class my_allocator
{
public:
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T*        pointer;
  typedef const T*  const_pointer;
  typedef T&        reference;
  typedef const T&  const_reference;
  typedef T         value_type;

  my_allocator() {}
  my_allocator(const my_allocator&) {}



  pointer   allocate(size_type n, const void * = 0) {
              T* t = (T*) malloc(n * sizeof(T));
              std::cout
              << "  used my_allocator to allocate   at address "
              << t << " (+)" << std::endl;
              return t;
            }
  
  void      deallocate(void* p, size_type) {
              if (p) {
                free(p);
                std::cout
                << "  used my_allocator to deallocate at address "
                << p << " (-)" << 
                std::endl;
              } 
            }

  pointer           address(reference x) const { return &x; }
  const_pointer     address(const_reference x) const { return &x; }
  my_allocator<T>&  operator=(const my_allocator&) { return *this; }
  void              construct(pointer p, const T& val) 
                    { new ((T*) p) T(val); }
  void              destroy(pointer p) { p->~T(); }

  size_type         max_size() const { return size_t(-1); }

  template <class U>
  struct rebind { typedef my_allocator<U> other; };

  template <class U>
  my_allocator(const my_allocator<U>&) {}

  template <class U>
  my_allocator& operator=(const my_allocator<U>&) { return *this; }
};

int main()
{
  const int numItems = 100;
  std::cout << "\nCreating a RWTValDlist with a default allocator"
            << std::endl;

  vector<int> vec;

  for (int i = 0; i < 5; ++i) 
  {
    vec.push_back(i);
  }

  for (int i=0 ; i < 5 ; ++i)
    cout << vec[i] << endl;

  vector<int, my_allocator<int> > custom_vec;


  for (int i = 0; i < 5; ++i) 
  {
    custom_vec.push_back(i);
  }

  for (int i=0 ; i < 5 ; ++i)
    cout << custom_vec[i] << endl;
  
  return 0;
}
