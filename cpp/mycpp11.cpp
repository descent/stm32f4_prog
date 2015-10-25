#include "stm32.h"

int print(int i)
{
  i+=1;
}

int print(char c)
{
 c-=1;
}

int lambda_test(int girls = 3, int boys = 4)
{
  auto totalChild = [](int x, int y) ->int{return x+y;};
  return totalChild(girls, boys);
}


void mymain(void)
{
  constexpr int d=1;
  int x=3;
  int y=9;

  char path[]=R"(/usr/local/aaa)";
  char wpath[]=R"("\usr\local\aaa")";

  int total = lambda_test(x, y);
  print(35);
  print('A');
  int arr[]={1,2,3,4,5};
  for (int &e : arr)
  {
    print(e);
  }
  while(1);
}
