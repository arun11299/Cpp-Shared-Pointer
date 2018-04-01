#include "shared_ptr.hpp"
#include <iostream>

struct Data {
  Data(int v): d(v) {}
  int d = 0;
};

void copy_test_int(arnml::shared_ptr<Data> sp)
{
  assert (sp.strong_ref_count() == 3);
  sp->d += 1;
}

void copy_test()
{
  arnml::shared_ptr<Data> sp{0};
  {
    auto copy_sp = sp;
    assert (sp.strong_ref_count() == copy_sp.strong_ref_count());
    assert (copy_sp.strong_ref_count() == 2);
    copy_test_int(copy_sp);
    assert (sp.strong_ref_count() == 2);
  }
  assert (sp.strong_ref_count() == 1);
}

int main() {
  copy_test();
  return 0;
}
