#include "shared_ptr.hpp"
#include <iostream>
#include <memory>

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

void base_der_test()
{
  struct Base
  {
    Base() { std::cout << "Base cons" << std::endl; }
    virtual void call() = 0;
    virtual ~Base() { std::cout << "Base dest" << std::endl; }
  };

  struct Derived: public Base
  {
    Derived(int): Base() { std::cout << "Derived cons" << std::endl; }
    void call() override { std::cout << "Derived::call" << std::endl; }
    ~Derived() { std::cout << "Derived dest" << std::endl; }
  };

  arnml::shared_ptr<Derived> ptr{42};
  Base* bp = ptr.get();
  bp->call();
}

void cyclic_ref_test()
{
  struct A {
    A(int) {}
    arnml::shared_ptr<A> ptr;
  };

  arnml::shared_ptr<A> a1{0};
  a1->ptr = a1;
}

int main() {
  copy_test();
  base_der_test();
  cyclic_ref_test();
  return 0;
}
