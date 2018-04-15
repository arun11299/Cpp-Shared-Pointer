#include "shared_ptr.hpp"
#include <iostream>
#include <string>
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
    A(int) 
      : ptr(nullptr)
    {}
    arnml::shared_ptr<A> ptr;
  };

  arnml::shared_ptr<A> a1{0};
  a1->ptr = a1;
}

void weak_ptr_test()
{
  auto observe = [](arnml::weak_ptr<int> weak) {
    std::cout << "in fn cnt: " << weak.get_weak_ref_count() << std::endl;
    if (auto observ = weak.lock()) {
      std::cout << "observe locked to get shared ptr\n";
    } else {
      std::cout << "observe failed to get a shared ptr\n";
    }
  };

  (void)observe;

  arnml::weak_ptr<int> wp;
  observe(wp);
  std::cout << "cnt: " << wp.get_weak_ref_count() << std::endl;
  {
    arnml::shared_ptr<int> sp_int{42};
    wp = sp_int;
    std::cout << "weak ptr initialized with shared_ptr\n";
    std::cout << "cnt: " << wp.get_weak_ref_count() << std::endl;
    observe(wp);
    std::cout << "sp cnt: " << sp_int.strong_ref_count() << std::endl;
  }
  std::cout << "ap count final: " << wp.get_weak_ref_count() << std::endl;
}

void cyclic_dependency_with_weak_ptr()
{
  struct A
  {
    arnml::weak_ptr<A> ptr;
  };

  arnml::shared_ptr<A> a{};
  a->ptr = a;
}

void with_std()
{
  arnml::shared_ptr<std::string> sp_str{"Arun"};
  std::cout << sp_str->c_str() << std::endl;

  sp_str = arnml::shared_ptr<std::string>{4, 'a'};
  std::cout << sp_str->c_str() << std::endl;
}

int main() {
  /*
  copy_test();
  base_der_test();
  cyclic_ref_test();
  weak_ptr_test();
  cyclic_dependency_with_weak_ptr();
  */
  with_std();
  return 0;
}
