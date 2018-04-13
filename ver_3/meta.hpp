#ifndef META_HPP
#define META_HPP

#include <cassert>
#include <utility>
#include <type_traits>

namespace arnml  {
namespace detail {
namespace meta   {

  /**
   * The storage for the found type in a
   * template variadic param list.
   * Used internally in the implementation of
   * `get_value_of_type_impl`.
   */
  template <typename T>
  struct FindResult
  {
    // Called when entry not found
    constexpr FindResult(bool)
      : value(false)
    {
    }

    constexpr FindResult(T&& v)
      : type(std::forward<T>(v))
    {
    }

    T& get_type_value() noexcept { assert(value); return type; }

    // When `false`, accessing union fields is UB
    bool value;

    // Anonymous union to work with constexpr
    // Active field cannot be changed in constexpr context
    union {
      char buf[sizeof(T)];
      T type;
    };
  };

  template <typename OfType>
  constexpr auto get_value_of_type_impl()
  {
    return FindResult<OfType>{false};
  }
  
  template <typename OfType, typename F, typename... Rest,
            std::enable_if_t<std::is_same<std::remove_reference_t<F>, OfType>::value>* = nullptr
           >
  constexpr auto get_value_of_type_impl(F&& f, Rest&&...)
  {
    return FindResult<OfType>{std::forward<OfType>(f)};
  }

  template <typename OfType, typename Q, typename... Rest,
            std::enable_if_t<!std::is_same<std::remove_reference_t<Q>, OfType>::value>* = nullptr
           >
  constexpr auto get_value_of_type_impl(Q&&, Rest&&... rest)
  {
    return get_value_of_type_impl<OfType>(std::forward<Rest>(rest)...);
  }


  template <typename OfType, typename... Args>
  constexpr FindResult<OfType> get_value_of_type(Args&&... args)
  {
    return get_value_of_type_impl<OfType>(std::forward<Args>(args)...);
  }

} // END namespace meta
} // END namespace detail
} // END namespace arnml

#endif
