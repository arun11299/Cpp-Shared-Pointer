#ifndef SP_TYPE_TRAITS_HPP
#define SP_TYPE_TRAITS_HPP

#include <utility>
#include <type_traits>

namespace arnml {
namespace detail {

///
template <typename... T>
using void_t = void;

template <typename T, typename=void>
struct is_complete: std::false_type
{};

template <typename T>
struct is_complete <T, decltype(void(sizeof(T)))>: std::true_type
{};

template <typename T, typename=void>
struct is_allocator: public std::false_type
{
};

template <typename T>
struct is_allocator<T, void_t<
                        decltype(std::declval<T>().allocate(0)),
                        decltype(std::declval<T>().deallocate(nullptr, 0))
                       >
                   >
                   : public std::true_type
{
};

} // END namespace arnml
} // END namespace arnml

#endif
