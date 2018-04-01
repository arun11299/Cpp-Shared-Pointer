#ifndef SHARED_PTR_HPP
#define SHARED_PTR_HPP

#include <new>
#include <cstdint>
#include <cassert>
#include <utility>
#include <type_traits>

namespace arnml {

// Reference Counting policies
struct single_threaded_t {};
struct multi_threaded_t {};

namespace detail {

/*!
 * The control block object where the 
 * allocated object resides along with
 * its reference count.
 */
template <
  // The allocated object type.
  typename T,
  // The ref count policy to be used.
  // Single threaded Policy or Multi Threaded Policy
  typename RefCntPolicy>
struct control_block;

/*!
 * Control Block specialized for Single Thread.
 */
template <typename T>
struct control_block<T, single_threaded_t>
{
  using storage_type = typename std::aligned_storage<sizeof(T), alignof(T)>::type;

  storage_type value_buf;
  size_t ref_cnt = 0;

  /**
   * Calls destructor of the embedded object.
   */
  ~control_block()
  {
    (reinterpret_cast<T*>(&value_buf))->~T();
  }
};

/**
 * Construct the object in its aligned storage area.
 */
template <typename T, typename... Args>
void construct(
    control_block<T, single_threaded_t>* cb,
    Args&&... args)
{
  new (&cb->value_buf) T{std::forward<Args>(args)...};
}

/**
 * Return the current ref count for the object.
 */
template <typename T>
size_t get_strong_ref_count(const control_block<T, single_threaded_t>* cb)
{
  return cb->ref_cnt;
}

/**
 * Increments the ref count.
 */
template <typename T>
void add_ref(control_block<T, single_threaded_t>* cb)
{
  cb->ref_cnt++;
}

/**
 * Decrements the ref count.
 */
template <typename T>
size_t decr_ref(control_block<T, single_threaded_t>* cb)
{
  assert (cb->ref_cnt > 0);
  return (--cb->ref_cnt);
}

/**
 * Get the actual mutable pointer type of the object residing
 * in the storage area.
 */
template <typename T>
T* get_type_ptr(control_block<T, single_threaded_t>* cb)
{
  return reinterpret_cast<T*>(&cb->value_buf);
}

/**
 * Get the actual const pointer type of the object residing
 * in the storage area.
 */
template <typename T>
const T* get_const_type_ptr(control_block<T, single_threaded_t>* cb)
{
  return const_cast<const T*>(reinterpret_cast<T*>(&cb->value_buf));
}

} // END namespace detail


/**
 * Shared Pointer.
 */
template <typename T, typename RefCntPolicy=single_threaded_t>
class shared_ptr
{
public:
  /**
   * Create a shared pointer from the Args.
   */
  template <typename... Args, 
           typename Cond = std::enable_if_t<
                             std::is_constructible<T, Args...>::value ||
                             std::is_convertible<T, Args...>::value>>
  explicit shared_ptr(Args&&... args);

  /**
   * Regular copy constructor.
   */
  shared_ptr(const shared_ptr& other)
    : ctrl_blk_(other.ctrl_blk_)
  {
    detail::add_ref(ctrl_blk_);
  }

  /**
   * Move copy constructor.
   */
  shared_ptr(shared_ptr&& other)
    : ctrl_blk_(other.ctrl_blk_)
  {
    other.ctrl_blk_ = nullptr;
  }

  /**
   * Assignment.
   * Destroy current control block/ decrse ref count.
   *
   * NOTE: Self assignment not supported.
   */
  shared_ptr& operator=(const shared_ptr& other)
  {
    //TODO: what to do ?
    assert (other.ctrl_blk_ != ctrl_blk_);

    this->~shared_ptr();

    ctrl_blk_ = other.ctrl_blk_;
    detail::add_ref(ctrl_blk_);
  }

  /**
   * Destructor.
   */
  ~shared_ptr();

public: // Pointer like semantics
  /**
   */
  T* operator->() noexcept
  {
    return detail::get_type_ptr(ctrl_blk_);
  }

  /**
   */
  const T* operator->() const noexcept
  {
    return detail::get_const_type_ptr(ctrl_blk_);
  }

  /**
   */
  T& operator*() noexcept
  {
    return *(detail::get_type_ptr(ctrl_blk_));
  }

  /**
   */
  const T& operator*() const noexcept
  {
    return *(detail::get_const_type_ptr(ctrl_blk_));
  }

public: // Reference cout APIs
  /**
   */
  size_t strong_ref_count() const noexcept
  {
    return detail::get_strong_ref_count(ctrl_blk_);
  }

private:
  /// The control block where the pointed type actually lives
  detail::control_block<T, RefCntPolicy>* ctrl_blk_ = nullptr;
};

//-----------------------------------------------------------------

template <typename T, typename RCP>
template <typename... Args, typename Cond>
shared_ptr<T, RCP>::shared_ptr(Args&&... args)
{
  ctrl_blk_ = new detail::control_block<T, RCP>{};

  // Construct the object and increment ref count
  detail::construct(ctrl_blk_, std::forward<Args>(args)...);
  detail::add_ref(ctrl_blk_);
}

template <typename T, typename RCP>
shared_ptr<T, RCP>::~shared_ptr()
{
  // moved from? not in control any more
  if (!ctrl_blk_) return;

  auto ref_cnt = detail::decr_ref(ctrl_blk_);
  if (ref_cnt == 0) {
    delete ctrl_blk_;
  }
}


} // END namespace arnml

#endif
