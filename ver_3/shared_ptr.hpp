#ifndef SHARED_PTR_HPP
#define SHARED_PTR_HPP

#include <new>
#include <cstdint>
#include <cassert>
#include <utility>
#include <memory>
#include <type_traits>

//#include "./ver_3/meta.hpp"
#include "./ver_3/sp_type_traits.hpp"

namespace arnml {

// Reference Counting policies
struct single_threaded_t {};
struct multi_threaded_t {};

namespace detail {

/*!
 */
template <typename T>
struct control_block_base
{
  using storage_type = typename std::aligned_storage<sizeof(T), alignof(T)>::type;
  /// The storage buffer to create the type `T`
  storage_type value_buf;

  /// Create a control object derived impl
  template <typename Allocator, typename RefCntPolicy>
  static control_block_base* create(Allocator&& allocator);

  /**
   * Calls destructor of the embedded object.
   */
  virtual ~control_block_base()
  {
    (reinterpret_cast<T*>(&value_buf))->~T();
  }

  /**
   * Get the actual mutable pointer type of the object residing
   * in the storage area.
   */
  T* get_type_ptr()
  {
    return reinterpret_cast<T*>(&value_buf);
  }

  /**
   * Get the actual const pointer type of the object residing
   * in the storage area.
   */
  const T* get_const_type_ptr()
  {
    return const_cast<const T*>(reinterpret_cast<T*>(&value_buf));
  }

  ///
  virtual void add_ref()   = 0;
  ///
  virtual size_t get_ref() const = 0;
  ///
  virtual size_t dec_ref() = 0;
  ///
  virtual void destruct() noexcept = 0;

};

/*!
 * The control block object where the 
 * allocated object resides along with
 * its reference count.
 */
template <
  // The allocated object type.
  typename T,
  // The allocator for the control object
  typename Allocator,
  // The ref count policy to be used.
  // Single threaded Policy or Multi Threaded Policy
  typename RefCntPolicy
>
struct control_block;

/*!
 * Control Block specialized for Single Thread.
 */
template <typename T, typename Allocator>
struct control_block<T, Allocator, single_threaded_t>
  : control_block_base<T>
{
  ///
  using allocator_type = typename std::allocator_traits<
    std::decay_t<Allocator>>::template rebind_alloc<control_block>;
  
  /// Reference counter for single threaded applications
  size_t ref_cnt_ = 0;

  /**
   * Constructor.
   */
  control_block(Allocator&& allocator)
    : control_block_base<T>()
    , alloc_(std::forward<Allocator>(allocator))
  {
  }

  /**
   * Increments the ref count.
   */
  void add_ref() override
  {
    ref_cnt_++;
  }

  /**
   * Return the current ref count for the object.
   */
  size_t get_ref() const override
  {
    return ref_cnt_;
  }

  /**
   * Decrements the ref count.
   */
  size_t dec_ref() override
  {
    assert (ref_cnt_ > 0);
    return (--ref_cnt_);
  }

  /**
   */
  void destruct() noexcept override
  {
    this->~control_block_base<T>();
    alloc_.deallocate(this, 1);
  }

  /**
   */
  ~control_block()
  {
  }

  allocator_type alloc_;
};


/**
 * Construct the control block.
 */
template <typename T>
template <typename Allocator, typename RCP>
control_block_base<T>*
control_block_base<T>::create(
    Allocator&& allocator)
{
  using DerSelf = typename detail::control_block<
                                      T, 
                                      std::decay_t<Allocator>,
                                      RCP 
                                      >;

  typename DerSelf::allocator_type rb_alloc;
 
  auto address = static_cast<DerSelf*>(rb_alloc.allocate(1));
  auto ctrl_blk = new (address) DerSelf{std::forward<Allocator>(allocator)};

  return static_cast<control_block_base<T>*>(ctrl_blk);
}


/**
 * Construct the object in its aligned storage area.
 */
template <typename T, typename... Args>
void construct_wrapped_type(
    control_block_base<T>* cb,
    Args&&... args)
{
  static_assert(
      std::is_constructible<T, Args...>::value ||
      std::is_convertible<T, Args...>::value,
      "Type T cannot be constructed or convertible with provided Args...");

  new (&cb->value_buf) T{std::forward<Args>(args)...};
  cb->add_ref();
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
   * Constructor for default constructible types.
   */
  template <typename Allocator=std::allocator<T>>
  explicit shared_ptr(Allocator&& a = Allocator{})
    : shared_ptr(std::forward<Allocator>(a))
  {
    //...
  }

  /**
   * Forwarding constructor.
   * Create a shared pointer from the Args.
   */
  template <typename... Args,
            typename Allocator=std::allocator<T>,
            typename Cond = std::enable_if_t<
                              !std::is_same<std::remove_reference_t<T>, shared_ptr>::value ||
                              arnml::detail::is_allocator<Allocator>::value
                            >
           >
  explicit shared_ptr(Args&&... args)
    : shared_ptr(Allocator{}, std::forward<Args>(args)...)
  {
    //....
  }

  /**
   */
  template <typename Allocator=std::allocator<T>,
            typename = std::enable_if_t<detail::is_allocator<Allocator>::value>>
  explicit shared_ptr(Allocator&& alloc);

  /**
   */
  template <typename... Args, typename Allocator=std::allocator<T>,
           typename Cond = std::enable_if_t<
                              !std::is_same<std::remove_reference_t<T>, shared_ptr>::value ||
                              arnml::detail::is_allocator<Allocator>::value>>
  explicit shared_ptr(Allocator&& alloc, Args&&... args);
    

  /**
   * Regular copy constructor.
   */
  shared_ptr(const shared_ptr& other)
    : ctrl_blk_(other.ctrl_blk_)
  {
    ctrl_blk_->add_ref();
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
    ctrl_blk_->add_ref();
  }

  /**
   * Destructor.
   */
  ~shared_ptr();

public: // Pointer like semantics
  /**
   */
  T* get() noexcept
  {
    return ctrl_blk_->get_type_ptr();
  }

  const T* get() const noexcept
  {
    return ctrl_blk_->get_const_type_ptr();
  }

  /**
   */
  T* operator->() noexcept
  {
    return ctrl_blk_->get_type_ptr();
  }

  /**
   */
  const T* operator->() const noexcept
  {
    return ctrl_blk_->get_const_type_ptr();
  }

  /**
   */
  T& operator*() noexcept
  {
    return *(ctrl_blk_->get_type_ptr());
  }

  /**
   */
  const T& operator*() const noexcept
  {
    return *(ctrl_blk_->get_const_type_ptr());
  }

public: // Reference cout APIs
  /**
   */
  size_t strong_ref_count() const noexcept
  {
    return ctrl_blk_->get_ref();
  }

private:
  /// The control block where the pointed type actually lives
  detail::control_block_base<T>* ctrl_blk_ = nullptr;
};

//-----------------------------------------------------------------

template <typename T, typename RCP>
template <typename Allocator,
          typename Cond>
shared_ptr<T, RCP>::shared_ptr(Allocator&& alloc)
{
  ctrl_blk_ = detail::control_block_base<T>::template create<Allocator, RCP>(
                              std::forward<Allocator>(alloc));

  // Construct the object and increment ref count
  detail::construct_wrapped_type(ctrl_blk_);
}


template <typename T, typename RCP>
template <typename... Args, typename Allocator,
          typename Cond>
shared_ptr<T, RCP>::shared_ptr(Allocator&& alloc, Args&&... args)
{
  ctrl_blk_ = detail::control_block_base<T>::template create<Allocator, RCP>(
                              std::forward<Allocator>(alloc));

  // Construct the object and increment ref count
  detail::construct_wrapped_type(
                    ctrl_blk_,
                    std::forward<Args>(args)...);
}


template <typename T, typename RCP>
shared_ptr<T, RCP>::~shared_ptr()
{
  // moved from? not in control any more
  if (!ctrl_blk_) return;

  auto ref_cnt = ctrl_blk_->dec_ref();
  if (ref_cnt == 0) {
    ctrl_blk_->destruct();
  }
}


} // END namespace arnml

#endif
