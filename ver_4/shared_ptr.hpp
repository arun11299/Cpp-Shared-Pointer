#ifndef SHARED_PTR_HPP
#define SHARED_PTR_HPP

#include <new>
#include <cstdint>
#include <cassert>
#include <utility>
#include <memory>
#include <type_traits>

//#include "./ver_4/meta.hpp"
#include "./ver_4/sp_type_traits.hpp"

namespace arnml {

// Reference Counting policies
struct single_threaded_t {};
struct multi_threaded_t {};

//Fwd decl weak_ptr
template <typename T, typename RCP> class weak_ptr;

namespace detail {

/*!
 * Base class for control block.
 *
 * TODO: I think I can use CRTP here (?)
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

  /// TODO: For atomic ref count policy, these APIS might
  // be not sufficient

  /// Add to the strong ref count
  virtual void add_ref()   = 0;
  /// Get the strong ref count
  virtual size_t get_ref() const = 0;
  /// Subtract from the strong ref count
  virtual size_t dec_ref() = 0;

  /// Add to the weak ref count
  virtual void add_weak_ref() = 0;
  /// Get the weak ref count
  virtual size_t get_weak_ref() const = 0;
  /// Subtract from the weak ref count
  virtual size_t dec_weak_ref() = 0;

  /// Destroy complete control block
  virtual void destruct_full() noexcept = 0;
  /// Destroy the object only by calling its destructor
  virtual void destruct_object() noexcept = 0;
  /// Destroy only the control block by deallocating
  virtual void destruct_ctrl_blk() noexcept = 0;

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
 *
 * TODO: Use EBO for Allocator.
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
  /// Weak ref count
  size_t weak_ref_cnt_ = 0;

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
   * Increment the weal ref count
   */
  void add_weak_ref() override
  {
    weak_ref_cnt_++;
  }

  /**
   * Returns the current weak ref count value.
   */
  size_t get_weak_ref() const override
  {
    return weak_ref_cnt_;
  }

  /**
   * Decrement the weak ref count
   */
  size_t dec_weak_ref() override
  {
    assert (weak_ref_cnt_ > 0);
    return (--weak_ref_cnt_);
  }

  /**
   * Destroys both the object and the control object
   */
  void destruct_full() noexcept override
  {
    this->~control_block_base<T>();
    alloc_.deallocate(this, 1);
  }

  /**
   * Calls the destrutor of the embedded object
   */
  void destruct_object() noexcept override
  {
    control_block_base<T>::get_type_ptr()->~T();
  }

  /**
   * Destroy the control block
   */
  void destruct_ctrl_blk() noexcept override
  {
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
 * Default construct the object in its aligned storage area.
 */
template <typename T>
void construct_wrapped_type(control_block_base<T>* cb)
{
  static_assert(
      std::is_default_constructible<T>::value,
      "Type T is not default constructible");

  new (&cb->value_buf) T;
  cb->add_ref();
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
      std::is_constructible<T, Args...>::value,
      "Type T cannot be constructed or convertible with provided Args...");

  //ATTN: Using braces to construct causes some wierd issues while
  //compiling with Clang. It considers it as initializer-list!
  new (&cb->value_buf) T(std::forward<Args>(args)...);
  cb->add_ref();
}

} // END namespace detail


/**
 * Shared Pointer.
 */
template <typename T, typename RefCntPolicy=single_threaded_t>
class shared_ptr
{
public: // typedefs
  ///
  using value_type = T;
  ///
  using ref_cnt_policy = RefCntPolicy;

  // To have access to the control block
  friend class weak_ptr<T, RefCntPolicy>;

public:
  /**
   * The default constructor.
   */
  explicit shared_ptr(std::nullptr_t)
  {
    //...
  }

  /**
   * For default constructible types
   */
  explicit shared_ptr()
    : shared_ptr(std::allocator<T>{})
  {
    //...
  }

  /**
   * Forwarding constructor.
   * Create a shared pointer from the Args.
   */
  template <typename First, typename... Args,
            typename Allocator=std::allocator<T>,
            typename Cond = std::enable_if_t<
                              !std::is_same<std::remove_reference_t<T>, shared_ptr>::value &&
                              arnml::detail::is_allocator<Allocator>::value &&
                              !arnml::detail::is_allocator<First>::value
                            >
           >
  explicit shared_ptr(First&& f, Args&&... args)
    : shared_ptr(Allocator{}, std::forward<First>(f), std::forward<Args>(args)...)
  {
    //....
  }

  /**
   * Constructor for default constructible types.
   */
  template <typename Allocator=std::allocator<T>,
            typename = std::enable_if_t<detail::is_allocator<Allocator>::value>
           >
  explicit shared_ptr(Allocator&& alloc=Allocator{});

  /**
   */
  template <typename... Args, typename Allocator=std::allocator<T>,
           typename Cond = std::enable_if_t<
                              !std::is_same<std::remove_reference_t<T>, shared_ptr>::value  &&
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

    return *this;
  }

  /**
   * Move Assignment
   *
   * NOTE: Self move not supported.
   */
  shared_ptr& operator=(shared_ptr&& other)
  {
    assert (other.ctrl_blk_ != ctrl_blk_);
    this->~shared_ptr();
    ctrl_blk_ = other.ctrl_blk_;
    other.ctrl_blk_ = nullptr;

    return *this;
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

  /**
   * operator bool
   */
  explicit operator bool() const noexcept
  {
    return ctrl_blk_;
  }

public: // Reference cout APIs
  /**
   */
  size_t strong_ref_count() const noexcept
  {
    return ctrl_blk_->get_ref();
  }

  /**
   */
  size_t weak_ref_count() const noexcept
  {
    return ctrl_blk_->get_weak_ref();
  }

private:
  /// Construct shared_ptr from the control block
  /// Used by weak_ptr::lock
  explicit shared_ptr(detail::control_block_base<T>* cb)
    : ctrl_blk_(cb)
  {
    // New instance means, update the strong count
    if (ctrl_blk_) ctrl_blk_->add_ref();
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

  // If weak pointer count is not zero, we do not
  // want to destruct the control block.
  // The control block has the counts. Only call
  // the type destructor.
  //
  // ATTN: Decrement the strong ref count after calling the 
  // object destructor because the object might be having
  // a weak pointer to this shared ptr and might end up deleting
  // the control block if we decrement the ref count before it.
  if (ctrl_blk_->get_ref() == 1) {
    ctrl_blk_->destruct_object();

    ctrl_blk_->dec_ref();

    if (ctrl_blk_->get_weak_ref() == 0) {
      ctrl_blk_->destruct_ctrl_blk();
      ctrl_blk_ = nullptr;
    }
  } else {
    ctrl_blk_->dec_ref();
  }
}

//-------------------------------------------------------------------------------
//

/**
 * Weak Pointer.
 */
template <typename T, typename RCP=single_threaded_t>
class weak_ptr
{
public: // typedefs
  ///
  using value_type = T;
  ///
  using ref_cnt_policy = RCP;

public:
  /**
   * Default constructor.
   */
  explicit weak_ptr()
  {
    //...
  }

  /**
   * Constructor taking in reference to 
   * an instance of shared_ptr.
   */
  explicit weak_ptr(shared_ptr<T, RCP>& sp)
    : ctrl_blk_(sp.ctrl_blk_)
  {
    ctrl_blk_->add_weak_ref();
  }

  /// Copy contructor
  weak_ptr(const weak_ptr& other)
    : ctrl_blk_(other.ctrl_blk_)
  {
    if (ctrl_blk_) ctrl_blk_->add_weak_ref();
  }

  /// Move copy contructor
  weak_ptr(weak_ptr&& other)
    : ctrl_blk_(other.ctrl_blk_)
  {
    other.ctrl_blk_ = nullptr;
    // Ref count remains the same
  }

  /// Assignment operator
  weak_ptr& operator=(const weak_ptr& other)
  {
    if (ctrl_blk_) {
      ctrl_blk_->dec_weak_ref();
      checked_destroy_control_block();
    }

    ctrl_blk_ = other.ctrl_blk_;
    ctrl_blk_->add_weak_ref();

    return *this;
  }

  /// Move assignement operator
  weak_ptr& operator=(weak_ptr&& other)
  {
    if (ctrl_blk_) {
      ctrl_blk_->dec_weak_ref();
      checked_destroy_control_block();
    }

    ctrl_blk_ = other.ctrl_blk_;
    other.ctrl_blk_= nullptr;

    return *this;
  }

  /// Assignment operator to shared_ptr
  weak_ptr& operator=(shared_ptr<T, RCP>& sp)
  {
    ctrl_blk_ = sp.ctrl_blk_;

    if (ctrl_blk_) ctrl_blk_->add_weak_ref();

    return *this;
  }

  /**
   * The destroyer
   */
  ~weak_ptr()
  {
    if (ctrl_blk_) {
      ctrl_blk_->dec_weak_ref();

      // Destroy the control block if there
      // are no more stron reference and the 
      // weak count has dropped to zero.
      if (ctrl_blk_->get_ref() == 0) {
        if (ctrl_blk_->get_weak_ref() == 0) {
          ctrl_blk_->destruct_ctrl_blk();
        }
      }
    }
  }

public:
  /**
   * Create a shared_ptr therebey creating a strong 
   * reference to the object or more precisely the control block.
   */
  shared_ptr<T, RCP> lock()
  {
    if (!ctrl_blk_ || !ctrl_blk_->get_ref()) return shared_ptr<T, RCP>{nullptr};
    return shared_ptr<T, RCP>{ctrl_blk_};
  }

  /// bool operator
  explicit operator bool() const noexcept
  {
    return ctrl_blk_;
  }

  ///get weak ref count
  size_t get_weak_ref_count() const noexcept
  {
    if (!ctrl_blk_) return 0;
    return ctrl_blk_->get_weak_ref();
  }

private:
  /// Check if the control blockj can be destroyed or not
  // If it can, the call the destroy function
  void checked_destroy_control_block()
  {
    if (!ctrl_blk_->get_ref() && !ctrl_blk_->get_weak_ref()) {
      ctrl_blk_->destruct();
    }
  }


private:
  // Control block managed by the shared_ptr instance
  detail::control_block_base<T>* ctrl_blk_ = nullptr;
};


} // END namespace arnml

#endif
