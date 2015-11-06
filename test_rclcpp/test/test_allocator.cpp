// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "gtest/gtest.h"

#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/msg/u_int32.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

#ifndef WIN32

#include <execinfo.h>
#include <cxxabi.h>

#endif  // not WIN32

static bool test_init = false;

/** Check a demangled stack backtrace of the caller function for the given tokens. */
static inline bool check_stacktrace(const char ** tokens, size_t num_tokens, size_t max_frames = 6)
{
# ifndef WIN32
  bool match = false;

  // storage array for stack trace address data
  void * addrlist[max_frames + 1];

  // retrieve current stack addresses
  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void *));

  if (addrlen == 0) {
    fprintf(stderr, "WARNING: stack trace empty, possibly corrupt\n");
    return false;
  }

  // resolve addresses into strings containing "filename(function+address)",
  // this array must be free()-ed
  char ** symbollist = backtrace_symbols(addrlist, addrlen);

  // allocate string which will be filled with the demangled function name
  size_t funcnamesize = 256;
  char * funcname = static_cast<char *>(malloc(funcnamesize));

  // fprintf(stderr, ">>>> stack trace:\n");
  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < addrlen; i++) {
    if (match) {
      break;
    }
    char * begin_name = 0, * begin_offset = 0, * end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char * p = symbollist[i]; *p; ++p) {
      if (*p == '(') {
        begin_name = p;
      } else if (*p == '+') {
        begin_offset = p;
      } else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset &&
      begin_name < begin_offset)
    {
      *begin_name++ = '\0';
      *begin_offset++ = '\0';
      *end_offset = '\0';

      // mangled name is now in [begin_name, begin_offset) and caller
      // offset in [begin_offset, end_offset). now apply
      // __cxa_demangle():

      int status;
      char * ret = abi::__cxa_demangle(begin_name,
          funcname, &funcnamesize, &status);
      if (status == 0) {
        funcname = ret;  // use possibly realloc()-ed string
        for (size_t j = 0; j < num_tokens; ++j) {
          if (strstr(symbollist[i],
            tokens[j]) != nullptr || strstr(funcname, tokens[j]) != nullptr)
          {
            match = true;
            break;
          }
        }
        // fprintf(stderr, "  %s : %s+%s\n", symbollist[i], funcname, begin_offset);
      } else {
        // demangling failed. Output function name as a C function with
        // no arguments.
        for (size_t j = 0; j < num_tokens; j++) {
          if (strstr(symbollist[i],
            tokens[j]) != nullptr || strstr(begin_name, tokens[j]) != nullptr)
          {
            match = true;
            break;
          }
        }
        // fprintf(stderr, "  %s : %s()+%s\n", symbollist[i], begin_name, begin_offset);
      }
    } else {
      // couldn't parse the line? print the whole line.
      for (size_t j = 0; j < num_tokens; j++) {
        if (strstr(symbollist[i], tokens[j]) != nullptr) {
          match = true;
          break;
        }
      }
      // fprintf(stderr, "  %s\n", symbollist[i]);
    }
  }

  free(funcname);
  free(symbollist);
  if (!match) {
    fprintf(stderr, "WARNING: Called function did not match accepted tokens\n");
  }
  return match;
#else
  return true;
#endif  // not WIN32
}

// Override global new
static size_t global_allocs = 0;
static size_t global_runtime_allocs = 0;

static const size_t num_rmw_tokens = 5;
static const char * rmw_tokens[num_rmw_tokens] = {"librmw", "dds", "DDS", "dcps", "DCPS"};

void * operator new(std::size_t size)
{
  if (test_init) {
    // Check the stacktrace to see the call originated in librmw or a DDS implementation
    if (!check_stacktrace(rmw_tokens, num_rmw_tokens)) {
      global_runtime_allocs++;
    }
  } else {
    global_allocs++;
  }
  return std::malloc(size);
}

static size_t global_runtime_deallocs = 0;
static size_t global_deallocs = 0;

void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (test_init) {
      // Check the stacktrace to see the call originated in librmw or a DDS implementation
      if (!check_stacktrace(rmw_tokens, num_rmw_tokens)) {
        global_runtime_deallocs++;
      }
    } else {
      global_deallocs++;
    }
    std::free(ptr);
    ptr = nullptr;
  }
}

void operator delete(void * ptr, size_t) noexcept
{
  if (ptr != nullptr) {
    global_deallocs++;
    std::free(ptr);
    ptr = nullptr;
  }
}

static size_t num_allocs = 0;
static size_t num_deallocs = 0;

// Necessary for using custom allocator with std::basic_string in GCC 4.8
// See: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=56437
template<typename T>
struct allocator_pointer_traits
{
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;
  using reference = T &;
  using const_reference = const T &;
};

template<>
struct allocator_pointer_traits<void>
{
};

// Simple instrumented allocator
template<typename T>
struct InstrumentedAllocator : public allocator_pointer_traits<T>
{
public:
  using value_type = T;

  InstrumentedAllocator() noexcept
  {
  }

  ~InstrumentedAllocator() noexcept
  {
  }

  template<typename U>
  InstrumentedAllocator(const InstrumentedAllocator<U> &) noexcept
  {
  }

  // During execution, publish/subscribe should only malloc from here
  T * allocate(size_t size, const void * = 0)
  {
    if (size == 0) {
      return nullptr;
    }
    num_allocs++;
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }

  void deallocate(T * ptr, size_t size)
  {
    (void)size;
    if (!ptr) {
      return;
    }
    num_deallocs++;
    std::free(ptr);
  }

  // construct and destroy should not be required to implement allocator_traits,
  // this is a bug in gcc 4.8 for usage in std::pair, std::map
  template<typename U, typename ... Args,
  typename std::enable_if<!std::is_const<U>::value>::type * = nullptr>
  void
  construct(U * ptr, Args && ... args)
  {
    ::new(ptr)U(std::forward<Args>(args) ...);
  }

  template<typename U>
  void
  destroy(U * ptr)
  {
    ptr->~U();
  }

  // rebind should not be required to implement allocator_traits, this is a bug in gcc 4.8
  template<typename U>
  struct rebind
  {
    typedef InstrumentedAllocator<U> other;
  };
};

template<typename T, typename U>
constexpr bool operator==(const InstrumentedAllocator<T> &,
  const InstrumentedAllocator<U> &) noexcept
{
  return true;
}

template<typename T, typename U>
constexpr bool operator!=(const InstrumentedAllocator<T> &,
  const InstrumentedAllocator<U> &) noexcept
{
  return false;
}

const size_t iterations = 1;

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
using rclcpp::message_memory_strategy::MessageMemoryStrategy;

TEST(CLASSNAME(test_allocator, RMW_IMPLEMENTATION), shared_ptr) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_allocator_shared_ptr", false);

  size_t counter = 0;
  auto callback = [&counter](test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  auto alloc = std::make_shared<InstrumentedAllocator<void>>();
  auto msg_mem_strat =
    std::make_shared<MessageMemoryStrategy<test_rclcpp::msg::UInt32,
    InstrumentedAllocator<void>>>(alloc);
  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>("test_allocator_shared_ptr", 10,
      alloc);
  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_allocator_shared_ptr", 10, callback, nullptr, false, msg_mem_strat, alloc);
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<InstrumentedAllocator<void>>>(alloc);

  rclcpp::executors::SingleThreadedExecutor executor(memory_strategy);
  executor.add_node(node);

  // Create msg to be published
  auto msg = std::allocate_shared<test_rclcpp::msg::UInt32>(*alloc.get());

  size_t test_initialization_allocs = num_allocs;
  size_t test_initialization_deallocs = num_deallocs;

  rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
  // After test_initialization, global new should only be called from within InstrumentedAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    msg->data = i;
    publisher->publish(msg);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
    executor.spin_some();
  }
  test_init = false;

  size_t runtime_allocs = num_allocs - test_initialization_allocs;
  size_t runtime_deallocs = num_deallocs - test_initialization_deallocs;

  EXPECT_EQ(global_runtime_allocs, 0);
  EXPECT_EQ(global_runtime_deallocs, 0);

  printf("Global new was called outside of execution: %zu\n", global_allocs);
  printf("Global delete was called outside of execution: %zu\n", global_deallocs);
  printf("Allocator new was called: %zu\n", test_initialization_allocs);
  printf("Allocator delete was called: %zu\n", test_initialization_deallocs);

  printf("Calls to global new after spinning some: %zu\n", global_runtime_allocs);
  printf("Calls to global delete after spinning some: %zu\n", global_runtime_deallocs);
  printf("Calls to allocator new after spinning some: %zu\n", runtime_allocs);
  printf("Calls to allocator delete after spinning some: %zu\n", runtime_deallocs);

  num_allocs = 0;
  num_deallocs = 0;
  global_allocs = 0;
  global_deallocs = 0;
  global_runtime_allocs = 0;
  global_runtime_deallocs = 0;
  rclcpp::shutdown();
}

TEST(CLASSNAME(test_allocator, RMW_IMPLEMENTATION), unique_ptr) {
  using UInt32Allocator = InstrumentedAllocator<test_rclcpp::msg::UInt32>;
  using UInt32Deleter = rclcpp::allocator::Deleter<UInt32Allocator, test_rclcpp::msg::UInt32>;

  rclcpp::init(0, nullptr);
  auto context = rclcpp::contexts::default_context::get_global_default_context();
  auto intra_process_manager_state =
    std::make_shared<rclcpp::intra_process_manager::IntraProcessManagerState<UInt32Allocator>>();
  context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(
    intra_process_manager_state);
  // Use intra-process
  auto node = rclcpp::Node::make_shared("test_allocator_unique_ptr", context, true);

  size_t counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::UniquePtrWithDeleter<UInt32Deleter> msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  auto alloc = std::make_shared<UInt32Allocator>();
  auto msg_mem_strat =
    std::make_shared<MessageMemoryStrategy<test_rclcpp::msg::UInt32,
    UInt32Allocator>>(alloc);
  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>("test_allocator_unique_ptr", 10,
      alloc);
  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_allocator_unique_ptr", 10, callback, nullptr, false, msg_mem_strat, alloc);
  std::shared_ptr<rclcpp::memory_strategy::MemoryStrategy> memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<UInt32Allocator>>(alloc);

  rclcpp::executors::SingleThreadedExecutor executor(memory_strategy);
  executor.add_node(node);

  size_t test_initialization_allocs = num_allocs;
  size_t test_initialization_deallocs = num_deallocs;

  rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
  // After test_initialization, global new should only be called from within InstrumentedAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    auto msg = std::unique_ptr<test_rclcpp::msg::UInt32, UInt32Deleter>(
      std::allocator_traits<UInt32Allocator>::allocate(*alloc.get(), 1));
    msg->data = i;
    publisher->publish(msg);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
    executor.spin_some();
  }
  test_init = false;

  size_t runtime_allocs = num_allocs - test_initialization_allocs;
  size_t runtime_deallocs = num_deallocs - test_initialization_deallocs;

  EXPECT_EQ(global_runtime_allocs, 0);
  EXPECT_EQ(global_runtime_deallocs, 0);

  printf("Global new was called outside of execution: %zu\n", global_allocs);
  printf("Global delete was called outside of execution: %zu\n",
    global_deallocs);
  printf("Allocator new was called during initalization: %zu\n", test_initialization_allocs);
  printf("Allocator delete was called during initalization: %zu\n", test_initialization_deallocs);

  printf("Calls to global new after spinning some: %zu\n", global_runtime_allocs);
  printf("Calls to global delete after spinning some: %zu\n", global_runtime_deallocs);

  printf("Calls to allocator new after spinning some: %zu\n", runtime_allocs);
  printf("Calls to allocator delete after spinning some: %zu\n", runtime_deallocs);

  num_allocs = 0;
  num_deallocs = 0;
  global_allocs = 0;
  global_deallocs = 0;
  global_runtime_allocs = 0;
  global_runtime_deallocs = 0;
  rclcpp::shutdown();
}
