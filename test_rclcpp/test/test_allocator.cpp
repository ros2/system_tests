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
#include <string>
#include <vector>
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

// TODO(jacquelinekay) improve this ignore rule (dogfooding or no allocations)
static const size_t num_rmw_tokens = 5;
static const char * rmw_tokens[num_rmw_tokens] = {"librmw", "dds", "DDS", "dcps", "DCPS"};

static const size_t iterations = 1;

static bool verbose = false;
static bool ignore_middleware_tokens = true;
static bool test_init = false;
static bool fail = false;

/** Check a demangled stack backtrace of the caller function for the given tokens.
 ** Adapted from: https://panthema.net/2008/0901-stacktrace-demangled
 **/
inline bool check_stacktrace(const char ** tokens, size_t num_tokens, size_t max_frames = 6)
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

  if (verbose) {
    fprintf(stderr, ">>>> stack trace:\n");
  }

  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < addrlen; i++) {
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
        if (verbose) {
          fprintf(stderr, "  %s : %s+%s\n", symbollist[i], funcname, begin_offset);
        }
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
        if (verbose) {
          fprintf(stderr, "  %s : %s()+%s\n", symbollist[i], begin_name, begin_offset);
        }
      }
    } else {
      // couldn't parse the line? print the whole line.
      for (size_t j = 0; j < num_tokens; j++) {
        if (strstr(symbollist[i], tokens[j]) != nullptr) {
          match = true;
          break;
        }
      }
      if (verbose) {
        fprintf(stderr, "  %s\n", symbollist[i]);
      }
    }
  }

  free(funcname);
  free(symbollist);
  if (!ignore_middleware_tokens) {
    return false;
  }
  return match;
#else
  return true;
#endif  // not WIN32
}

void * operator new(std::size_t size)
{
  if (test_init) {
    // Check the stacktrace to see the call originated in librmw or a DDS implementation
    fail |= !check_stacktrace(rmw_tokens, num_rmw_tokens);
  }
  return std::malloc(size);
}

void operator delete(void * ptr) noexcept
{
  if (ptr != nullptr) {
    if (test_init) {
      // Check the stacktrace to see the call originated in librmw or a DDS implementation
      fail |= !check_stacktrace(rmw_tokens, num_rmw_tokens);
    }
    std::free(ptr);
    ptr = nullptr;
  }
}

void operator delete(void * ptr, size_t) noexcept
{
  if (ptr != nullptr) {
    if (test_init) {
      // Check the stacktrace to see the call originated in librmw or a DDS implementation
      EXPECT_TRUE(check_stacktrace(rmw_tokens, num_rmw_tokens));
    }
    std::free(ptr);
    ptr = nullptr;
  }
}


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
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }

  void deallocate(T * ptr, size_t size)
  {
    (void)size;
    if (!ptr) {
      return;
    }
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


using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

class AllocatorTest : public::testing::Test
{
protected:
  std::string test_name_;

  rclcpp::node::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;
  rclcpp::publisher::Publisher<test_rclcpp::msg::UInt32,
  InstrumentedAllocator<void>>::SharedPtr publisher_;
  rclcpp::message_memory_strategy::MessageMemoryStrategy<test_rclcpp::msg::UInt32,
  InstrumentedAllocator<void>>::SharedPtr msg_memory_strategy_;
  std::shared_ptr<InstrumentedAllocator<void>> alloc;

  bool intra_process_;

  using UInt32Allocator = InstrumentedAllocator<test_rclcpp::msg::UInt32>;
  using UInt32Deleter = rclcpp::allocator::Deleter<UInt32Allocator, test_rclcpp::msg::UInt32>;

  void initialize(bool intra_process, const std::string & name)
  {
    test_name_ = name;
    intra_process_ = intra_process;

    auto context = rclcpp::contexts::default_context::get_global_default_context();
    auto intra_process_manager_state =
      std::make_shared<rclcpp::intra_process_manager::IntraProcessManagerImpl<UInt32Allocator>>();
    context->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>(
      intra_process_manager_state);

    node_ = rclcpp::Node::make_shared(name, context, intra_process);
    alloc = std::make_shared<InstrumentedAllocator<void>>();
    msg_memory_strategy_ =
      std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy
      <test_rclcpp::msg::UInt32,
      InstrumentedAllocator<void>>>(alloc);
    publisher_ = node_->create_publisher<test_rclcpp::msg::UInt32>(name, 10, alloc);
    memory_strategy_ =
      std::make_shared<AllocatorMemoryStrategy<InstrumentedAllocator<void>>>(alloc);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(memory_strategy_);

    executor_->add_node(node_);
  }

  AllocatorTest()
  {
  }

  ~AllocatorTest()
  {
  }
};


TEST_F(AllocatorTest, allocator_shared_ptr) {
  initialize(false, "allocator_shared_ptr");
  size_t counter = 0;
  auto callback = [&counter](test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  auto subscriber = node_->create_subscription<test_rclcpp::msg::UInt32>(
    "allocator_shared_ptr", 10, callback, nullptr, false, msg_memory_strategy_, alloc);
  // Create msg to be published
  auto msg = std::allocate_shared<test_rclcpp::msg::UInt32>(*alloc.get());

  rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
  // After test_initialization, global new should only be called from within InstrumentedAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    msg->data = i;
    publisher_->publish(msg);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
    executor_->spin_some();
  }
  test_init = false;
  EXPECT_FALSE(fail);
  fail = false;
}

TEST_F(AllocatorTest, allocator_unique_ptr) {
  initialize(true, "allocator_unique_ptr");
  size_t counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::UniquePtrWithDeleter<UInt32Deleter> msg) -> void
    {
      EXPECT_EQ(counter, msg->data);
      counter++;
    };

  auto subscriber = node_->create_subscription<test_rclcpp::msg::UInt32>(
    "allocator_unique_ptr", 10, callback, nullptr, false, msg_memory_strategy_, alloc);

  InstrumentedAllocator<test_rclcpp::msg::UInt32> msg_alloc;

  rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
  // After test_initialization, global new should only be called from within InstrumentedAllocator.
  test_init = true;
  for (uint32_t i = 0; i < iterations; i++) {
    auto msg = std::unique_ptr<test_rclcpp::msg::UInt32, UInt32Deleter>(
      std::allocator_traits<UInt32Allocator>::allocate(msg_alloc, 1));
    msg->data = i;
    publisher_->publish(msg);
    rclcpp::utilities::sleep_for(std::chrono::milliseconds(1));
    executor_->spin_some();
  }
  test_init = false;
  EXPECT_FALSE(fail);
  fail = false;
}

void print_help()
{
  printf("--all-tokens: Do not ignore middleware tokens.\n");
  printf("--verbose: Report stack traces and allocation statistics.\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  // argc and argv are modified by InitGoogleTest
  std::vector<std::string> args(argv + 1, argv + argc);

  if (std::find(args.begin(), args.end(), "--help") != args.end()) {
    print_help();
    return 0;
  }
  verbose = std::find(args.begin(), args.end(), "--verbose") != args.end();
  ignore_middleware_tokens = std::find(args.begin(), args.end(), "--all-tokens") == args.end();

  return RUN_ALL_TESTS();
}
