#pragma once

#include <utility>

namespace metronome {
// SFINAE utility
// shamelessly stolen from https://jguegant.github.io/blogs/tech/sfinae-introduction.html
template <typename FnType>
class IsValidContainer {
 public:
  template <typename... Params>
  constexpr auto operator()(Params&&...) {
    return isValid<Params...>(int{0});
  }

 private:
  template <typename... Params>
  constexpr auto isValid(int /* unused */)
      -> decltype(std::declval<FnType>()(std::declval<Params>()...),
                  std::true_type()) {
    return std::true_type();
  }

  template <typename... Params>
  constexpr std::false_type isValid(...) {
    return std::false_type();
  }
};

template <typename FnType>
constexpr auto isValidFactory(FnType&&) {
  return IsValidContainer<FnType>();
}

#define DEFINE_METHOD_CALL_IF_AVAILABLE(resultFnName, methodName) \
  auto resultFnName##Function_has_##methodName = isValidFactory([](auto&& instance, auto&&... args) -> decltype(instance.methodName(args...)) {}); \
                                                                  \
  template <typename InstanceType, typename... Args> \
  auto resultFnName(InstanceType& instance, Args&... args) \
    ->typename std::enable_if<decltype(resultFnName##Function_has_##methodName(instance, args...))::value, void>::type { \
    instance.methodName(args...); \
  } \
\
  template <typename InstanceType, typename... Args> \
    auto resultFnName(InstanceType& instance, Args&... args) \
      ->typename std::enable_if<!decltype(resultFnName##Function_has_##methodName(instance, args...))::value, void>::type {}

} //namespace metronome