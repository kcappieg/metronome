#pragma once

#include <exception>
#include <string>
namespace metronome {
class MetronomeException : public std::exception {
 public:
  explicit MetronomeException(std::string message) noexcept : message{std::move(message)} {}
  MetronomeException(const MetronomeException& other) noexcept : message{other.message} {};

  [[nodiscard]] const char* what() const noexcept override { return this->message.c_str(); }

 protected:
  const std::string message;
};

class MetronomeTimeoutException : public MetronomeException {
 public:
  MetronomeTimeoutException() noexcept : MetronomeException("Timeout!") {}
  MetronomeTimeoutException(const MetronomeTimeoutException&) noexcept = default;
};

}  // namespace metronome
