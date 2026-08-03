// Minimal error type for the Scorbot comms layer.
//
// std::expected is C++23 and the Holoscan build is C++20, and this layer must
// not depend on holoscan::expected because it has to stay usable stand-alone.
// So setup-time calls return Status/Result and the hot path returns plain
// bool/size_t to keep it allocation-free.

#ifndef SCORBOT_STATUS_HPP_
#define SCORBOT_STATUS_HPP_

#include <string>
#include <utility>

namespace scorbot {

class Status {
 public:
  Status() = default;

  static Status ok() { return {}; }
  static Status error(std::string message, int code = -1) {
    Status s;
    s.code_ = code;
    s.message_ = std::move(message);
    return s;
  }

  [[nodiscard]] bool ok_status() const { return code_ == 0; }
  explicit operator bool() const { return code_ == 0; }

  [[nodiscard]] int code() const { return code_; }
  [[nodiscard]] const std::string& message() const { return message_; }

  [[nodiscard]] const char* c_str() const {
    return message_.empty() ? "ok" : message_.c_str();
  }

 private:
  int code_ = 0;
  std::string message_;
};

template <typename T>
class Result {
 public:
  Result(T value) : value_(std::move(value)) {}  // NOLINT(google-explicit-constructor)
  Result(Status status) : status_(std::move(status)) {}  // NOLINT

  explicit operator bool() const { return status_.ok_status(); }
  [[nodiscard]] const Status& status() const { return status_; }

  T& operator*() { return value_; }
  const T& operator*() const { return value_; }
  T* operator->() { return &value_; }
  const T* operator->() const { return &value_; }

 private:
  Status status_;
  T value_{};
};

}  // namespace scorbot

#endif  // SCORBOT_STATUS_HPP_
