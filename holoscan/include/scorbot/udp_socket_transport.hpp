// Kernel UDP socket backend.
//
// The portable baseline: works on any NIC, needs no privileges, and is the
// reference against which the ibverbs backend is measured.

#ifndef SCORBOT_UDP_SOCKET_TRANSPORT_HPP_
#define SCORBOT_UDP_SOCKET_TRANSPORT_HPP_

#include "scorbot/transport.hpp"

namespace scorbot {

class UdpSocketTransport final : public Transport {
 public:
  UdpSocketTransport() = default;
  ~UdpSocketTransport() override;

  UdpSocketTransport(const UdpSocketTransport&) = delete;
  UdpSocketTransport& operator=(const UdpSocketTransport&) = delete;

  Status open(const TransportConfig& cfg) override;
  void close() override;
  [[nodiscard]] bool is_open() const override { return fd_ >= 0; }

  bool send(const void* payload, size_t len) override;
  size_t poll_recv(void* buf, size_t cap, RxMeta* meta) override;

  [[nodiscard]] const char* name() const override { return "socket"; }
  [[nodiscard]] uint16_t local_port() const override { return local_port_; }

 private:
  int fd_ = -1;
  uint16_t local_port_ = 0;
};

}  // namespace scorbot

#endif  // SCORBOT_UDP_SOCKET_TRANSPORT_HPP_
