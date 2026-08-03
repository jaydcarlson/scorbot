#include "scorbot/udp_socket_transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>

namespace scorbot {
namespace {

uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace

UdpSocketTransport::~UdpSocketTransport() { close(); }

Status UdpSocketTransport::open(const TransportConfig& cfg) {
  close();

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return Status::error(std::string("socket() failed: ") + std::strerror(errno), errno);
  }

  int one = 1;
  (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  // Generous buffers so a scheduling hiccup does not drop telemetry.
  int bufsize = 16 * 1024 * 1024;
  (void)::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
  (void)::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));

  if (!cfg.ifname.empty()) {
    // Best effort: needs CAP_NET_RAW, and the route is usually unambiguous
    // anyway, so a failure here is not worth aborting over.
    (void)::setsockopt(fd_, SOL_SOCKET, SO_BINDTODEVICE, cfg.ifname.c_str(),
                       static_cast<socklen_t>(cfg.ifname.size()));
  }

  if (cfg.busy_poll_us > 0) {
    auto usec = static_cast<int>(cfg.busy_poll_us);
    (void)::setsockopt(fd_, SOL_SOCKET, SO_BUSY_POLL, &usec, sizeof(usec));
  }

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = cfg.src_ip == 0 ? INADDR_ANY : htonl(cfg.src_ip);
  local.sin_port = htons(cfg.src_port);
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
    const int err = errno;
    close();
    return Status::error(std::string("bind() failed: ") + std::strerror(err), err);
  }

  // Record whatever port we actually got, since the firmware has to be told
  // where to send state when src_port was left as 0.
  sockaddr_in bound{};
  socklen_t bound_len = sizeof(bound);
  if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&bound), &bound_len) == 0) {
    local_port_ = ntohs(bound.sin_port);
  }

  // Connecting a UDP socket lets the hot path use send() rather than sendto(),
  // skipping a route lookup per datagram, and filters out stray sources.
  if (cfg.dst_ip != 0 && cfg.dst_port != 0) {
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_addr.s_addr = htonl(cfg.dst_ip);
    peer.sin_port = htons(cfg.dst_port);
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) != 0) {
      const int err = errno;
      close();
      return Status::error(std::string("connect() failed: ") + std::strerror(err), err);
    }
  }

  const int flags = ::fcntl(fd_, F_GETFL, 0);
  (void)::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  return Status::ok();
}

void UdpSocketTransport::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  local_port_ = 0;
}

bool UdpSocketTransport::send(const void* payload, size_t len) {
  if (fd_ < 0) {
    return false;
  }
  const ssize_t n = ::send(fd_, payload, len, MSG_DONTWAIT);
  return n == static_cast<ssize_t>(len);
}

size_t UdpSocketTransport::poll_recv(void* buf, size_t cap, RxMeta* meta) {
  if (fd_ < 0) {
    return 0;
  }
  const ssize_t n = ::recv(fd_, buf, cap, MSG_DONTWAIT);
  if (n <= 0) {
    return 0;
  }
  if (meta != nullptr) {
    meta->hw_timestamp = 0;
    meta->sw_timestamp_ns = monotonic_ns();
  }
  return static_cast<size_t>(n);
}

}  // namespace scorbot
