// Pluggable datagram transport for the Scorbot data plane.
//
// Two backends implement this interface:
//
//   UdpSocketTransport - ordinary connected UDP socket. Portable, needs no
//   privileges, goes through the kernel network stack.
//
//   IbvRawTransport - ConnectX raw packet queue pair. Bypasses the kernel
//   entirely, hand-builds Ethernet/IPv4/UDP frames and steers the matching
//   receive flow straight into userspace. Needs CAP_NET_RAW and an mlx5 device.
//
// Both put real UDP on the wire, so a single firmware implementation talks to
// either one and tcpdump sees both.

#ifndef SCORBOT_TRANSPORT_HPP_
#define SCORBOT_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "scorbot/status.hpp"

namespace scorbot {

struct MacAddr {
  uint8_t b[6]{};

  [[nodiscard]] bool is_zero() const {
    for (uint8_t v : b) {
      if (v != 0) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] std::string to_string() const;
  static Result<MacAddr> parse(const std::string& text);
};

enum class TransportKind {
  kSocket,
  kIbverbs,
};

const char* to_string(TransportKind kind);
Result<TransportKind> parse_transport_kind(const std::string& text);

struct TransportConfig {
  TransportKind kind = TransportKind::kSocket;

  // Endpoints. IPs and ports are in host byte order; conversion happens at the
  // point of use so callers never juggle byte order.
  uint32_t src_ip = 0;
  uint32_t dst_ip = 0;
  uint16_t src_port = 0;
  uint16_t dst_port = 0;

  // L2 identities. The socket backend ignores these because the kernel
  // resolves them via ARP; the ibverbs backend requires both because there is
  // no ARP on a raw queue pair.
  MacAddr src_mac{};
  MacAddr dst_mac{};

  // Interface to bind to. Used by the socket backend for SO_BINDTODEVICE and
  // by callers to look up the matching RDMA device.
  std::string ifname;

  // ibverbs backend.
  std::string ibv_device = "mlx5_1";
  uint8_t ibv_port = 1;
  bool hugepages = false;
  uint32_t rx_depth = 1024;
  uint32_t tx_depth = 1024;

  // Tuning knobs honoured by whichever backend can use them.
  uint32_t busy_poll_us = 0;  // socket: SO_BUSY_POLL
  size_t max_frame = 2048;
};

struct RxMeta {
  // Raw NIC completion timestamp in device clock units. Zero when the backend
  // has no hardware timestamping; convert with Transport::hw_ts_to_ns.
  uint64_t hw_timestamp = 0;
  uint64_t sw_timestamp_ns = 0;
};

class Transport {
 public:
  virtual ~Transport() = default;

  virtual Status open(const TransportConfig& cfg) = 0;
  virtual void close() = 0;
  [[nodiscard]] virtual bool is_open() const = 0;

  // Hot path. Allocation-free and non-blocking.
  virtual bool send(const void* payload, size_t len) = 0;

  // Returns the number of payload bytes copied into buf, or 0 when nothing is
  // pending. Never blocks.
  virtual size_t poll_recv(void* buf, size_t cap, RxMeta* meta) = 0;

  // Reap send completions. A no-op on backends that do not need it.
  virtual void poll_send() {}

  [[nodiscard]] virtual bool has_hw_timestamps() const { return false; }
  [[nodiscard]] virtual uint64_t hw_ts_to_ns(uint64_t raw) const { return raw; }
  [[nodiscard]] virtual const char* name() const = 0;

  // Local port actually bound, which matters when src_port was left at 0 and
  // the kernel picked one. The firmware needs it to address state packets.
  [[nodiscard]] virtual uint16_t local_port() const = 0;
};

std::unique_ptr<Transport> make_transport(TransportKind kind);

// True when the ibverbs backend was compiled in.
bool ibverbs_available();

// Helpers for turning human-readable configuration into wire values.
Result<uint32_t> parse_ipv4(const std::string& text);
std::string ipv4_to_string(uint32_t ip);

// Look up the MAC and IPv4 address the kernel has assigned to an interface.
Status query_interface(const std::string& ifname, MacAddr* mac, uint32_t* ip);

// Resolve a peer MAC from the kernel neighbour table, which lets the ibverbs
// backend reuse the ARP the kernel already did rather than implementing its own.
Status resolve_peer_mac(const std::string& ifname, uint32_t peer_ip, MacAddr* mac);

}  // namespace scorbot

#endif  // SCORBOT_TRANSPORT_HPP_
