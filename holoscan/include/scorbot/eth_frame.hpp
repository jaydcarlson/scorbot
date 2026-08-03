// Hand-built Ethernet/IPv4/UDP framing.
//
// The ibverbs raw packet queue pair has no network stack behind it, so we
// build and parse the headers ourselves. What goes on the wire is ordinary
// UDP, which is what lets the same firmware endpoint serve both backends and
// lets tcpdump decode the traffic normally.

#ifndef SCORBOT_ETH_FRAME_HPP_
#define SCORBOT_ETH_FRAME_HPP_

#include <cstdint>
#include <cstring>

#include "scorbot/transport.hpp"

namespace scorbot {

#pragma pack(push, 1)

struct EthHeader {
  uint8_t dst_mac[6];
  uint8_t src_mac[6];
  uint16_t ethertype;  // network order
};

struct Ipv4Header {
  uint8_t version_ihl;       // 0x45: IPv4, 20-byte header
  uint8_t dscp_ecn;
  uint16_t total_length;     // network order, IP header + UDP + payload
  uint16_t identification;
  uint16_t flags_fragment;   // network order
  uint8_t ttl;
  uint8_t protocol;
  uint16_t header_checksum;  // network order, over the 20-byte IP header
  uint32_t src_ip;           // network order
  uint32_t dst_ip;           // network order
};

struct UdpHeader {
  uint16_t src_port;  // network order
  uint16_t dst_port;  // network order
  uint16_t length;    // network order, UDP header + payload
  uint16_t checksum;  // 0 means "not computed", which is legal over IPv4
};

#pragma pack(pop)

inline constexpr size_t kEthHeaderLen = sizeof(EthHeader);
inline constexpr size_t kIpv4HeaderLen = sizeof(Ipv4Header);
inline constexpr size_t kUdpHeaderLen = sizeof(UdpHeader);
inline constexpr size_t kHeadersLen = kEthHeaderLen + kIpv4HeaderLen + kUdpHeaderLen;

inline constexpr uint16_t kEthertypeIpv4 = 0x0800;
inline constexpr uint8_t kIpProtoUdp = 17;

static_assert(kEthHeaderLen == 14, "ethernet header must be 14 bytes");
static_assert(kIpv4HeaderLen == 20, "ipv4 header must be 20 bytes");
static_assert(kUdpHeaderLen == 8, "udp header must be 8 bytes");
static_assert(kHeadersLen == 42, "combined headers must be 42 bytes");

inline uint16_t host_to_net16(uint16_t v) {
  return static_cast<uint16_t>((v << 8) | (v >> 8));
}

inline uint16_t net_to_host16(uint16_t v) { return host_to_net16(v); }

inline uint32_t host_to_net32(uint32_t v) {
  return __builtin_bswap32(v);
}

inline uint32_t net_to_host32(uint32_t v) { return __builtin_bswap32(v); }

// Standard one's-complement checksum over the IPv4 header, returned in network
// byte order ready to store.
//
// Reads through unsigned char deliberately. Casting the header to uint16_t* and
// summing words is the obvious formulation but violates strict aliasing, and at
// -O2 GCC then computes the sum over stale bytes and emits packets no receiver
// will accept. Byte access is always allowed to alias.
inline uint16_t ipv4_checksum_bytes(const uint8_t* header) {
  uint32_t sum = 0;
  for (size_t i = 0; i < kIpv4HeaderLen; i += 2) {
    sum += (static_cast<uint32_t>(header[i]) << 8) | static_cast<uint32_t>(header[i + 1]);
  }
  while ((sum >> 16) != 0U) {
    sum = (sum & 0xFFFFU) + (sum >> 16);
  }
  return host_to_net16(static_cast<uint16_t>(~sum));
}

inline uint16_t ipv4_checksum(const Ipv4Header& hdr) {
  return ipv4_checksum_bytes(reinterpret_cast<const uint8_t*>(&hdr));
}

// Writes a complete frame into out and returns its total length. out must have
// room for kHeadersLen + payload_len bytes.
inline size_t build_udp_frame(const MacAddr& src_mac, const MacAddr& dst_mac, uint32_t src_ip,
                              uint32_t dst_ip, uint16_t src_port, uint16_t dst_port,
                              const void* payload, size_t payload_len, uint8_t* out) {
  auto* eth = reinterpret_cast<EthHeader*>(out);
  std::memcpy(eth->dst_mac, dst_mac.b, 6);
  std::memcpy(eth->src_mac, src_mac.b, 6);
  eth->ethertype = host_to_net16(kEthertypeIpv4);

  auto* ip = reinterpret_cast<Ipv4Header*>(out + kEthHeaderLen);
  ip->version_ihl = 0x45;
  ip->dscp_ecn = 0;
  ip->total_length =
      host_to_net16(static_cast<uint16_t>(kIpv4HeaderLen + kUdpHeaderLen + payload_len));
  ip->identification = 0;
  ip->flags_fragment = host_to_net16(0x4000);  // don't fragment
  ip->ttl = 64;
  ip->protocol = kIpProtoUdp;
  ip->header_checksum = 0;
  ip->src_ip = host_to_net32(src_ip);
  ip->dst_ip = host_to_net32(dst_ip);
  ip->header_checksum = ipv4_checksum_bytes(out + kEthHeaderLen);

  auto* udp = reinterpret_cast<UdpHeader*>(out + kEthHeaderLen + kIpv4HeaderLen);
  udp->src_port = host_to_net16(src_port);
  udp->dst_port = host_to_net16(dst_port);
  udp->length = host_to_net16(static_cast<uint16_t>(kUdpHeaderLen + payload_len));
  udp->checksum = 0;

  if (payload != nullptr && payload_len > 0) {
    std::memcpy(out + kHeadersLen, payload, payload_len);
  }
  return kHeadersLen + payload_len;
}

struct ParsedFrame {
  const uint8_t* payload = nullptr;
  size_t payload_len = 0;
  uint32_t src_ip = 0;
  uint16_t src_port = 0;
  uint16_t dst_port = 0;
};

inline bool parse_udp_frame(const uint8_t* frame, size_t len, ParsedFrame* out) {
  if (frame == nullptr || len < kHeadersLen) {
    return false;
  }
  const auto* eth = reinterpret_cast<const EthHeader*>(frame);
  if (net_to_host16(eth->ethertype) != kEthertypeIpv4) {
    return false;
  }
  const auto* ip = reinterpret_cast<const Ipv4Header*>(frame + kEthHeaderLen);
  if ((ip->version_ihl >> 4) != 4 || ip->protocol != kIpProtoUdp) {
    return false;
  }
  // Options-bearing headers would shift the UDP offset; the firmware never
  // sends them, so rejecting is safer than guessing.
  const size_t ihl_bytes = static_cast<size_t>(ip->version_ihl & 0x0F) * 4U;
  if (ihl_bytes != kIpv4HeaderLen) {
    return false;
  }
  const auto* udp = reinterpret_cast<const UdpHeader*>(frame + kEthHeaderLen + kIpv4HeaderLen);
  const size_t udp_len = net_to_host16(udp->length);
  if (udp_len < kUdpHeaderLen || kEthHeaderLen + kIpv4HeaderLen + udp_len > len) {
    return false;
  }

  out->payload = frame + kHeadersLen;
  out->payload_len = udp_len - kUdpHeaderLen;
  out->src_ip = net_to_host32(ip->src_ip);
  out->src_port = net_to_host16(udp->src_port);
  out->dst_port = net_to_host16(udp->dst_port);
  return true;
}

}  // namespace scorbot

#endif  // SCORBOT_ETH_FRAME_HPP_
