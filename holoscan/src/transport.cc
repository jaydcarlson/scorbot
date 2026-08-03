#include "scorbot/transport.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netpacket/packet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "scorbot/ibv_raw_transport.hpp"
#include "scorbot/udp_socket_transport.hpp"

namespace scorbot {

std::string MacAddr::to_string() const {
  char buf[18];
  std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", b[0], b[1], b[2], b[3], b[4],
                b[5]);
  return {buf};
}

Result<MacAddr> MacAddr::parse(const std::string& text) {
  MacAddr mac;
  unsigned int v[6];
  if (std::sscanf(text.c_str(), "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) !=
      6) {
    return Status::error("not a MAC address: " + text);
  }
  for (int i = 0; i < 6; ++i) {
    if (v[i] > 0xFF) {
      return Status::error("MAC octet out of range: " + text);
    }
    mac.b[i] = static_cast<uint8_t>(v[i]);
  }
  return mac;
}

const char* to_string(TransportKind kind) {
  switch (kind) {
    case TransportKind::kSocket:
      return "socket";
    case TransportKind::kIbverbs:
      return "ibverbs";
  }
  return "unknown";
}

Result<TransportKind> parse_transport_kind(const std::string& text) {
  if (text == "socket" || text == "udp") {
    return TransportKind::kSocket;
  }
  if (text == "ibverbs" || text == "ibv" || text == "connectx") {
    return TransportKind::kIbverbs;
  }
  return Status::error("unknown transport '" + text + "', expected socket or ibverbs");
}

Result<uint32_t> parse_ipv4(const std::string& text) {
  in_addr addr{};
  if (inet_pton(AF_INET, text.c_str(), &addr) != 1) {
    return Status::error("not an IPv4 address: " + text);
  }
  return ntohl(addr.s_addr);
}

std::string ipv4_to_string(uint32_t ip) {
  in_addr addr{};
  addr.s_addr = htonl(ip);
  char buf[INET_ADDRSTRLEN] = {};
  inet_ntop(AF_INET, &addr, buf, sizeof(buf));
  return {buf};
}

Status query_interface(const std::string& ifname, MacAddr* mac, uint32_t* ip) {
  ifaddrs* ifa_list = nullptr;
  if (getifaddrs(&ifa_list) != 0) {
    return Status::error(std::string("getifaddrs failed: ") + std::strerror(errno), errno);
  }

  bool found_mac = false;
  bool found_ip = false;
  for (ifaddrs* ifa = ifa_list; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifname != ifa->ifa_name) {
      continue;
    }
    if (ifa->ifa_addr->sa_family == AF_PACKET && mac != nullptr) {
      const auto* ll = reinterpret_cast<const sockaddr_ll*>(ifa->ifa_addr);
      if (ll->sll_halen == 6) {
        std::memcpy(mac->b, ll->sll_addr, 6);
        found_mac = true;
      }
    } else if (ifa->ifa_addr->sa_family == AF_INET && ip != nullptr) {
      const auto* sin = reinterpret_cast<const sockaddr_in*>(ifa->ifa_addr);
      *ip = ntohl(sin->sin_addr.s_addr);
      found_ip = true;
    }
  }
  freeifaddrs(ifa_list);

  if (mac != nullptr && !found_mac) {
    return Status::error("no MAC address for interface " + ifname);
  }
  if (ip != nullptr && !found_ip) {
    return Status::error("no IPv4 address for interface " + ifname);
  }
  return Status::ok();
}

Status resolve_peer_mac(const std::string& ifname, uint32_t peer_ip, MacAddr* mac) {
  if (mac == nullptr) {
    return Status::error("resolve_peer_mac needs an output pointer");
  }

  // Nudge the kernel into resolving the peer first. A datagram to a discard
  // port is enough to populate the neighbour table without disturbing the
  // firmware, and costs nothing if the entry is already there.
  const int probe = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (probe >= 0) {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = htonl(peer_ip);
    dst.sin_port = htons(9);  // discard
    const char byte = 0;
    (void)::sendto(probe, &byte, 1, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    ::close(probe);
  }

  // Read the resolved entry back out of the kernel. Parsing /proc/net/arp is
  // far less code than an rtnetlink conversation and this runs once at setup.
  const std::string want_ip = ipv4_to_string(peer_ip);
  for (int attempt = 0; attempt < 20; ++attempt) {
    FILE* f = std::fopen("/proc/net/arp", "re");
    if (f == nullptr) {
      return Status::error("cannot open /proc/net/arp");
    }
    char line[256];
    if (std::fgets(line, sizeof(line), f) == nullptr) {  // discard the header row
      std::fclose(f);
      return Status::error("/proc/net/arp is empty");
    }
    while (std::fgets(line, sizeof(line), f) != nullptr) {
      char ip_text[64] = {};
      char hw_text[64] = {};
      char dev_text[64] = {};
      unsigned int hw_type = 0;
      unsigned int flags = 0;
      char mask_text[64] = {};
      if (std::sscanf(line, "%63s 0x%x 0x%x %63s %63s %63s", ip_text, &hw_type, &flags, hw_text,
                      mask_text, dev_text) < 6) {
        continue;
      }
      if (want_ip != ip_text || ifname != dev_text) {
        continue;
      }
      if ((flags & 0x02U) == 0U) {  // ATF_COM: entry not yet complete
        continue;
      }
      std::fclose(f);
      auto parsed = MacAddr::parse(hw_text);
      if (!parsed) {
        return parsed.status();
      }
      *mac = *parsed;
      return Status::ok();
    }
    std::fclose(f);
    usleep(10000);
  }

  return Status::error("could not resolve MAC for " + want_ip + " on " + ifname +
                       " (is the robot powered and on this segment?)");
}

std::unique_ptr<Transport> make_transport(TransportKind kind) {
  switch (kind) {
    case TransportKind::kSocket:
      return std::make_unique<UdpSocketTransport>();
    case TransportKind::kIbverbs:
      return std::make_unique<IbvRawTransport>();
  }
  return nullptr;
}

bool ibverbs_available() {
#ifdef SCORBOT_HAVE_IBVERBS
  return true;
#else
  return false;
#endif
}

}  // namespace scorbot
