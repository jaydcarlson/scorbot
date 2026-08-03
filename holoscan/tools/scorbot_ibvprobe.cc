// scorbot_ibvprobe - diagnostics for the ConnectX raw packet path.
//
// Two jobs, neither of which needs a robot that speaks our protocol:
//
//   --selftest  Validates frame construction offline: build a frame, parse it
//               back, and check the IPv4 header checksum against an
//               independently computed reference.
//
//   (default)   Brings up the raw queue pair and fires hand-built UDP frames at
//               an arbitrary MAC/IP/port. Useful because mlx5 raw-QP transmit
//               bypasses the kernel, so local tcpdump cannot see it; aiming at
//               a closed port on a live host and watching for the ICMP
//               port-unreachable reply proves the frames are well formed and
//               really reached the peer.

#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "scorbot/eth_frame.hpp"
#include "scorbot/ibv_raw_transport.hpp"
#include "scorbot/scorbot_link.hpp"
#include "scorbot/transport.hpp"

namespace {

// Independent reference: sum the header as 16-bit big-endian words with the
// checksum field zeroed, fold the carries, and complement.
uint16_t reference_checksum(const uint8_t* header) {
  uint32_t sum = 0;
  for (std::size_t i = 0; i < scorbot::kIpv4HeaderLen; i += 2) {
    if (i == 10) {
      continue;  // the checksum field itself
    }
    sum += (static_cast<uint32_t>(header[i]) << 8) | header[i + 1];
  }
  while ((sum >> 16) != 0U) {
    sum = (sum & 0xFFFFU) + (sum >> 16);
  }
  return static_cast<uint16_t>(~sum);
}

int selftest() {
  using namespace scorbot;
  int failures = 0;
  auto check = [&failures](bool ok, const char* what) {
    std::printf("  %-52s %s\n", what, ok ? "pass" : "FAIL");
    if (!ok) {
      ++failures;
    }
  };

  MacAddr src{{0xf0, 0xfb, 0x7f, 0xec, 0x71, 0x95}};
  MacAddr dst{{0x00, 0x80, 0xe1, 0x00, 0x00, 0x00}};
  const uint32_t src_ip = *parse_ipv4("192.168.0.16");
  const uint32_t dst_ip = *parse_ipv4("192.168.0.161");

  scorbot_pose_t pose{};
  pose.magic = SCORBOT_MAGIC_POSE;
  pose.version = SCORBOT_PROTO_VERSION;
  pose.seq = 0x11223344;
  pose.t_tx_ns = 0x0123456789ABCDEFULL;
  for (unsigned j = 0; j < SCORBOT_MAX_JOINTS; ++j) {
    pose.mode[j] = SCORBOT_MODE_POSITION;
    pose.setpoint[j] = static_cast<float>(j) * 1.5F;
  }

  std::vector<uint8_t> frame(kHeadersLen + sizeof(pose));
  const size_t len =
      build_udp_frame(src, dst, src_ip, dst_ip, 6011, 6001, &pose, sizeof(pose), frame.data());

  std::printf("frame construction (%zu bytes total, %zu payload):\n", len, sizeof(pose));
  check(len == 106, "total frame length is 106 bytes");

  const auto* eth = reinterpret_cast<const EthHeader*>(frame.data());
  check(std::memcmp(eth->dst_mac, dst.b, 6) == 0, "destination MAC");
  check(std::memcmp(eth->src_mac, src.b, 6) == 0, "source MAC");
  check(net_to_host16(eth->ethertype) == 0x0800, "ethertype is IPv4");

  const auto* ip = reinterpret_cast<const Ipv4Header*>(frame.data() + kEthHeaderLen);
  check(ip->version_ihl == 0x45, "IPv4 version and header length");
  check(ip->protocol == 17, "protocol is UDP");
  check(net_to_host16(ip->total_length) == 20 + 8 + sizeof(pose), "IP total length");
  check(net_to_host32(ip->src_ip) == src_ip, "source IP");
  check(net_to_host32(ip->dst_ip) == dst_ip, "destination IP");
  const uint16_t got_ck = net_to_host16(ip->header_checksum);
  const uint16_t want_ck = reference_checksum(frame.data() + kEthHeaderLen);
  if (got_ck != want_ck) {
    std::printf("    (checksum got 0x%04x, reference 0x%04x)\n", got_ck, want_ck);
  }
  check(got_ck == want_ck, "IPv4 checksum matches independent reference");

  // The conclusive test: folding the whole header including its checksum must
  // come out all ones for any receiver to accept the packet.
  uint32_t fold = 0;
  for (size_t i = 0; i < kIpv4HeaderLen; i += 2) {
    const uint8_t* h = frame.data() + kEthHeaderLen;
    fold += (static_cast<uint32_t>(h[i]) << 8) | h[i + 1];
  }
  while ((fold >> 16) != 0U) {
    fold = (fold & 0xFFFFU) + (fold >> 16);
  }
  check(fold == 0xFFFFU, "full header folds to 0xffff (receiver-side validity)");

  const auto* udp = reinterpret_cast<const UdpHeader*>(frame.data() + kEthHeaderLen +
                                                       kIpv4HeaderLen);
  check(net_to_host16(udp->src_port) == 6011, "UDP source port");
  check(net_to_host16(udp->dst_port) == 6001, "UDP destination port");
  check(net_to_host16(udp->length) == 8 + sizeof(pose), "UDP length");

  ParsedFrame parsed;
  check(parse_udp_frame(frame.data(), len, &parsed), "frame parses back");
  check(parsed.payload_len == sizeof(pose), "parsed payload length");
  check(std::memcmp(parsed.payload, &pose, sizeof(pose)) == 0, "payload round trips byte for byte");
  check(parsed.src_ip == src_ip, "parsed source IP");
  check(parsed.dst_port == 6001, "parsed destination port");

  // A truncated frame must be rejected rather than read past its end.
  check(!parse_udp_frame(frame.data(), kHeadersLen - 1, &parsed), "truncated frame is rejected");
  std::vector<uint8_t> bad = frame;
  reinterpret_cast<EthHeader*>(bad.data())->ethertype = host_to_net16(0x86DD);
  check(!parse_udp_frame(bad.data(), len, &parsed), "non-IPv4 ethertype is rejected");

  std::printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL PASS" : "FAILURES", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? 0 : 1;
}

void print_usage() {
  std::printf(
      "usage: scorbot_ibvprobe [options]\n"
      "  --selftest         validate frame construction offline and exit\n"
      "  --iface NAME       local interface (default enp2s0f1np1)\n"
      "  --ibv-dev NAME     RDMA device (default mlx5_1)\n"
      "  --dst-ip ADDR      destination address (required)\n"
      "  --dst-mac MAC      destination MAC; looked up via the kernel if omitted\n"
      "  --dst-port N       destination UDP port (default 6001)\n"
      "  --local-port N     source UDP port, also the flow rule match (default 6011)\n"
      "  --count N          frames to send (default 10)\n"
      "  --rate HZ          send rate (default 100)\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string ifname = "enp2s0f1np1";
  std::string ibv_dev = "mlx5_1";
  std::string dst_ip_text;
  std::string dst_mac_text;
  uint16_t dst_port = 6001;
  uint16_t local_port = 6011;
  int count = 10;
  double rate_hz = 100.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--selftest") {
      return selftest();
    } else if (arg == "--iface" && i + 1 < argc) {
      ifname = argv[++i];
    } else if (arg == "--ibv-dev" && i + 1 < argc) {
      ibv_dev = argv[++i];
    } else if (arg == "--dst-ip" && i + 1 < argc) {
      dst_ip_text = argv[++i];
    } else if (arg == "--dst-mac" && i + 1 < argc) {
      dst_mac_text = argv[++i];
    } else if (arg == "--dst-port" && i + 1 < argc) {
      dst_port = static_cast<uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--local-port" && i + 1 < argc) {
      local_port = static_cast<uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--count" && i + 1 < argc) {
      count = std::stoi(argv[++i]);
    } else if (arg == "--rate" && i + 1 < argc) {
      rate_hz = std::stod(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      return 1;
    }
  }

  if (!scorbot::ibverbs_available()) {
    std::fprintf(stderr, "this build has no ibverbs support\n");
    return 1;
  }
  if (dst_ip_text.empty()) {
    std::fprintf(stderr, "--dst-ip is required\n");
    print_usage();
    return 1;
  }

  scorbot::TransportConfig cfg;
  cfg.kind = scorbot::TransportKind::kIbverbs;
  cfg.ifname = ifname;
  cfg.ibv_device = ibv_dev;
  cfg.src_port = local_port;
  cfg.dst_port = dst_port;

  auto dst_ip = scorbot::parse_ipv4(dst_ip_text);
  if (!dst_ip) {
    std::fprintf(stderr, "%s\n", dst_ip.status().c_str());
    return 1;
  }
  cfg.dst_ip = *dst_ip;

  if (scorbot::Status s = scorbot::query_interface(ifname, &cfg.src_mac, &cfg.src_ip); !s) {
    std::fprintf(stderr, "%s\n", s.c_str());
    return 1;
  }

  if (!dst_mac_text.empty()) {
    auto mac = scorbot::MacAddr::parse(dst_mac_text);
    if (!mac) {
      std::fprintf(stderr, "%s\n", mac.status().c_str());
      return 1;
    }
    cfg.dst_mac = *mac;
  } else if (scorbot::Status s = scorbot::resolve_peer_mac(ifname, cfg.dst_ip, &cfg.dst_mac); !s) {
    std::fprintf(stderr, "%s\n", s.c_str());
    return 1;
  }

  std::printf("source      %s  %s:%u\n", cfg.src_mac.to_string().c_str(),
              scorbot::ipv4_to_string(cfg.src_ip).c_str(), cfg.src_port);
  std::printf("destination %s  %s:%u\n", cfg.dst_mac.to_string().c_str(),
              scorbot::ipv4_to_string(cfg.dst_ip).c_str(), cfg.dst_port);

  scorbot::IbvRawTransport transport;
  if (scorbot::Status s = transport.open(cfg); !s) {
    std::fprintf(stderr, "transport did not come up: %s\n", s.c_str());
    return 1;
  }
  std::printf("queue pair and flow rule are up (hardware timestamps: %s)\n",
              transport.has_hw_timestamps() ? "yes" : "no");

  scorbot_pose_t pose{};
  pose.magic = SCORBOT_MAGIC_POSE;
  pose.version = SCORBOT_PROTO_VERSION;
  for (auto& m : pose.mode) {
    m = SCORBOT_MODE_HOLD;
  }

  const auto period = std::chrono::nanoseconds(static_cast<long>(1e9 / rate_hz));
  int sent = 0;
  int received = 0;
  uint8_t rx[2048];

  for (int i = 0; i < count; ++i) {
    pose.seq = static_cast<uint32_t>(i + 1);
    pose.t_tx_ns = scorbot::monotonic_ns();
    if (transport.send(&pose, sizeof(pose))) {
      ++sent;
    }
    transport.poll_send();
    scorbot::RxMeta meta{};
    while (transport.poll_recv(rx, sizeof(rx), &meta) > 0) {
      ++received;
    }
    std::this_thread::sleep_for(period);
  }

  // Give any late replies a moment to arrive.
  for (int i = 0; i < 200; ++i) {
    scorbot::RxMeta meta{};
    while (transport.poll_recv(rx, sizeof(rx), &meta) > 0) {
      ++received;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::printf("\nsent %d frames, received %d\n", sent, received);
  std::printf(
      "Note: mlx5 raw-QP transmit bypasses the kernel, so local tcpdump will not\n"
      "      show these. Watch the peer, or watch this host for an ICMP reply.\n");
  transport.close();
  return 0;
}
