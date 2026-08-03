// scorbot_ping - round trip latency benchmark, one transport at a time.
//
// Each probe sets SCORBOT_POSE_FLAG_REPLY_NOW so the firmware answers on
// receipt rather than on its own periodic tick. Without that, every sample
// would carry up to a full tick period of phase wait, which at 1 kHz is
// hundreds of microseconds of noise on top of a measurement whose interesting
// range is tens of microseconds.

#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "scorbot/scorbot_link.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int /*sig*/) { g_stop = 1; }

void print_usage() {
  std::printf(
      "usage: scorbot_ping [options]\n"
      "  --ip ADDR          robot address (default 192.168.0.161)\n"
      "  --iface NAME       local interface (default enp2s0f1np1)\n"
      "  --transport KIND   socket or ibverbs (default socket)\n"
      "  --ibv-dev NAME     RDMA device (default mlx5_1)\n"
      "  --local-port N     local data plane port (default 6011)\n"
      "  --count N          probes to send (default 10000)\n"
      "  --rate N           probes per second (default 1000)\n"
      "  --warmup N         probes to discard first (default 200)\n"
      "  --busy-poll US     SO_BUSY_POLL microseconds, socket backend only\n"
      "  --force            take the session from another host\n");
}

}  // namespace

int main(int argc, char** argv) {
  scorbot::ScorbotLink::Config cfg;
  int count = 10000;
  int rate_hz = 1000;
  int warmup = 200;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ip" && i + 1 < argc) {
      cfg.robot_ip = argv[++i];
    } else if (arg == "--iface" && i + 1 < argc) {
      cfg.ifname = argv[++i];
    } else if (arg == "--ibv-dev" && i + 1 < argc) {
      cfg.ibv_device = argv[++i];
    } else if (arg == "--local-port" && i + 1 < argc) {
      cfg.local_data_port = static_cast<uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--busy-poll" && i + 1 < argc) {
      cfg.busy_poll_us = static_cast<uint32_t>(std::stoi(argv[++i]));
    } else if (arg == "--transport" && i + 1 < argc) {
      auto kind = scorbot::parse_transport_kind(argv[++i]);
      if (!kind) {
        std::fprintf(stderr, "%s\n", kind.status().c_str());
        return 1;
      }
      cfg.transport = *kind;
    } else if (arg == "--count" && i + 1 < argc) {
      count = std::stoi(argv[++i]);
    } else if (arg == "--rate" && i + 1 < argc) {
      rate_hz = std::stoi(argv[++i]);
    } else if (arg == "--warmup" && i + 1 < argc) {
      warmup = std::stoi(argv[++i]);
    } else if (arg == "--force") {
      cfg.force_session = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      return 1;
    }
  }

  std::signal(SIGINT, on_signal);

  scorbot::ScorbotLink link;
  if (scorbot::Status s = link.open(cfg); !s) {
    std::fprintf(stderr, "could not open the link: %s\n", s.c_str());
    return 1;
  }

  std::printf("scorbot_ping: %s over %s, %d probes at %d Hz (%d warmup)\n", cfg.robot_ip.c_str(),
              link.transport_name(), count, rate_hz, warmup);

  const uint64_t period_ns = rate_hz > 0 ? 1000000000ULL / static_cast<uint64_t>(rate_hz) : 0;
  uint64_t next_ns = scorbot::monotonic_ns();

  int sent = 0;
  int lost = 0;
  for (int i = 0; i < count && g_stop == 0; ++i) {
    auto rtt = link.measure_rtt_us(50);
    if (!rtt) {
      ++lost;
    } else if (i >= warmup) {
      link.latency().add_us(*rtt);
    }
    ++sent;

    if (period_ns > 0) {
      next_ns += period_ns;
      const uint64_t now = scorbot::monotonic_ns();
      if (next_ns > now) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(next_ns - now));
      } else {
        next_ns = now;  // we fell behind; do not try to catch up in a burst
      }
    }
  }

  auto& stats = link.latency();
  std::printf("\n--- %s transport, %d probes sent, %d lost ---\n", link.transport_name(), sent,
              lost);
  if (stats.count() == 0) {
    std::fprintf(stderr, "no samples collected; is the robot streaming?\n");
    return 1;
  }
  std::printf("round trip (us):\n");
  std::printf("  min    %8.1f\n", stats.min_us());
  std::printf("  mean   %8.1f\n", stats.mean_us());
  std::printf("  p50    %8.1f\n", stats.percentile_us(50.0));
  std::printf("  p90    %8.1f\n", stats.percentile_us(90.0));
  std::printf("  p99    %8.1f\n", stats.percentile_us(99.0));
  std::printf("  p99.9  %8.1f\n", stats.percentile_us(99.9));
  std::printf("  max    %8.1f\n", stats.max_us());
  std::printf("samples  %8zu\n", stats.count());

  return 0;
}
