// scorbot_stream - a Holoscan 5 graph driving the arm.
//
//   TrajectorySource (OnClock)  ->  ScorbotTxOp  ->  wire
//                                        |
//   TelemetrySink  <-  ScorbotRxOp  <----+---------  wire
//
// The link itself is owned by main() and bound into a slot before compile,
// because the SDK reconstructs operators from copy-captured values and will not
// carry a live handle through graph.op<>().

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <holoscan/core/compile.hpp>
#include <holoscan/core/connection_options.hpp>
#include <holoscan/core/graph.hpp>
#include <holoscan/core/run.hpp>
#include <holoscan/time/realtime_clock.hpp>

#include "operators/scorbot_ops.hpp"

namespace {

using namespace std::chrono_literals;

std::atomic<bool> g_stop{false};
void on_signal(int /*sig*/) { g_stop.store(true); }

std::atomic<std::uint64_t> g_states{0};
std::atomic<std::uint64_t> g_commands{0};

// Sweeps every homed joint through a slow sine inside its own travel limits.
class TrajectorySource final : public holoscan::Operator<> {
 public:
  TrajectorySource(std::int64_t period_ns, std::uint32_t joint_count, double amplitude,
                   double freq_hz, std::uint32_t joint_mask)
      : period_ns_(period_ns),
        joint_count_(joint_count),
        amplitude_(amplitude),
        freq_hz_(freq_hz),
        joint_mask_(joint_mask) {}

  void setup(holoscan::OperatorSpec& spec) override {
    spec.output(targets, "targets").max_emits_per_compute(1U);
  }

  [[nodiscard]] holoscan::Contract contract() const override {
    holoscan::Contract result;
    result.trigger(holoscan::OnClock{.period = std::chrono::nanoseconds{period_ns_}});
    return result;
  }

  void start() override { link_ = scorbot::LinkRegistry::resolve(0); }

  [[nodiscard]] holoscan::expected<void, holoscan::Error> compute(
      holoscan::ExecutionContext&) override {
    const double t = static_cast<double>(tick_++) * static_cast<double>(period_ns_) * 1e-9;
    const double phase = 2.0 * M_PI * freq_hz_ * t;

    scorbot::JointTargets out{};
    out.count = joint_count_;
    for (std::uint32_t j = 0; j < joint_count_ && j < SCORBOT_MAX_JOINTS; ++j) {
      // Joints outside the mask hold station. Sweeping a joint whose reference
      // has not been established would move it relative to nothing.
      if ((joint_mask_ & (1U << j)) == 0U) {
        out.mode[j] = SCORBOT_MODE_HOLD;
        continue;
      }
      float lo = -180.0F;
      float hi = 180.0F;
      if (link_ != nullptr && j < link_->info().joint_count) {
        lo = link_->info().joints[j].min_angle;
        hi = link_->info().joints[j].max_angle;
      }
      // Stay clear of the ends of travel; the sweep is a demo, not a limit test.
      const float span = (hi - lo) * static_cast<float>(amplitude_) * 0.5F;
      const float mid = 0.5F * (lo + hi);
      out.mode[j] = SCORBOT_MODE_POSITION;
      out.setpoint[j] = mid + span * static_cast<float>(std::sin(phase + 0.4 * j));
    }
    g_commands.fetch_add(1, std::memory_order_relaxed);
    return targets.emit(out);
  }

  holoscan::Output<scorbot::JointTargets> targets;

 private:
  std::int64_t period_ns_;
  std::uint32_t joint_count_;
  double amplitude_;
  double freq_hz_;
  std::uint32_t joint_mask_;
  std::uint64_t tick_ = 0;
  scorbot::ScorbotLink* link_ = nullptr;
};

// Consumes telemetry and keeps a running count plus the latest reading.
class TelemetrySink final : public holoscan::Operator<> {
 public:
  void setup(holoscan::OperatorSpec& spec) override {
    spec.input(state, "state").queue_depth(8U);
  }

  [[nodiscard]] holoscan::Contract contract() const override {
    holoscan::Contract result;
    result.trigger(holoscan::OnEach{state});
    return result;
  }

  [[nodiscard]] holoscan::expected<void, holoscan::Error> compute(
      holoscan::ExecutionContext&) override {
    auto sample = state.receive();
    if (!sample) {
      return holoscan::make_unexpected(std::move(sample).error());
    }
    latest = sample->data;
    g_states.fetch_add(1, std::memory_order_relaxed);
    return {};
  }

  holoscan::Input<scorbot_state_t> state;
  static inline scorbot_state_t latest{};
};

// Drains the transmit operator's echo port. Without a consumer the port's loan
// pool fills after a couple of emits, so wiring it here also exercises the
// "what did we actually put on the wire" path that a recorder would use.
class SentDrainOp final : public holoscan::Operator<> {
 public:
  void setup(holoscan::OperatorSpec& spec) override {
    spec.input(sent, "sent").queue_depth(8U);
  }

  [[nodiscard]] holoscan::Contract contract() const override {
    holoscan::Contract result;
    result.trigger(holoscan::OnEach{sent});
    return result;
  }

  [[nodiscard]] holoscan::expected<void, holoscan::Error> compute(
      holoscan::ExecutionContext&) override {
    auto sample = sent.receive();
    if (!sample) {
      return holoscan::make_unexpected(std::move(sample).error());
    }
    last_seq = sample->data.seq;
    return {};
  }

  holoscan::Input<scorbot_pose_t> sent;
  static inline std::atomic<std::uint32_t> last_seq{0};
};

void print_session_diagnostics(const holoscan::RunSession& session) {
  for (const auto& d : session.diagnostics()) {
    std::fprintf(stderr, "  diagnostic %s severity=%u vertex=%u\n", std::string(d.token).c_str(),
                 static_cast<unsigned>(d.severity),
                 static_cast<unsigned>(d.source.vertex.value));
  }
}

void print_usage() {
  std::printf(
      "usage: scorbot_stream [options]\n"
      "  --ip ADDR          robot address (default 192.168.0.161)\n"
      "  --iface NAME       local interface (default enp2s0f1np1)\n"
      "  --transport KIND   socket or ibverbs (default socket)\n"
      "  --ibv-dev NAME     RDMA device (default mlx5_1)\n"
      "  --local-port N     local data plane port (default 6011)\n"
      "  --rate HZ          setpoint rate (default 500)\n"
      "  --seconds N        run duration, 0 for until interrupted (default 10)\n"
      "  --amplitude F      fraction of each joint's travel to sweep (default 0.3)\n"
      "  --freq HZ          sweep frequency (default 0.15)\n"
      "  --joint-mask N     bitmask of joints to sweep; others hold (default all)\n"
      "  --require-homed    refuse to move unless every joint reports homed\n"
      "  --force            take the session from another host\n");
}

}  // namespace

int main(int argc, char** argv) {
  scorbot::ScorbotLink::Config cfg;
  double rate_hz = 500.0;
  double seconds = 10.0;
  double amplitude = 0.3;
  double freq_hz = 0.15;
  bool require_homed = false;
  std::uint32_t joint_mask = 0xFFU;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ip" && i + 1 < argc) {
      cfg.robot_ip = argv[++i];
    } else if (arg == "--iface" && i + 1 < argc) {
      cfg.ifname = argv[++i];
    } else if (arg == "--ibv-dev" && i + 1 < argc) {
      cfg.ibv_device = argv[++i];
    } else if (arg == "--local-port" && i + 1 < argc) {
      cfg.local_data_port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--transport" && i + 1 < argc) {
      auto kind = scorbot::parse_transport_kind(argv[++i]);
      if (!kind) {
        std::fprintf(stderr, "%s\n", kind.status().c_str());
        return 1;
      }
      cfg.transport = *kind;
    } else if (arg == "--rate" && i + 1 < argc) {
      rate_hz = std::stod(argv[++i]);
    } else if (arg == "--seconds" && i + 1 < argc) {
      seconds = std::stod(argv[++i]);
    } else if (arg == "--amplitude" && i + 1 < argc) {
      amplitude = std::stod(argv[++i]);
    } else if (arg == "--freq" && i + 1 < argc) {
      freq_hz = std::stod(argv[++i]);
    } else if (arg == "--joint-mask" && i + 1 < argc) {
      joint_mask = static_cast<std::uint32_t>(std::stoul(argv[++i], nullptr, 0));
    } else if (arg == "--require-homed") {
      require_homed = true;
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
  std::printf("scorbot_stream: %s over %s, session %u, %u joints\n", cfg.robot_ip.c_str(),
              link.transport_name(), link.session_id(), link.info().joint_count);

  // Take one reading before composing so the sweep can be centred, and so
  // --require-homed has something to check.
  scorbot_state_t first{};
  for (int i = 0; i < 200 && !link.poll_state(first); ++i) {
    (void)link.send_keepalive();
    std::this_thread::sleep_for(5ms);
  }

  if (require_homed) {
    for (std::uint8_t j = 0; j < link.info().joint_count; ++j) {
      if ((joint_mask & (1U << j)) == 0U) {
        continue;
      }
      if ((first.jflags[j] & SCORBOT_JFLAG_HOMED) == 0U) {
        std::fprintf(stderr, "joint %u (%s) is not homed; run 'scorbot_cli home' first\n", j,
                     link.info().joints[j].name.c_str());
        return 1;
      }
    }
  }

  // Bind before compile: every reconstructed operator resolves this same slot.
  scorbot::LinkRegistry::bind(link, 0);

  const auto period_ns = static_cast<std::int64_t>(1e9 / rate_hz);

  holoscan::Graph graph{"scorbot-stream"};
  const auto source = graph.op<TrajectorySource>("trajectory", period_ns,
                                                 static_cast<std::uint32_t>(
                                                     link.info().joint_count),
                                                 amplitude, freq_hz, joint_mask);
  const auto tx = graph.op<scorbot::ScorbotTxOp>("tx", 0U);
  const auto rx = graph.op<scorbot::ScorbotRxOp>("rx", 0U, static_cast<std::int64_t>(200));
  const auto sink = graph.op<TelemetrySink>("telemetry");
  const auto sent_drain = graph.op<SentDrainOp>("sent_drain");

  graph.add_flow(source->targets, tx->targets, holoscan::ConnectionOptions{.queue_depth = 4U});
  graph.add_flow(tx->sent, sent_drain->sent, holoscan::ConnectionOptions{.queue_depth = 8U});
  graph.add_flow(rx->state, sink->state, holoscan::ConnectionOptions{.queue_depth = 8U});

  // Two clock sources means EA1 requires manual partitioning, and every
  // operator must be covered exactly once by a partition with a single
  // topology root. The split is also the concurrency design: telemetry polling
  // gets its own execution lane instead of serializing behind setpoint sends.
  graph.partition("command").add(source).add(tx).add(sent_drain);
  graph.partition("telemetry").add(rx).add(sink);

  graph.set_default_clock(graph.add_clock<holoscan::RealtimeClock>("clock"));

  const holoscan::ExecutionPlan plan = holoscan::compile(graph);
  if (!plan.ok()) {
    // The full plan JSON is enormous; lead with the diagnostics that say why.
    std::fprintf(stderr, "graph compile failed:\n");
    for (const auto& d : plan.diagnostics()) {
      std::fprintf(stderr, "  %s (severity %u)\n", std::string(d.token).c_str(),
                   static_cast<unsigned>(d.severity));
    }
    if (std::getenv("SCORBOT_DUMP_PLAN") != nullptr) {
      std::fprintf(stderr, "%s\n", std::string(plan.json()).c_str());
    } else {
      std::fprintf(stderr, "  (set SCORBOT_DUMP_PLAN=1 for the full plan)\n");
    }
    return 1;
  }

  holoscan::RunSession session = holoscan::run_async(plan);

  const auto started = std::chrono::steady_clock::now();
  while (!g_stop.load()) {
    std::this_thread::sleep_for(200ms);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (seconds > 0.0 && elapsed >= seconds) {
      break;
    }
  }

  session.request_stop();
  session.wait();

  if (session.termination() != holoscan::RunTermination::kStopped ||
      !session.diagnostics().empty()) {
    std::fprintf(stderr, "\nrun ended abnormally (termination=%u)\n",
                 static_cast<unsigned>(session.termination()));
    print_session_diagnostics(session);
  }

  // Leave the arm holding rather than sweeping toward a stale setpoint.
  (void)link.set_mode_all(SCORBOT_MODE_HOLD);

  const auto& stats = link.stats();
  std::printf("\ncommands emitted   %llu\n",
              static_cast<unsigned long long>(g_commands.load()));
  std::printf("poses transmitted  %llu (%llu failed)\n",
              static_cast<unsigned long long>(stats.poses_sent),
              static_cast<unsigned long long>(stats.poses_failed));
  std::printf("states received    %llu (%llu dropped, %llu bad)\n",
              static_cast<unsigned long long>(stats.states_received),
              static_cast<unsigned long long>(stats.states_dropped),
              static_cast<unsigned long long>(stats.bad_packets));
  std::printf("states into graph  %llu\n", static_cast<unsigned long long>(g_states.load()));
  if (link.latency().count() > 0) {
    // Command-to-report, which includes the firmware's wait for its next
    // reporting tick. Use scorbot_ping for pure wire round trip.
    std::printf("command-to-report (us)  p50 %.1f  p99 %.1f  max %.1f\n",
                link.latency().percentile_us(50.0), link.latency().percentile_us(99.0),
                link.latency().max_us());
  }
  return 0;
}
