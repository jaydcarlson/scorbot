#include "scorbot/scorbot_link.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <thread>

namespace scorbot {

uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

// --------------------------------------------------------------------------
// LatencyStats
// --------------------------------------------------------------------------

void LatencyStats::add_us(double us) {
  samples_.push_back(us);
  sum_ += us;
}

void LatencyStats::reset() {
  samples_.clear();
  sum_ = 0.0;
}

double LatencyStats::min_us() const {
  if (samples_.empty()) {
    return 0.0;
  }
  return *std::min_element(samples_.begin(), samples_.end());
}

double LatencyStats::max_us() const {
  if (samples_.empty()) {
    return 0.0;
  }
  return *std::max_element(samples_.begin(), samples_.end());
}

double LatencyStats::mean_us() const {
  if (samples_.empty()) {
    return 0.0;
  }
  return sum_ / static_cast<double>(samples_.size());
}

double LatencyStats::percentile_us(double p) const {
  if (samples_.empty()) {
    return 0.0;
  }
  std::vector<double> sorted = samples_;
  std::sort(sorted.begin(), sorted.end());
  // Nearest-rank, matching how the ethernet motor benchmark reports.
  auto idx = static_cast<size_t>(p / 100.0 * static_cast<double>(sorted.size()));
  if (idx >= sorted.size()) {
    idx = sorted.size() - 1;
  }
  return sorted[idx];
}

// --------------------------------------------------------------------------
// ScorbotLink
// --------------------------------------------------------------------------

ScorbotLink::~ScorbotLink() { close(); }

const char* ScorbotLink::transport_name() const {
  return transport_ ? transport_->name() : "none";
}

Status ScorbotLink::open_control(const Config& cfg) {
  cfg_ = cfg;

  auto ip = parse_ipv4(cfg_.robot_ip);
  if (!ip) {
    return ip.status();
  }
  robot_ip_ = *ip;

  ControlClient::Config ccfg;
  ccfg.robot_ip = robot_ip_;
  ccfg.robot_port = cfg_.robot_ctrl_port;
  ccfg.ifname = cfg_.ifname;
  ccfg.timeout_ms = cfg_.ctrl_timeout_ms;
  ccfg.retries = cfg_.ctrl_retries;
  if (Status s = control_.open(ccfg); !s) {
    return s;
  }

  auto info = query_info();
  if (!info) {
    control_.close();
    return info.status();
  }
  return Status::ok();
}

Result<RobotInfo> ScorbotLink::query_info() {
  auto reply = control_.hello();
  if (!reply) {
    return reply.status();
  }

  RobotInfo info{};
  info.fw_version = reply->fw_version;
  info.proto_version = reply->proto_version;
  info.uptime_ms = reply->uptime_ms;
  info.active_session = reply->active_session;
  info.owner_ip = reply->owner_ip;
  info.joint_count = reply->joint_count;
  std::memcpy(info.robot_mac.b, reply->robot_mac, 6);

  for (size_t j = 0; j < SCORBOT_MAX_JOINTS; ++j) {
    char name[SCORBOT_NAME_LEN + 1] = {};
    std::memcpy(name, reply->joint_name[j], SCORBOT_NAME_LEN);
    info.joints[j].name = name;
    info.joints[j].min_angle = reply->min_angle[j];
    info.joints[j].max_angle = reply->max_angle[j];
  }

  if (info.proto_version != SCORBOT_PROTO_VERSION) {
    return Status::error("protocol mismatch: robot speaks v" +
                         std::to_string(info.proto_version) + ", this build speaks v" +
                         std::to_string(SCORBOT_PROTO_VERSION));
  }

  info_ = info;
  robot_mac_ = info.robot_mac;
  return info;
}

Status ScorbotLink::open(const Config& cfg) {
  if (Status s = open_control(cfg); !s) {
    return s;
  }

  // Learn our own L2/L3 identity so the firmware can address us without ARP,
  // exactly as the Hololink data plane is configured today.
  if (Status s = query_interface(cfg_.ifname, &local_mac_, &local_ip_); !s) {
    close();
    return s;
  }

  if (Status s = bring_up_transport(); !s) {
    close();
    return s;
  }

  scorbot_open_req_t req{};
  std::memcpy(req.host_mac, local_mac_.b, 6);
  req.host_port = transport_->local_port();
  req.host_ip = local_ip_;
  req.watchdog_ms = cfg_.watchdog_ms;
  req.force = cfg_.force_session ? 1U : 0U;

  auto opened = control_.open_session(req);
  if (!opened) {
    close();
    return opened.status();
  }
  session_id_ = opened->session_id;

  if (Status s = control_.stream_start(cfg_.state_period_us, cfg_.watchdog_ms); !s) {
    close();
    return s;
  }

  streaming_ = true;
  tx_seq_ = 0;
  have_rx_seq_ = false;
  return Status::ok();
}

Status ScorbotLink::bring_up_transport() {
  transport_ = make_transport(cfg_.transport);
  if (!transport_) {
    return Status::error("could not create transport");
  }

  TransportConfig tcfg;
  tcfg.kind = cfg_.transport;
  tcfg.src_ip = local_ip_;
  tcfg.dst_ip = robot_ip_;
  tcfg.src_port = cfg_.local_data_port;
  tcfg.dst_port = cfg_.robot_data_port;
  tcfg.ifname = cfg_.ifname;
  tcfg.ibv_device = cfg_.ibv_device;
  tcfg.ibv_port = cfg_.ibv_port;
  tcfg.hugepages = cfg_.hugepages;
  tcfg.busy_poll_us = cfg_.busy_poll_us;

  if (cfg_.transport == TransportKind::kIbverbs) {
    // A raw queue pair has no ARP, so both MACs must be known up front. HELLO
    // already told us the robot's; fall back to the kernel neighbour table if
    // the firmware reported nothing useful.
    tcfg.src_mac = local_mac_;
    tcfg.dst_mac = robot_mac_;
    if (tcfg.dst_mac.is_zero()) {
      if (Status s = resolve_peer_mac(cfg_.ifname, robot_ip_, &tcfg.dst_mac); !s) {
        return s;
      }
      robot_mac_ = tcfg.dst_mac;
    }
    if (tcfg.src_port == 0) {
      return Status::error("the ibverbs backend needs a fixed local_data_port for its flow rule");
    }
  }

  return transport_->open(tcfg);
}

void ScorbotLink::close() {
  if (streaming_ && control_.is_open()) {
    (void)control_.stream_stop();
    (void)control_.close_session();
  }
  streaming_ = false;
  if (transport_) {
    transport_->close();
    transport_.reset();
  }
  control_.close();
  session_id_ = 0;
  have_state_ = false;
  have_rx_seq_ = false;
}

// ---- Control plane passthroughs -----------------------------------------

Status ScorbotLink::set_mode_all(uint8_t mode) {
  uint8_t modes[SCORBOT_MAX_JOINTS];
  std::fill_n(modes, SCORBOT_MAX_JOINTS, mode);
  return control_.set_mode(modes, 0xFF);
}

Status ScorbotLink::set_mode(uint8_t joint, uint8_t mode) {
  if (joint >= SCORBOT_MAX_JOINTS) {
    return Status::error("joint index out of range");
  }
  uint8_t modes[SCORBOT_MAX_JOINTS] = {};
  modes[joint] = mode;
  return control_.set_mode(modes, static_cast<uint8_t>(1U << joint));
}

Status ScorbotLink::set_gains(const scorbot_gains_t& gains) { return control_.set_gains(gains); }

Result<scorbot_gains_t> ScorbotLink::get_gains(uint8_t joint) { return control_.get_gains(joint); }

Status ScorbotLink::set_homing_cfg(const scorbot_homing_cfg_t& cfg) {
  return control_.set_homing_cfg(cfg);
}

Result<scorbot_homing_cfg_t> ScorbotLink::get_homing_cfg(uint8_t joint) {
  return control_.get_homing_cfg(joint);
}

Status ScorbotLink::set_limits(uint8_t joint, float min_angle, float max_angle) {
  scorbot_limits_t limits{};
  limits.joint = joint;
  limits.min_angle = min_angle;
  limits.max_angle = max_angle;
  return control_.set_limits(limits);
}

Status ScorbotLink::zero(uint8_t joint, float angle) { return control_.zero(joint, angle); }

Status ScorbotLink::abort() { return control_.abort(); }

Status ScorbotLink::start_homing(uint8_t joint_mask) {
  return control_.home(joint_mask, SCORBOT_HOME_CONFIRM);
}

Result<scorbot_home_status_t> ScorbotLink::homing_status() { return control_.home_status(); }

Status ScorbotLink::home_blocking(uint8_t joint_mask, uint32_t timeout_ms) {
  if (Status s = start_homing(joint_mask); !s) {
    return s;
  }

  const uint64_t deadline = monotonic_ns() + static_cast<uint64_t>(timeout_ms) * 1000000ULL;
  while (monotonic_ns() < deadline) {
    auto st = homing_status();
    if (!st) {
      return st.status();
    }
    if (st->state == SCORBOT_HOMING_DONE) {
      return Status::ok();
    }
    if (st->state == SCORBOT_HOMING_FAULT) {
      return Status::error("homing faulted on joint " + std::to_string(st->joint) + " (code " +
                           std::to_string(st->error) + ")");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  (void)abort();
  return Status::error("homing did not finish within " + std::to_string(timeout_ms) + " ms");
}

// ---- Data plane ----------------------------------------------------------

bool ScorbotLink::send_pose(scorbot_pose_t& pose) {
  if (!transport_ || !transport_->is_open()) {
    return false;
  }
  pose.magic = SCORBOT_MAGIC_POSE;
  pose.version = SCORBOT_PROTO_VERSION;
  pose.session_id = session_id_;
  pose.seq = ++tx_seq_;
  pose.t_tx_ns = monotonic_ns();

  if (!transport_->send(&pose, sizeof(pose))) {
    ++stats_.poses_failed;
    return false;
  }
  ++stats_.poses_sent;
  return true;
}

bool ScorbotLink::send_positions(const float* angles, size_t count, uint8_t mode) {
  scorbot_pose_t pose{};
  const size_t n = std::min<size_t>(count, SCORBOT_MAX_JOINTS);
  for (size_t j = 0; j < n; ++j) {
    pose.mode[j] = mode;
    pose.setpoint[j] = angles[j];
  }
  // Joints the caller did not mention keep holding rather than going limp.
  for (size_t j = n; j < SCORBOT_MAX_JOINTS; ++j) {
    pose.mode[j] = SCORBOT_MODE_HOLD;
  }
  return send_pose(pose);
}

bool ScorbotLink::send_keepalive() {
  scorbot_pose_t pose{};
  pose.flags = SCORBOT_POSE_FLAG_KEEPALIVE;
  for (auto& m : pose.mode) {
    m = SCORBOT_MODE_HOLD;
  }
  return send_pose(pose);
}

bool ScorbotLink::validate_state(const scorbot_state_t& state) {
  if (state.magic != SCORBOT_MAGIC_STATE || state.version != SCORBOT_PROTO_VERSION) {
    ++stats_.bad_packets;
    return false;
  }
  // Session 0 means the firmware is replying outside a session, which happens
  // between a stream stop and the next open; treat it as stale rather than bad.
  if (session_id_ != 0 && state.session_id != session_id_) {
    ++stats_.bad_packets;
    return false;
  }
  return true;
}

bool ScorbotLink::poll_state(scorbot_state_t& out) {
  if (!transport_ || !transport_->is_open()) {
    return false;
  }

  scorbot_state_t scratch{};
  RxMeta meta{};
  bool got_any = false;

  // Drain the whole queue and keep only the newest. For a control loop, a
  // backlog of stale telemetry is worthless: only the latest reading matters.
  while (true) {
    const size_t n = transport_->poll_recv(&scratch, sizeof(scratch), &meta);
    if (n == 0) {
      break;
    }
    if (n != sizeof(scorbot_state_t) || !validate_state(scratch)) {
      ++stats_.bad_packets;
      continue;
    }

    if (have_rx_seq_ && scratch.seq > last_rx_seq_ + 1) {
      stats_.states_dropped += scratch.seq - last_rx_seq_ - 1;
    }
    last_rx_seq_ = scratch.seq;
    have_rx_seq_ = true;
    ++stats_.states_received;

    if ((scratch.flags & SCORBOT_STATE_FLAG_WATCHDOG) != 0U &&
        (last_state_.flags & SCORBOT_STATE_FLAG_WATCHDOG) == 0U) {
      ++stats_.watchdog_trips;
    }

    // t_echo_ns is our own transmit timestamp coming back, so the difference
    // needs no clock synchronisation with the firmware. Note this is
    // command-to-report latency, not pure wire round trip: on a free-running
    // stream it also contains however long the firmware waited for its next
    // reporting tick. measure_rtt_us() is the one that isolates the wire.
    //
    // Only trust echoes of sequence numbers we actually sent this session; a
    // firmware that failed to clear its echo fields on session open would
    // otherwise produce a round trip spanning two sessions.
    if (scratch.t_echo_ns != 0 && scratch.echo_seq != 0 && scratch.echo_seq <= tx_seq_) {
      const uint64_t now = monotonic_ns();
      if (now > scratch.t_echo_ns) {
        const double rtt_us = static_cast<double>(now - scratch.t_echo_ns) / 1000.0;
        stats_.last_rtt_us = rtt_us;
        latency_.add_us(rtt_us);
      }
    }

    last_state_ = scratch;
    have_state_ = true;
    got_any = true;
  }

  if (got_any) {
    out = last_state_;
  }
  return got_any;
}

void ScorbotLink::poll_send() {
  if (transport_) {
    transport_->poll_send();
  }
}

Result<double> ScorbotLink::measure_rtt_us(uint32_t timeout_ms) {
  if (!transport_ || !transport_->is_open()) {
    return Status::error("data plane is not open");
  }

  // Drop anything already queued so we time our own exchange rather than a
  // packet the firmware happened to emit on its own tick.
  scorbot_state_t drain{};
  RxMeta meta{};
  while (transport_->poll_recv(&drain, sizeof(drain), &meta) > 0) {
  }

  scorbot_pose_t pose{};
  pose.flags = SCORBOT_POSE_FLAG_KEEPALIVE | SCORBOT_POSE_FLAG_REPLY_NOW;
  for (auto& m : pose.mode) {
    m = SCORBOT_MODE_HOLD;
  }
  if (!send_pose(pose)) {
    return Status::error("failed to transmit the probe");
  }
  const uint32_t want_seq = pose.seq;
  const uint64_t sent_ns = pose.t_tx_ns;

  const uint64_t deadline = monotonic_ns() + static_cast<uint64_t>(timeout_ms) * 1000000ULL;
  while (monotonic_ns() < deadline) {
    transport_->poll_send();
    const size_t n = transport_->poll_recv(&drain, sizeof(drain), &meta);
    if (n != sizeof(scorbot_state_t)) {
      continue;
    }
    if (!validate_state(drain) || drain.echo_seq != want_seq) {
      continue;  // an unrelated tick-driven packet
    }
    const double rtt_us = static_cast<double>(monotonic_ns() - sent_ns) / 1000.0;
    stats_.last_rtt_us = rtt_us;
    ++stats_.states_received;
    last_state_ = drain;
    have_state_ = true;
    return rtt_us;
  }

  return Status::error("no reply within " + std::to_string(timeout_ms) + " ms");
}

void ScorbotLink::reset_stats() {
  stats_ = LinkStats{};
  latency_.reset();
}

}  // namespace scorbot
