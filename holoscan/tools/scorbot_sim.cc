// scorbot_sim - a stand-in for the STM32 firmware.
//
// Speaks the robot side of both planes over ordinary sockets: answers control
// plane requests, accepts pose packets, runs a crude joint model and a homing
// state machine, and streams telemetry back. It exists so the comms layer, the
// Holoscan operators and the web UI can all be exercised end to end before any
// real motor turns.
//
// The joint model is intentionally simple. The point is to make the protocol
// and the sequencing observable, not to be a faithful dynamics simulation.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "scorbot/transport.hpp"
#include "scorbot_proto.h"

namespace {

uint64_t now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

uint32_t now_ms() { return static_cast<uint32_t>(now_ns() / 1000000ULL); }

struct SimJoint {
  std::string name;
  float min_angle = -180.0F;
  float max_angle = 180.0F;
  float gear_ratio = 100.0F;  // encoder counts per degree
  float max_speed_dps = 90.0F;

  // Live state.
  float position = 0.0F;
  float velocity = 0.0F;
  float setpoint = 0.0F;
  float effort = 0.0F;
  uint8_t mode = SCORBOT_MODE_IDLE;
  bool homed = false;
  bool faulted = false;
  bool saturated = false;

  // PID.
  float kp = 0.05F;
  float ki = 0.0F;
  float kd = 0.0F;
  float i_accum = 0.0F;
  float i_clamp = 0.25F;
  float out_clamp = 1.0F;
  float deadband_deg = 0.05F;
  float max_vel_dps = 60.0F;
  float ramped_setpoint = 0.0F;

  // Homing configuration and the mechanical truth the model uses.
  int8_t direction = -1;
  uint8_t order = 0;
  uint8_t enabled = 1;
  float seek_duty = 0.6F;
  float creep_duty = 0.15F;
  float backoff_deg = 3.0F;
  float home_offset_deg = 0.0F;
  uint32_t timeout_ms = 20000;
  float stall_eps_deg = 0.05F;
  uint32_t stall_window_ms = 750;

  float switch_at = 0.0F;  // the switch closes at or below this angle
  float hard_stop = 0.0F;  // physical travel limit, just past the switch

  // Per-joint homing progress.
  uint8_t home_phase = SCORBOT_HOMING_IDLE;
  float backoff_target = 0.0F;

  [[nodiscard]] bool switch_closed() const { return position <= switch_at; }
};

struct Session {
  uint32_t id = 0;
  uint32_t owner_ip = 0;
  uint8_t host_mac[6] = {};
  uint32_t host_ip = 0;
  uint16_t host_port = 0;
  uint32_t watchdog_ms = 100;
  uint32_t state_period_us = 1000;
  bool streaming = false;
};

class Simulator {
 public:
  explicit Simulator(bool verbose) : verbose_(verbose) { build_joints(); }

  int run(uint16_t ctrl_port, uint16_t data_port);

 private:
  void build_joints();
  void tick(double dt);
  void step_homing(double dt);
  void begin_group(uint8_t order);
  void fill_state(scorbot_state_t& out) const;

  void handle_control(const uint8_t* buf, size_t len, const sockaddr_in& from);
  void handle_pose(const uint8_t* buf, size_t len);
  void reply(uint16_t opcode, uint32_t xid, uint16_t status, const void* payload, size_t len,
             const sockaddr_in& to);
  void all_joints_idle();

  std::vector<SimJoint> joints_;
  Session session_;
  bool verbose_ = false;

  int ctrl_fd_ = -1;
  int data_fd_ = -1;

  uint32_t tick_seq_ = 0;
  uint32_t echo_seq_ = 0;
  uint64_t t_echo_ns_ = 0;
  uint64_t last_pose_ns_ = 0;
  uint64_t boot_ns_ = now_ns();

  uint8_t homing_state_ = SCORBOT_HOMING_IDLE;
  uint8_t homing_joint_ = 0xFF;
  uint8_t homing_order_ = 0;
  uint8_t homing_mask_ = 0;
  uint32_t homing_started_ms_ = 0;
  uint64_t phase_started_ns_ = 0;

  uint8_t fault_ = SCORBOT_FAULT_NONE;
  bool watchdog_tripped_ = false;
  bool estop_ = false;
  uint16_t missed_deadlines_ = 0;
};

void Simulator::build_joints() {
  // Mirrors the real joint table, with the linear slide included as joint 6.
  struct Spec {
    const char* name;
    float min_angle;
    float max_angle;
    float gear_ratio;
    uint8_t order;
  };
  // Homing order confirmed against the hardware: gripper first, the two
  // coupled wrist joints together, then elbow, lift, pan, and finally the
  // slide. Equal order values home concurrently.
  const Spec specs[] = {
      {"shoulder_pan", -130.0F, 160.0F, 6500.0F / 90.0F, 4},
      {"shoulder_lift", 0.0F, 90.0F, 20000.0F / 360.0F, 3},
      {"elbow", -130.0F, 160.0F, 5000.0F / 90.0F, 2},
      {"wrist_1", -180.0F, 180.0F, 1200.0F / 90.0F, 1},
      {"wrist_2", -180.0F, 180.0F, 1200.0F / 90.0F, 1},
      {"gripper", 0.0F, 60.0F, 100.0F, 0},
      {"slide", 0.0F, 300.0F, 50.0F, 5},
  };

  for (const auto& spec : specs) {
    SimJoint j;
    j.name = spec.name;
    j.min_angle = spec.min_angle;
    j.max_angle = spec.max_angle;
    j.gear_ratio = spec.gear_ratio;
    j.order = spec.order;
    // Every limit switch sits at the negative extent of travel, so the switch
    // closes at min_angle and that is the angle homing assigns.
    j.switch_at = spec.min_angle;
    j.hard_stop = spec.min_angle - 2.0F;
    j.home_offset_deg = spec.min_angle;
    // Start somewhere plausible and un-homed, as the real robot does at boot.
    j.position = spec.min_angle + 0.35F * (spec.max_angle - spec.min_angle);
    j.setpoint = j.position;
    j.ramped_setpoint = j.position;
    joints_.push_back(j);
  }
  joints_.resize(SCORBOT_MAX_JOINTS);
}

void Simulator::all_joints_idle() {
  for (auto& j : joints_) {
    j.mode = SCORBOT_MODE_IDLE;
    j.effort = 0.0F;
    j.velocity = 0.0F;
    j.i_accum = 0.0F;
  }
}

void Simulator::begin_group(uint8_t order) {
  homing_order_ = order;
  homing_state_ = SCORBOT_HOMING_SEEK;
  phase_started_ns_ = now_ns();
  homing_joint_ = 0xFF;
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto& j = joints_[i];
    if (j.name.empty() || j.order != order || j.enabled == 0U) {
      continue;
    }
    if ((homing_mask_ & (1U << i)) == 0U) {
      continue;
    }
    j.home_phase = SCORBOT_HOMING_SEEK;
    j.mode = SCORBOT_MODE_PWM;
    j.homed = false;
    if (homing_joint_ == 0xFF) {
      homing_joint_ = static_cast<uint8_t>(i);
    }
  }
}

void Simulator::step_homing(double /*dt*/) {
  if (homing_state_ == SCORBOT_HOMING_IDLE || homing_state_ == SCORBOT_HOMING_DONE ||
      homing_state_ == SCORBOT_HOMING_FAULT) {
    return;
  }

  if (homing_state_ == SCORBOT_HOMING_PARK) {
    all_joints_idle();
    // Find the lowest order group that has a joint selected for homing.
    int lowest = -1;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto& j = joints_[i];
      if (j.name.empty() || j.enabled == 0U || (homing_mask_ & (1U << i)) == 0U) {
        continue;
      }
      if (lowest < 0 || j.order < static_cast<uint8_t>(lowest)) {
        lowest = j.order;
      }
    }
    if (lowest < 0) {
      homing_state_ = SCORBOT_HOMING_DONE;
      return;
    }
    begin_group(static_cast<uint8_t>(lowest));
    return;
  }

  bool group_active = false;
  bool group_done = true;

  for (size_t i = 0; i < joints_.size(); ++i) {
    auto& j = joints_[i];
    if (j.name.empty() || j.order != homing_order_ || (homing_mask_ & (1U << i)) == 0U ||
        j.enabled == 0U) {
      continue;
    }
    if (j.home_phase == SCORBOT_HOMING_LATCH || j.home_phase == SCORBOT_HOMING_IDLE) {
      continue;
    }
    group_active = true;
    group_done = false;
    homing_joint_ = static_cast<uint8_t>(i);

    const double elapsed_ms = static_cast<double>(now_ns() - phase_started_ns_) / 1e6;
    if (elapsed_ms > static_cast<double>(j.timeout_ms)) {
      homing_state_ = SCORBOT_HOMING_FAULT;
      fault_ = SCORBOT_FAULT_HOMING;
      j.faulted = true;
      all_joints_idle();
      std::fprintf(stderr, "sim: homing timeout on %s\n", j.name.c_str());
      return;
    }

    switch (j.home_phase) {
      case SCORBOT_HOMING_SEEK:
        j.mode = SCORBOT_MODE_PWM;
        j.setpoint = static_cast<float>(j.direction) * j.seek_duty;
        if (j.switch_closed()) {
          j.home_phase = SCORBOT_HOMING_BACKOFF;
          j.backoff_target = j.position - static_cast<float>(j.direction) * j.backoff_deg;
          phase_started_ns_ = now_ns();
        }
        break;

      case SCORBOT_HOMING_BACKOFF:
        // Retreat far enough that the switch definitely releases, so the slow
        // pass that follows always starts from a known open state.
        j.mode = SCORBOT_MODE_PWM;
        j.setpoint = static_cast<float>(-j.direction) * j.creep_duty * 2.0F;
        if (!j.switch_closed() &&
            std::fabs(j.position - j.backoff_target) < 0.5F) {
          j.home_phase = SCORBOT_HOMING_CREEP;
          phase_started_ns_ = now_ns();
        }
        break;

      case SCORBOT_HOMING_CREEP:
        j.mode = SCORBOT_MODE_PWM;
        j.setpoint = static_cast<float>(j.direction) * j.creep_duty;
        if (j.switch_closed()) {
          j.home_phase = SCORBOT_HOMING_LATCH;
        }
        break;

      default:
        break;
    }

    if (j.home_phase == SCORBOT_HOMING_LATCH) {
      // Latch the reference: this position is now home_offset_deg by definition.
      j.position = j.home_offset_deg;
      j.setpoint = j.home_offset_deg;
      j.ramped_setpoint = j.home_offset_deg;
      j.velocity = 0.0F;
      j.effort = 0.0F;
      j.homed = true;
      j.mode = SCORBOT_MODE_POSITION;
      if (verbose_) {
        std::printf("sim: %s homed at %.2f\n", j.name.c_str(), j.home_offset_deg);
      }
    }
  }

  if (group_active && !group_done) {
    // Still working through this group.
    bool all_latched = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto& j = joints_[i];
      if (j.name.empty() || j.order != homing_order_ || (homing_mask_ & (1U << i)) == 0U ||
          j.enabled == 0U) {
        continue;
      }
      if (j.home_phase != SCORBOT_HOMING_LATCH) {
        all_latched = false;
        break;
      }
    }
    if (!all_latched) {
      return;
    }
  }

  // This group is finished; find the next one.
  int next = -1;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& j = joints_[i];
    if (j.name.empty() || j.enabled == 0U || (homing_mask_ & (1U << i)) == 0U) {
      continue;
    }
    if (j.order <= homing_order_ || j.homed) {
      continue;
    }
    if (next < 0 || j.order < static_cast<uint8_t>(next)) {
      next = j.order;
    }
  }

  if (next < 0) {
    homing_state_ = SCORBOT_HOMING_DONE;
    homing_joint_ = 0xFF;
    for (auto& j : joints_) {
      if (j.homed) {
        j.mode = SCORBOT_MODE_POSITION;
      }
    }
    if (verbose_) {
      std::printf("sim: homing sequence complete\n");
    }
    return;
  }
  begin_group(static_cast<uint8_t>(next));
}

void Simulator::tick(double dt) {
  step_homing(dt);

  for (auto& j : joints_) {
    if (j.name.empty()) {
      continue;
    }

    float duty = 0.0F;
    switch (j.mode) {
      case SCORBOT_MODE_IDLE:
        j.i_accum = 0.0F;
        j.ramped_setpoint = j.position;
        break;

      case SCORBOT_MODE_HOLD:
        // Hold wherever we are: retarget once, then run the position loop.
        if (j.ramped_setpoint == 0.0F && j.setpoint == 0.0F) {
          j.ramped_setpoint = j.position;
        }
        [[fallthrough]];

      case SCORBOT_MODE_POSITION: {
        float target = (j.mode == SCORBOT_MODE_HOLD) ? j.ramped_setpoint : j.setpoint;
        target = std::clamp(target, j.min_angle, j.max_angle);
        // Slew-limit the setpoint so a big step does not command a lurch.
        const float max_step = j.max_vel_dps * static_cast<float>(dt);
        const float delta = std::clamp(target - j.ramped_setpoint, -max_step, max_step);
        j.ramped_setpoint += delta;

        const float error = j.ramped_setpoint - j.position;
        if (std::fabs(error) < j.deadband_deg) {
          j.i_accum = 0.0F;
          duty = 0.0F;
        } else {
          j.i_accum = std::clamp(j.i_accum + error * static_cast<float>(dt) * j.ki, -j.i_clamp,
                                 j.i_clamp);
          const float derivative = -j.velocity;
          duty = j.kp * error + j.i_accum + j.kd * derivative;
        }
        break;
      }

      case SCORBOT_MODE_VELOCITY:
        duty = std::clamp(j.setpoint / j.max_speed_dps, -1.0F, 1.0F);
        break;

      case SCORBOT_MODE_PWM:
        duty = j.setpoint;
        break;

      default:
        break;
    }

    const float clamped = std::clamp(duty, -j.out_clamp, j.out_clamp);
    j.saturated = std::fabs(clamped - duty) > 1e-6F;
    j.effort = clamped;

    j.velocity = clamped * j.max_speed_dps;
    j.position += j.velocity * static_cast<float>(dt);

    // Mechanical stops. The switch sits just inside the negative stop.
    if (j.position < j.hard_stop) {
      j.position = j.hard_stop;
      j.velocity = 0.0F;
    }
    const float upper = j.max_angle + 2.0F;
    if (j.position > upper) {
      j.position = upper;
      j.velocity = 0.0F;
    }
  }
}

void Simulator::fill_state(scorbot_state_t& out) const {
  std::memset(&out, 0, sizeof(out));
  out.magic = SCORBOT_MAGIC_STATE;
  out.version = SCORBOT_PROTO_VERSION;
  out.session_id = session_.id;
  out.seq = tick_seq_;
  out.t_echo_ns = t_echo_ns_;
  out.echo_seq = echo_seq_;
  out.t_fw_us = static_cast<uint32_t>((now_ns() - boot_ns_) / 1000ULL);
  out.homing_state = homing_state_;
  out.homing_joint = homing_joint_;
  out.fault = fault_;
  out.missed_deadlines = missed_deadlines_;

  if (session_.streaming) {
    out.flags |= SCORBOT_STATE_FLAG_STREAMING;
  }
  if (watchdog_tripped_) {
    out.flags |= SCORBOT_STATE_FLAG_WATCHDOG;
  }
  if (estop_) {
    out.flags |= SCORBOT_STATE_FLAG_ESTOP;
  }
  if (homing_state_ != SCORBOT_HOMING_IDLE && homing_state_ != SCORBOT_HOMING_DONE &&
      homing_state_ != SCORBOT_HOMING_FAULT) {
    out.flags |= SCORBOT_STATE_FLAG_HOMING;
  }

  for (size_t i = 0; i < joints_.size() && i < SCORBOT_MAX_JOINTS; ++i) {
    const auto& j = joints_[i];
    out.mode[i] = j.mode;
    out.position[i] = j.position;
    out.velocity[i] = j.velocity;
    out.effort[i] = j.effort;
    out.encoder[i] = static_cast<int32_t>(j.position * j.gear_ratio);
    uint8_t flags = 0;
    if (j.homed) {
      flags |= SCORBOT_JFLAG_HOMED;
    }
    if (j.switch_closed()) {
      flags |= SCORBOT_JFLAG_AT_LIMIT;
      out.limit_mask |= static_cast<uint16_t>(1U << i);
    }
    if (j.saturated) {
      flags |= SCORBOT_JFLAG_SATURATED;
    }
    if (j.faulted) {
      flags |= SCORBOT_JFLAG_FAULT;
    }
    out.jflags[i] = flags;
  }
}

void Simulator::reply(uint16_t opcode, uint32_t xid, uint16_t status, const void* payload,
                      size_t len, const sockaddr_in& to) {
  uint8_t buf[SCORBOT_CTRL_MAX_DATAGRAM];
  auto* hdr = reinterpret_cast<scorbot_ctrl_hdr_t*>(buf);
  std::memset(hdr, 0, sizeof(*hdr));
  hdr->magic = SCORBOT_MAGIC_CTRL;
  hdr->version = SCORBOT_PROTO_VERSION;
  hdr->opcode = opcode;
  hdr->xid = xid;
  hdr->session_id = session_.id;
  hdr->status = status;
  hdr->payload_len = static_cast<uint16_t>(len);
  if (payload != nullptr && len > 0) {
    std::memcpy(buf + sizeof(*hdr), payload, len);
  }
  (void)::sendto(ctrl_fd_, buf, sizeof(*hdr) + len, 0, reinterpret_cast<const sockaddr*>(&to),
                 sizeof(to));
}

void Simulator::handle_control(const uint8_t* buf, size_t len, const sockaddr_in& from) {
  if (len < sizeof(scorbot_ctrl_hdr_t)) {
    return;
  }
  const auto* hdr = reinterpret_cast<const scorbot_ctrl_hdr_t*>(buf);
  if (hdr->magic != SCORBOT_MAGIC_CTRL) {
    return;
  }
  if (hdr->version != SCORBOT_PROTO_VERSION) {
    reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_VERSION, nullptr, 0, from);
    return;
  }

  const uint8_t* payload = buf + sizeof(*hdr);
  const size_t payload_len = std::min<size_t>(hdr->payload_len, len - sizeof(*hdr));

  // HOME_STATUS is polled continuously during a homing run, so logging it would
  // bury everything else.
  if (verbose_ && hdr->opcode != SCORBOT_OP_HOME_STATUS) {
    std::printf("sim: ctrl opcode=%u xid=%u len=%zu\n", hdr->opcode, hdr->xid, payload_len);
  }

  // Everything except discovery and session establishment requires the caller
  // to hold the current session.
  switch (hdr->opcode) {
    case SCORBOT_OP_HELLO:
    case SCORBOT_OP_OPEN_SESSION:
      break;
    default:
      if (session_.id == 0 || hdr->session_id != session_.id) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_SESSION, nullptr, 0, from);
        return;
      }
  }

  switch (hdr->opcode) {
    case SCORBOT_OP_HELLO: {
      scorbot_hello_reply_t out{};
      out.fw_version = 0x00050000;
      out.proto_version = SCORBOT_PROTO_VERSION;
      out.uptime_ms = static_cast<uint32_t>((now_ns() - boot_ns_) / 1000000ULL);
      out.active_session = session_.id;
      out.owner_ip = session_.owner_ip;
      // A recognisable locally-administered address; the real firmware reports
      // the STMicroelectronics MAC burned into the board.
      const uint8_t mac[6] = {0x02, 0x53, 0x43, 0x4F, 0x52, 0x42};
      std::memcpy(out.robot_mac, mac, 6);
      uint8_t count = 0;
      for (size_t i = 0; i < joints_.size() && i < SCORBOT_MAX_JOINTS; ++i) {
        if (joints_[i].name.empty()) {
          continue;
        }
        std::strncpy(out.joint_name[i], joints_[i].name.c_str(), SCORBOT_NAME_LEN - 1);
        out.min_angle[i] = joints_[i].min_angle;
        out.max_angle[i] = joints_[i].max_angle;
        count = static_cast<uint8_t>(i + 1);
      }
      out.joint_count = count;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, &out, sizeof(out), from);
      return;
    }

    case SCORBOT_OP_OPEN_SESSION: {
      if (payload_len != sizeof(scorbot_open_req_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_open_req_t req{};
      std::memcpy(&req, payload, sizeof(req));

      if (session_.id != 0 && req.force == 0U && session_.owner_ip != ntohl(from.sin_addr.s_addr)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BUSY, nullptr, 0, from);
        return;
      }

      static uint32_t next_session = 1;
      session_.id = next_session++;
      session_.owner_ip = ntohl(from.sin_addr.s_addr);
      std::memcpy(session_.host_mac, req.host_mac, 6);
      session_.host_ip = req.host_ip;
      session_.host_port = req.host_port;
      session_.watchdog_ms = req.watchdog_ms != 0U ? req.watchdog_ms : 100U;
      session_.streaming = false;
      watchdog_tripped_ = false;
      fault_ = SCORBOT_FAULT_NONE;
      // Clear the echo fields: carrying a previous session's transmit timestamp
      // into the new one makes the host compute a round trip spanning both.
      echo_seq_ = 0;
      t_echo_ns_ = 0;

      scorbot_open_reply_t out{};
      out.session_id = session_.id;
      const uint8_t mac[6] = {0x02, 0x53, 0x43, 0x4F, 0x52, 0x42};
      std::memcpy(out.robot_mac, mac, 6);
      out.robot_port = SCORBOT_DATA_PORT;
      out.robot_ip = 0;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, &out, sizeof(out), from);

      if (verbose_) {
        std::printf("sim: session %u opened by %s:%u\n", session_.id,
                    scorbot::ipv4_to_string(session_.host_ip).c_str(), session_.host_port);
      }
      return;
    }

    case SCORBOT_OP_CLOSE_SESSION:
      session_ = Session{};
      all_joints_idle();
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;

    case SCORBOT_OP_SET_MODE: {
      if (payload_len != sizeof(scorbot_set_mode_req_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_set_mode_req_t req{};
      std::memcpy(&req, payload, sizeof(req));
      for (size_t i = 0; i < joints_.size() && i < SCORBOT_MAX_JOINTS; ++i) {
        if ((req.joint_mask & (1U << i)) == 0U || joints_[i].name.empty()) {
          continue;
        }
        joints_[i].mode = req.mode[i];
        if (req.mode[i] == SCORBOT_MODE_HOLD) {
          joints_[i].ramped_setpoint = joints_[i].position;
        }
      }
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;
    }

    case SCORBOT_OP_SET_GAINS: {
      if (payload_len != sizeof(scorbot_gains_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_gains_t g{};
      std::memcpy(&g, payload, sizeof(g));
      if (g.joint >= joints_.size() || joints_[g.joint].name.empty()) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_JOINT, nullptr, 0, from);
        return;
      }
      auto& j = joints_[g.joint];
      j.kp = g.kp;
      j.ki = g.ki;
      j.kd = g.kd;
      j.i_clamp = g.i_clamp;
      j.out_clamp = g.out_clamp;
      j.deadband_deg = g.deadband_deg;
      j.max_vel_dps = g.max_vel_dps;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;
    }

    case SCORBOT_OP_GET_GAINS: {
      if (payload_len != sizeof(scorbot_gains_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_gains_t req{};
      std::memcpy(&req, payload, sizeof(req));
      if (req.joint >= joints_.size() || joints_[req.joint].name.empty()) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_JOINT, nullptr, 0, from);
        return;
      }
      const auto& j = joints_[req.joint];
      scorbot_gains_t out{};
      out.joint = req.joint;
      out.kp = j.kp;
      out.ki = j.ki;
      out.kd = j.kd;
      out.i_clamp = j.i_clamp;
      out.out_clamp = j.out_clamp;
      out.deadband_deg = j.deadband_deg;
      out.max_vel_dps = j.max_vel_dps;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, &out, sizeof(out), from);
      return;
    }

    case SCORBOT_OP_SET_HOMING_CFG: {
      if (payload_len != sizeof(scorbot_homing_cfg_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_homing_cfg_t c{};
      std::memcpy(&c, payload, sizeof(c));
      if (c.joint >= joints_.size() || joints_[c.joint].name.empty()) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_JOINT, nullptr, 0, from);
        return;
      }
      auto& j = joints_[c.joint];
      j.direction = c.direction;
      j.order = c.order;
      j.enabled = c.enabled;
      j.seek_duty = c.seek_duty;
      j.creep_duty = c.creep_duty;
      j.backoff_deg = c.backoff_deg;
      j.home_offset_deg = c.home_offset_deg;
      j.timeout_ms = c.timeout_ms;
      j.stall_eps_deg = c.stall_eps_deg;
      j.stall_window_ms = c.stall_window_ms;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;
    }

    case SCORBOT_OP_GET_HOMING_CFG: {
      if (payload_len != sizeof(scorbot_homing_cfg_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_homing_cfg_t req{};
      std::memcpy(&req, payload, sizeof(req));
      if (req.joint >= joints_.size() || joints_[req.joint].name.empty()) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_JOINT, nullptr, 0, from);
        return;
      }
      const auto& j = joints_[req.joint];
      scorbot_homing_cfg_t out{};
      out.joint = req.joint;
      out.direction = j.direction;
      out.order = j.order;
      out.enabled = j.enabled;
      out.seek_duty = j.seek_duty;
      out.creep_duty = j.creep_duty;
      out.backoff_deg = j.backoff_deg;
      out.home_offset_deg = j.home_offset_deg;
      out.timeout_ms = j.timeout_ms;
      out.stall_eps_deg = j.stall_eps_deg;
      out.stall_window_ms = j.stall_window_ms;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, &out, sizeof(out), from);
      return;
    }

    case SCORBOT_OP_SET_LIMITS: {
      if (payload_len != sizeof(scorbot_limits_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_limits_t l{};
      std::memcpy(&l, payload, sizeof(l));
      if (l.joint >= joints_.size() || joints_[l.joint].name.empty()) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_JOINT, nullptr, 0, from);
        return;
      }
      joints_[l.joint].min_angle = l.min_angle;
      joints_[l.joint].max_angle = l.max_angle;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;
    }

    case SCORBOT_OP_ZERO: {
      if (payload_len != sizeof(scorbot_zero_req_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_zero_req_t z{};
      std::memcpy(&z, payload, sizeof(z));
      if (z.joint >= joints_.size() || joints_[z.joint].name.empty()) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_JOINT, nullptr, 0, from);
        return;
      }
      joints_[z.joint].position = z.angle;
      joints_[z.joint].ramped_setpoint = z.angle;
      joints_[z.joint].setpoint = z.angle;
      joints_[z.joint].homed = true;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;
    }

    case SCORBOT_OP_HOME: {
      if (payload_len != sizeof(scorbot_home_req_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_home_req_t req{};
      std::memcpy(&req, payload, sizeof(req));
      if (req.confirm != SCORBOT_HOME_CONFIRM) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_NOT_CONFIRMED, nullptr, 0, from);
        return;
      }
      homing_mask_ = req.joint_mask;
      homing_state_ = SCORBOT_HOMING_PARK;
      homing_started_ms_ = now_ms();
      phase_started_ns_ = now_ns();
      fault_ = SCORBOT_FAULT_NONE;
      for (auto& j : joints_) {
        j.home_phase = SCORBOT_HOMING_IDLE;
        j.faulted = false;
      }
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      if (verbose_) {
        std::printf("sim: homing started, mask=0x%02x\n", homing_mask_);
      }
      return;
    }

    case SCORBOT_OP_HOME_STATUS: {
      scorbot_home_status_t out{};
      out.state = homing_state_;
      out.joint = homing_joint_;
      out.order = homing_order_;
      out.error = fault_;
      uint8_t mask = 0;
      for (size_t i = 0; i < joints_.size() && i < SCORBOT_MAX_JOINTS; ++i) {
        if (joints_[i].homed) {
          mask |= static_cast<uint8_t>(1U << i);
        }
      }
      out.homed_mask = mask;
      out.elapsed_ms = now_ms() - homing_started_ms_;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, &out, sizeof(out), from);
      return;
    }

    case SCORBOT_OP_STREAM_START: {
      if (payload_len != sizeof(scorbot_stream_req_t)) {
        reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_LENGTH, nullptr, 0, from);
        return;
      }
      scorbot_stream_req_t req{};
      std::memcpy(&req, payload, sizeof(req));
      session_.state_period_us = req.state_period_us != 0U ? req.state_period_us : 1000U;
      session_.watchdog_ms = req.watchdog_ms != 0U ? req.watchdog_ms : 100U;
      session_.streaming = true;
      last_pose_ns_ = now_ns();
      watchdog_tripped_ = false;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      if (verbose_) {
        std::printf("sim: streaming to %s:%u every %u us\n",
                    scorbot::ipv4_to_string(session_.host_ip).c_str(), session_.host_port,
                    session_.state_period_us);
      }
      return;
    }

    case SCORBOT_OP_STREAM_STOP:
      session_.streaming = false;
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      return;

    case SCORBOT_OP_ABORT:
      estop_ = true;
      homing_state_ = SCORBOT_HOMING_IDLE;
      homing_joint_ = 0xFF;
      all_joints_idle();
      reply(hdr->opcode, hdr->xid, SCORBOT_OK, nullptr, 0, from);
      if (verbose_) {
        std::printf("sim: ABORT - all joints idle\n");
      }
      return;

    default:
      reply(hdr->opcode, hdr->xid, SCORBOT_ERR_BAD_OPCODE, nullptr, 0, from);
      return;
  }
}

void Simulator::handle_pose(const uint8_t* buf, size_t len) {
  if (len != sizeof(scorbot_pose_t)) {
    return;
  }
  scorbot_pose_t pose{};
  std::memcpy(&pose, buf, sizeof(pose));

  if (pose.magic != SCORBOT_MAGIC_POSE || pose.version != SCORBOT_PROTO_VERSION) {
    return;
  }
  if (session_.id == 0 || pose.session_id != session_.id) {
    return;
  }

  last_pose_ns_ = now_ns();
  if (watchdog_tripped_) {
    watchdog_tripped_ = false;
    if (fault_ == SCORBOT_FAULT_WATCHDOG) {
      fault_ = SCORBOT_FAULT_NONE;
    }
  }
  echo_seq_ = pose.seq;
  t_echo_ns_ = pose.t_tx_ns;

  if ((pose.flags & SCORBOT_POSE_FLAG_ESTOP) != 0U) {
    estop_ = true;
    all_joints_idle();
    return;
  }
  estop_ = false;

  // A keepalive only feeds the watchdog; its payload is deliberately ignored.
  const bool keepalive = (pose.flags & SCORBOT_POSE_FLAG_KEEPALIVE) != 0U;
  const bool homing_active = homing_state_ != SCORBOT_HOMING_IDLE &&
                             homing_state_ != SCORBOT_HOMING_DONE &&
                             homing_state_ != SCORBOT_HOMING_FAULT;

  if (!keepalive && !homing_active) {
    for (size_t i = 0; i < joints_.size() && i < SCORBOT_MAX_JOINTS; ++i) {
      auto& j = joints_[i];
      if (j.name.empty()) {
        continue;
      }
      const uint8_t want = pose.mode[i];
      if (want == SCORBOT_MODE_HOLD && j.mode != SCORBOT_MODE_HOLD) {
        j.ramped_setpoint = j.position;
      }
      j.mode = want;
      if (want != SCORBOT_MODE_HOLD) {
        j.setpoint = pose.setpoint[i];
      }
    }
  }

  if ((pose.flags & SCORBOT_POSE_FLAG_REPLY_NOW) != 0U && session_.streaming) {
    // Answer this exchange immediately rather than on the next tick, so the
    // host can time a real round trip without tick phase in the measurement.
    scorbot_state_t state{};
    ++tick_seq_;
    fill_state(state);
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = htonl(session_.host_ip);
    to.sin_port = htons(session_.host_port);
    (void)::sendto(data_fd_, &state, sizeof(state), 0, reinterpret_cast<sockaddr*>(&to),
                   sizeof(to));
  }
}

int Simulator::run(uint16_t ctrl_port, uint16_t data_port) {
  // Line-buffer so the log is readable live even when redirected to a file.
  (void)setvbuf(stdout, nullptr, _IOLBF, 0);

  ctrl_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  data_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (ctrl_fd_ < 0 || data_fd_ < 0) {
    std::fprintf(stderr, "sim: socket() failed: %s\n", std::strerror(errno));
    return 1;
  }

  int one = 1;
  (void)::setsockopt(ctrl_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  (void)::setsockopt(data_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;

  addr.sin_port = htons(ctrl_port);
  if (::bind(ctrl_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::fprintf(stderr, "sim: bind control port %u failed: %s\n", ctrl_port,
                 std::strerror(errno));
    return 1;
  }
  addr.sin_port = htons(data_port);
  if (::bind(data_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::fprintf(stderr, "sim: bind data port %u failed: %s\n", data_port, std::strerror(errno));
    return 1;
  }

  std::printf("scorbot_sim: control on udp/%u, data on udp/%u, %zu joints\n", ctrl_port, data_port,
              joints_.size());
  std::printf("             homing order: gripper, wrists, elbow, lift, pan, slide\n");

  uint64_t last_tick = now_ns();
  uint64_t last_state_tx = now_ns();
  uint8_t buf[2048];

  while (true) {
    pollfd fds[2] = {{ctrl_fd_, POLLIN, 0}, {data_fd_, POLLIN, 0}};
    (void)::poll(fds, 2, 1);

    if ((fds[0].revents & POLLIN) != 0) {
      sockaddr_in from{};
      socklen_t from_len = sizeof(from);
      const ssize_t n =
          ::recvfrom(ctrl_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &from_len);
      if (n > 0) {
        handle_control(buf, static_cast<size_t>(n), from);
      }
    }

    if ((fds[1].revents & POLLIN) != 0) {
      // Drain every queued pose so a burst does not build a backlog.
      while (true) {
        const ssize_t n = ::recv(data_fd_, buf, sizeof(buf), MSG_DONTWAIT);
        if (n <= 0) {
          break;
        }
        handle_pose(buf, static_cast<size_t>(n));
      }
    }

    const uint64_t now = now_ns();

    const double dt = static_cast<double>(now - last_tick) / 1e9;
    if (dt >= 0.001) {
      tick(dt);
      last_tick = now;
    }

    if (session_.streaming) {
      // Watchdog: if the host stops talking, fall back to a safe state.
      //
      // Suspended while homing runs. Homing is driven entirely by the firmware
      // and takes seconds, during which the host has nothing to stream; faulting
      // it out mid-sequence would abandon the arm at a limit switch.
      const bool homing_active = homing_state_ != SCORBOT_HOMING_IDLE &&
                                 homing_state_ != SCORBOT_HOMING_DONE &&
                                 homing_state_ != SCORBOT_HOMING_FAULT;
      const uint64_t silence_ms = (now - last_pose_ns_) / 1000000ULL;
      if (homing_active) {
        last_pose_ns_ = now;
      } else if (!watchdog_tripped_ && silence_ms > session_.watchdog_ms) {
        watchdog_tripped_ = true;
        fault_ = SCORBOT_FAULT_WATCHDOG;
        for (auto& j : joints_) {
          if (j.mode != SCORBOT_MODE_IDLE) {
            j.mode = SCORBOT_MODE_HOLD;
            j.ramped_setpoint = j.position;
          }
        }
        if (verbose_) {
          std::printf("sim: watchdog tripped after %llu ms of silence\n",
                      static_cast<unsigned long long>(silence_ms));
        }
      }

      if ((now - last_state_tx) / 1000ULL >= session_.state_period_us) {
        last_state_tx = now;
        ++tick_seq_;
        scorbot_state_t state{};
        fill_state(state);
        sockaddr_in to{};
        to.sin_family = AF_INET;
        to.sin_addr.s_addr = htonl(session_.host_ip);
        to.sin_port = htons(session_.host_port);
        (void)::sendto(data_fd_, &state, sizeof(state), 0, reinterpret_cast<sockaddr*>(&to),
                       sizeof(to));
      }
    }
  }
}

void print_usage() {
  std::printf(
      "usage: scorbot_sim [options]\n"
      "  --ctrl-port N   control plane port (default %u)\n"
      "  --data-port N   data plane port (default %u)\n"
      "  --verbose       log control requests and homing transitions\n",
      SCORBOT_CTRL_PORT, SCORBOT_DATA_PORT);
}

}  // namespace

int main(int argc, char** argv) {
  uint16_t ctrl_port = SCORBOT_CTRL_PORT;
  uint16_t data_port = SCORBOT_DATA_PORT;
  bool verbose = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ctrl-port" && i + 1 < argc) {
      ctrl_port = static_cast<uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--data-port" && i + 1 < argc) {
      data_port = static_cast<uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--verbose" || arg == "-v") {
      verbose = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", arg.c_str());
      print_usage();
      return 1;
    }
  }

  Simulator sim(verbose);
  return sim.run(ctrl_port, data_port);
}
