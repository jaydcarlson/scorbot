// Holoscan 5 operators for the Scorbot link.
//
// ScorbotTxOp takes a list of joint positions and pushes it to the firmware.
// ScorbotRxOp pulls telemetry back off the wire and emits it into the graph.
//
// Both are thin: all protocol and transport work lives in ScorbotLink, which
// knows nothing about Holoscan. These operators only adapt it to the graph.
//
// Binding the link:
//   The SDK's operator factory reconstructs operators from copy-captured value
//   arguments, and GraphOwnedCapture explicitly rejects pointers and smart
//   pointers, so a live ScorbotLink cannot be passed through graph.op<>().
//   Instead main() owns the link and binds it into a numeric slot before
//   compile, and every reconstructed instance resolves that same slot. This is
//   the pattern the SDK's own hardware capture operators use.

#ifndef SCORBOT_OPS_HPP_
#define SCORBOT_OPS_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string_view>

#include <holoscan/channel/pod_type.hpp>
#include <holoscan/core/execution_context.hpp>
#include <holoscan/core/operator.hpp>
#include <holoscan/core/operator_spec.hpp>
#include <holoscan/core/temporal_contract.hpp>

#include "scorbot/scorbot_link.hpp"
#include "scorbot_proto.h"

namespace scorbot {

// Graph-level command payload: the list of joint positions the user asked for.
//
// Fields are ordered by decreasing alignment so the struct tiles itself with no
// implicit padding, which the Holoscan POD contract requires.
struct JointTargets {
  std::uint32_t count = 0;  // how many entries of setpoint/mode are meaningful
  std::uint32_t flags = 0;  // SCORBOT_POSE_FLAG_* passed straight through
  float setpoint[SCORBOT_MAX_JOINTS] = {};
  std::uint8_t mode[SCORBOT_MAX_JOINTS] = {};
};

static_assert(sizeof(JointTargets) == 48, "JointTargets must stay padding-free");
static_assert(std::is_standard_layout_v<JointTargets>);
static_assert(std::is_trivially_copyable_v<JointTargets>);

}  // namespace scorbot

namespace holoscan {

template <>
struct ChannelTypeIdentity<scorbot::JointTargets> {
  static constexpr std::string_view name = "scorbot.JointTargets";
  static constexpr std::uint32_t version = 1U;
};

template <>
struct ChannelPodLayout<scorbot::JointTargets> {
  static constexpr auto fields = pod_fields(
      pod_field("count", &scorbot::JointTargets::count, offsetof(scorbot::JointTargets, count)),
      pod_field("flags", &scorbot::JointTargets::flags, offsetof(scorbot::JointTargets, flags)),
      pod_field("setpoint", &scorbot::JointTargets::setpoint,
                offsetof(scorbot::JointTargets, setpoint)),
      pod_field("mode", &scorbot::JointTargets::mode, offsetof(scorbot::JointTargets, mode)));
};

// The wire structs ride the graph unchanged. They are packed, so their fields
// already tile them exactly with no padding for the contract to trip over.
template <>
struct ChannelTypeIdentity<scorbot_pose_t> {
  static constexpr std::string_view name = "scorbot.RobotPose";
  static constexpr std::uint32_t version = SCORBOT_PROTO_VERSION;
};

template <>
struct ChannelPodLayout<scorbot_pose_t> {
  static constexpr auto fields = pod_fields(
      pod_field("magic", &scorbot_pose_t::magic, offsetof(scorbot_pose_t, magic)),
      pod_field("version", &scorbot_pose_t::version, offsetof(scorbot_pose_t, version)),
      pod_field("flags", &scorbot_pose_t::flags, offsetof(scorbot_pose_t, flags)),
      pod_field("session_id", &scorbot_pose_t::session_id, offsetof(scorbot_pose_t, session_id)),
      pod_field("seq", &scorbot_pose_t::seq, offsetof(scorbot_pose_t, seq)),
      pod_field("t_tx_ns", &scorbot_pose_t::t_tx_ns, offsetof(scorbot_pose_t, t_tx_ns)),
      pod_field("mode", &scorbot_pose_t::mode, offsetof(scorbot_pose_t, mode)),
      pod_field("setpoint", &scorbot_pose_t::setpoint, offsetof(scorbot_pose_t, setpoint)));
};

template <>
struct ChannelTypeIdentity<scorbot_state_t> {
  static constexpr std::string_view name = "scorbot.RobotState";
  static constexpr std::uint32_t version = SCORBOT_PROTO_VERSION;
};

template <>
struct ChannelPodLayout<scorbot_state_t> {
  static constexpr auto fields = pod_fields(
      pod_field("magic", &scorbot_state_t::magic, offsetof(scorbot_state_t, magic)),
      pod_field("version", &scorbot_state_t::version, offsetof(scorbot_state_t, version)),
      pod_field("flags", &scorbot_state_t::flags, offsetof(scorbot_state_t, flags)),
      pod_field("session_id", &scorbot_state_t::session_id, offsetof(scorbot_state_t, session_id)),
      pod_field("seq", &scorbot_state_t::seq, offsetof(scorbot_state_t, seq)),
      pod_field("t_echo_ns", &scorbot_state_t::t_echo_ns, offsetof(scorbot_state_t, t_echo_ns)),
      pod_field("echo_seq", &scorbot_state_t::echo_seq, offsetof(scorbot_state_t, echo_seq)),
      pod_field("t_fw_us", &scorbot_state_t::t_fw_us, offsetof(scorbot_state_t, t_fw_us)),
      pod_field("limit_mask", &scorbot_state_t::limit_mask, offsetof(scorbot_state_t, limit_mask)),
      pod_field("homing_state", &scorbot_state_t::homing_state,
                offsetof(scorbot_state_t, homing_state)),
      pod_field("homing_joint", &scorbot_state_t::homing_joint,
                offsetof(scorbot_state_t, homing_joint)),
      pod_field("fault", &scorbot_state_t::fault, offsetof(scorbot_state_t, fault)),
      pod_field("reserved", &scorbot_state_t::reserved, offsetof(scorbot_state_t, reserved)),
      pod_field("missed_deadlines", &scorbot_state_t::missed_deadlines,
                offsetof(scorbot_state_t, missed_deadlines)),
      pod_field("mode", &scorbot_state_t::mode, offsetof(scorbot_state_t, mode)),
      pod_field("jflags", &scorbot_state_t::jflags, offsetof(scorbot_state_t, jflags)),
      pod_field("position", &scorbot_state_t::position, offsetof(scorbot_state_t, position)),
      pod_field("velocity", &scorbot_state_t::velocity, offsetof(scorbot_state_t, velocity)),
      pod_field("effort", &scorbot_state_t::effort, offsetof(scorbot_state_t, effort)),
      pod_field("encoder", &scorbot_state_t::encoder, offsetof(scorbot_state_t, encoder)));
};

}  // namespace holoscan

namespace scorbot {

// Slot registry shared by both operators, so a graph can carry more than one
// robot without either operator owning the link.
class LinkRegistry {
 public:
  static constexpr std::size_t kMaxSlots = 4;

  static void bind(ScorbotLink& link, std::uint32_t slot = 0) {
    if (slot >= kMaxSlots) {
      throw std::out_of_range("ScorbotLink binding slot out of range");
    }
    slots_[slot] = &link;
  }

  static ScorbotLink* resolve(std::uint32_t slot) {
    return slot < kMaxSlots ? slots_[slot] : nullptr;
  }

 private:
  static inline std::array<ScorbotLink*, kMaxSlots> slots_{};
};

// ---------------------------------------------------------------------------
// ScorbotTxOp - joint positions in, UDP on the wire out.
// ---------------------------------------------------------------------------
class ScorbotTxOp final : public holoscan::Operator<> {
 public:
  explicit ScorbotTxOp(std::uint32_t slot = 0) : slot_(slot) {}

  void setup(holoscan::OperatorSpec& spec) override {
    spec.input(targets, "targets").queue_depth(4U);
    spec.output(sent, "sent").max_emits_per_compute(1U);
  }

  [[nodiscard]] holoscan::Contract contract() const override {
    holoscan::Contract result;
    // Fire the instant a setpoint shows up: the whole point is to get it onto
    // the wire without waiting for a tick boundary.
    result.trigger(holoscan::OnEach{targets});
    return result;
  }

  void start() override {
    link_ = LinkRegistry::resolve(slot_);
    if (link_ == nullptr) {
      throw std::runtime_error("ScorbotTxOp: no ScorbotLink bound to its slot");
    }
  }

  [[nodiscard]] holoscan::expected<void, holoscan::Error> compute(
      holoscan::ExecutionContext&) override {
    auto sample = targets.receive();
    if (!sample) {
      return holoscan::make_unexpected(std::move(sample).error());
    }
    const JointTargets& want = sample->data;

    scorbot_pose_t pose{};
    pose.flags = static_cast<std::uint16_t>(want.flags);
    const std::uint32_t n = want.count < SCORBOT_MAX_JOINTS ? want.count : SCORBOT_MAX_JOINTS;
    for (std::uint32_t j = 0; j < n; ++j) {
      pose.mode[j] = want.mode[j];
      pose.setpoint[j] = want.setpoint[j];
    }
    // Joints the producer did not speak for keep holding rather than go limp.
    for (std::uint32_t j = n; j < SCORBOT_MAX_JOINTS; ++j) {
      pose.mode[j] = SCORBOT_MODE_HOLD;
    }

    if (!link_->send_pose(pose)) {
      ++dropped_;
      // A failed transmit is not fatal: the next setpoint supersedes this one
      // anyway, and stopping the graph over one dropped datagram would be worse.
      return {};
    }
    ++transmitted_;
    link_->poll_send();

    // The echo port is observability only; the robot already has the command.
    // If nothing is draining it the loan pool fills, and failing the graph over
    // a backed-up logging port would be the wrong trade.
    if (auto echoed = sent.emit(pose); !echoed) {
      ++echo_dropped_;
    }
    return {};
  }

  holoscan::Input<JointTargets> targets;
  holoscan::Output<scorbot_pose_t> sent;

  [[nodiscard]] std::uint64_t transmitted() const { return transmitted_; }
  [[nodiscard]] std::uint64_t dropped() const { return dropped_; }
  [[nodiscard]] std::uint64_t echo_dropped() const { return echo_dropped_; }

 private:
  std::uint32_t slot_;
  ScorbotLink* link_ = nullptr;
  std::uint64_t transmitted_ = 0;
  std::uint64_t dropped_ = 0;
  std::uint64_t echo_dropped_ = 0;
};

// ---------------------------------------------------------------------------
// ScorbotRxOp - telemetry off the wire and into the graph.
// ---------------------------------------------------------------------------
class ScorbotRxOp final : public holoscan::Operator<> {
 public:
  // poll_period_us should be comfortably shorter than the firmware's state
  // period so telemetry is picked up promptly rather than aliased.
  explicit ScorbotRxOp(std::uint32_t slot = 0, std::int64_t poll_period_us = 200)
      : slot_(slot), poll_period_ns_(poll_period_us * 1000) {}

  void setup(holoscan::OperatorSpec& spec) override {
    spec.output(state, "state").max_emits_per_compute(1U);
  }

  [[nodiscard]] holoscan::Contract contract() const override {
    holoscan::Contract result;
    // EA1 cannot bind a socket descriptor or a completion queue to a readiness
    // trigger, so poll on a clock. This is the same approach the SDK's own
    // network camera source takes.
    result.trigger(holoscan::OnClock{.period = std::chrono::nanoseconds{poll_period_ns_}});
    return result;
  }

  void start() override {
    link_ = LinkRegistry::resolve(slot_);
    if (link_ == nullptr) {
      throw std::runtime_error("ScorbotRxOp: no ScorbotLink bound to its slot");
    }
  }

  [[nodiscard]] holoscan::expected<void, holoscan::Error> compute(
      holoscan::ExecutionContext& context) override {
    link_->poll_send();

    scorbot_state_t latest{};
    if (!link_->poll_state(latest)) {
      return {};  // nothing new this tick; emitting a stale copy would mislead
    }
    ++received_;
    return state.emit(latest, holoscan::EmitOptions{.capture_time = context.activation_time()});
  }

  holoscan::Output<scorbot_state_t> state;

  [[nodiscard]] std::uint64_t received() const { return received_; }

 private:
  std::uint32_t slot_;
  std::int64_t poll_period_ns_;
  ScorbotLink* link_ = nullptr;
  std::uint64_t received_ = 0;
};

}  // namespace scorbot

#endif  // SCORBOT_OPS_HPP_
