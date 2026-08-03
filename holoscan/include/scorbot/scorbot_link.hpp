// ScorbotLink - the comms layer.
//
// Owns both planes: a call/response control channel for configuration and
// homing, and a free-running data plane for setpoints and telemetry. The data
// plane runs over a pluggable Transport, so the same object drives either an
// ordinary UDP socket or a ConnectX raw packet queue pair.
//
// Deliberately free of any Holoscan dependency. The Holoscan operators sit on
// top of this, and the command line tools and web UI use it directly, which is
// what keeps that separation honest.

#ifndef SCORBOT_LINK_HPP_
#define SCORBOT_LINK_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "scorbot/control_client.hpp"
#include "scorbot/status.hpp"
#include "scorbot/transport.hpp"
#include "scorbot_proto.h"

namespace scorbot {

uint64_t monotonic_ns();

// Rolling round-trip statistics derived from the echo fields in the telemetry
// stream. Percentiles are computed over retained samples.
class LatencyStats {
 public:
  void add_us(double us);
  void reset();

  [[nodiscard]] size_t count() const { return samples_.size(); }
  [[nodiscard]] double min_us() const;
  [[nodiscard]] double mean_us() const;
  [[nodiscard]] double percentile_us(double p) const;
  [[nodiscard]] double max_us() const;

 private:
  std::vector<double> samples_;
  double sum_ = 0.0;
};

struct LinkStats {
  uint64_t poses_sent = 0;
  uint64_t poses_failed = 0;
  uint64_t states_received = 0;
  uint64_t states_dropped = 0;  // gaps in the firmware sequence counter
  uint64_t bad_packets = 0;     // wrong magic, version, session or size
  uint64_t watchdog_trips = 0;
  double last_rtt_us = 0.0;
};

struct JointInfo {
  std::string name;
  float min_angle = 0.0F;
  float max_angle = 0.0F;
};

struct RobotInfo {
  uint32_t fw_version = 0;
  uint32_t proto_version = 0;
  uint32_t uptime_ms = 0;
  uint32_t active_session = 0;
  uint32_t owner_ip = 0;
  MacAddr robot_mac{};
  uint8_t joint_count = 0;
  std::array<JointInfo, SCORBOT_MAX_JOINTS> joints{};
};

class ScorbotLink {
 public:
  struct Config {
    // Where the robot is.
    std::string robot_ip = "192.168.0.161";
    uint16_t robot_data_port = SCORBOT_DATA_PORT;
    uint16_t robot_ctrl_port = SCORBOT_CTRL_PORT;

    // Which local interface to use. The ibverbs backend also needs the RDMA
    // device name for the same port.
    std::string ifname = "enp2s0f1np1";
    std::string ibv_device = "mlx5_1";
    uint8_t ibv_port = 1;

    TransportKind transport = TransportKind::kSocket;

    // Local data plane port. 0 lets the kernel choose on the socket backend;
    // the ibverbs backend needs a fixed one for its flow rule.
    uint16_t local_data_port = 6011;

    uint32_t watchdog_ms = 100;
    uint32_t state_period_us = 1000;
    uint32_t ctrl_timeout_ms = 250;
    int ctrl_retries = 4;
    uint32_t busy_poll_us = 0;
    bool hugepages = false;

    // Take the streaming session even if another host already holds it.
    bool force_session = false;
  };

  ScorbotLink() = default;
  ~ScorbotLink();

  ScorbotLink(const ScorbotLink&) = delete;
  ScorbotLink& operator=(const ScorbotLink&) = delete;

  // Control plane only. Safe to call without claiming the data plane, which is
  // what the web UI's observe mode uses.
  Status open_control(const Config& cfg);

  // Full bring-up: control plane, session, transport, then arm the stream.
  Status open(const Config& cfg);
  void close();

  [[nodiscard]] bool is_open() const { return streaming_; }
  [[nodiscard]] bool control_open() const { return control_.is_open(); }
  [[nodiscard]] uint32_t session_id() const { return session_id_; }
  [[nodiscard]] const RobotInfo& info() const { return info_; }
  [[nodiscard]] const Config& config() const { return cfg_; }
  [[nodiscard]] const char* transport_name() const;

  // ---- Control plane -----------------------------------------------------
  Result<RobotInfo> query_info();
  Status set_mode_all(uint8_t mode);
  Status set_mode(uint8_t joint, uint8_t mode);
  Status set_gains(const scorbot_gains_t& gains);
  Result<scorbot_gains_t> get_gains(uint8_t joint);
  Status set_homing_cfg(const scorbot_homing_cfg_t& cfg);
  Result<scorbot_homing_cfg_t> get_homing_cfg(uint8_t joint);
  Status set_limits(uint8_t joint, float min_angle, float max_angle);
  Status zero(uint8_t joint, float angle);
  Status abort();

  // Starts homing and returns immediately. joint_mask of 0xFF homes everything
  // in the firmware's configured order.
  Status start_homing(uint8_t joint_mask = 0xFF);
  Result<scorbot_home_status_t> homing_status();
  // Convenience wrapper that polls until the sequence finishes or faults.
  Status home_blocking(uint8_t joint_mask, uint32_t timeout_ms);

  // ---- Data plane (hot path, allocation free) ----------------------------

  // Fills in magic, version, session and sequence, then transmits.
  bool send_pose(scorbot_pose_t& pose);

  // Convenience for the common case of position control on every joint.
  bool send_positions(const float* angles, size_t count, uint8_t mode = SCORBOT_MODE_POSITION);

  // Copies the newest valid telemetry packet into out. Returns false when
  // nothing new has arrived. Drains everything queued so the caller always
  // sees the freshest state rather than working through a backlog.
  bool poll_state(scorbot_state_t& out);

  // Reap transmit completions. Only the ibverbs backend needs this.
  void poll_send();

  // Keeps the firmware watchdog fed without changing any setpoint.
  bool send_keepalive();

  // Round trip measured by asking the firmware to reply immediately rather
  // than on its own tick, so the number is not polluted by tick phase.
  Result<double> measure_rtt_us(uint32_t timeout_ms = 50);

  [[nodiscard]] const LinkStats& stats() const { return stats_; }
  [[nodiscard]] LatencyStats& latency() { return latency_; }
  void reset_stats();

  [[nodiscard]] const scorbot_state_t& last_state() const { return last_state_; }
  [[nodiscard]] bool have_state() const { return have_state_; }

 private:
  Status bring_up_transport();
  bool validate_state(const scorbot_state_t& state);

  Config cfg_;
  ControlClient control_;
  std::unique_ptr<Transport> transport_;

  RobotInfo info_{};
  uint32_t session_id_ = 0;
  uint32_t tx_seq_ = 0;
  uint32_t last_rx_seq_ = 0;
  bool have_rx_seq_ = false;
  bool streaming_ = false;

  scorbot_state_t last_state_{};
  bool have_state_ = false;

  LinkStats stats_{};
  LatencyStats latency_;

  MacAddr local_mac_{};
  MacAddr robot_mac_{};
  uint32_t local_ip_ = 0;
  uint32_t robot_ip_ = 0;
};

}  // namespace scorbot

#endif  // SCORBOT_LINK_HPP_
