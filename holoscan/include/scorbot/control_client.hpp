// Control plane client: strict call/response over an ordinary UDP socket.
//
// This side of the protocol carries session setup, motor modes, PID gains,
// homing and abort. Correctness matters far more than latency here, so it uses
// the kernel stack, matches replies by transaction id and retries on timeout.
// Every operation is idempotent, which is what makes blind retries safe.
//
// Homing is asynchronous: HOME returns as soon as the firmware accepts it, and
// progress is polled with home_status() or watched in the telemetry stream.

#ifndef SCORBOT_CONTROL_CLIENT_HPP_
#define SCORBOT_CONTROL_CLIENT_HPP_

#include <cstdint>
#include <string>

#include "scorbot/status.hpp"
#include "scorbot_proto.h"

namespace scorbot {

const char* status_to_string(uint16_t status);

class ControlClient {
 public:
  struct Config {
    uint32_t robot_ip = 0;
    uint16_t robot_port = SCORBOT_CTRL_PORT;
    std::string ifname;
    uint32_t timeout_ms = 250;
    int retries = 4;
  };

  ControlClient() = default;
  ~ControlClient();

  ControlClient(const ControlClient&) = delete;
  ControlClient& operator=(const ControlClient&) = delete;

  Status open(const Config& cfg);
  void close();
  [[nodiscard]] bool is_open() const { return fd_ >= 0; }

  void set_session(uint32_t session_id) { session_id_ = session_id; }
  [[nodiscard]] uint32_t session() const { return session_id_; }

  // Send one request and wait for the matching reply, retrying on timeout.
  // reply_len may be null when no payload is expected.
  Status transact(uint16_t opcode, const void* payload, size_t payload_len, void* reply,
                  size_t reply_cap, size_t* reply_len);

  Result<scorbot_hello_reply_t> hello();
  Result<scorbot_open_reply_t> open_session(const scorbot_open_req_t& req);
  Status close_session();

  Status set_mode(const uint8_t modes[SCORBOT_MAX_JOINTS], uint8_t joint_mask);
  Status set_gains(const scorbot_gains_t& gains);
  Result<scorbot_gains_t> get_gains(uint8_t joint);
  Status set_homing_cfg(const scorbot_homing_cfg_t& cfg);
  Result<scorbot_homing_cfg_t> get_homing_cfg(uint8_t joint);
  Status set_limits(const scorbot_limits_t& limits);
  Status zero(uint8_t joint, float angle);

  // Kicks off the homing sequence and returns as soon as the firmware accepts
  // it. Refused unless confirm is SCORBOT_HOME_CONFIRM.
  Status home(uint8_t joint_mask, uint32_t confirm);
  Result<scorbot_home_status_t> home_status();

  Status stream_start(uint32_t state_period_us, uint32_t watchdog_ms);
  Status stream_stop();
  Status abort();

 private:
  int fd_ = -1;
  Config cfg_;
  uint32_t session_id_ = 0;
  uint32_t next_xid_ = 1;
};

}  // namespace scorbot

#endif  // SCORBOT_CONTROL_CLIENT_HPP_
