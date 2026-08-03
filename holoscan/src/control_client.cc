#include "scorbot/control_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

#include "scorbot/transport.hpp"

namespace scorbot {

const char* status_to_string(uint16_t status) {
  switch (status) {
    case SCORBOT_OK:
      return "ok";
    case SCORBOT_ERR_BAD_MAGIC:
      return "bad magic";
    case SCORBOT_ERR_BAD_VERSION:
      return "protocol version mismatch";
    case SCORBOT_ERR_BAD_OPCODE:
      return "unsupported opcode";
    case SCORBOT_ERR_BAD_LENGTH:
      return "bad payload length";
    case SCORBOT_ERR_BAD_SESSION:
      return "session mismatch";
    case SCORBOT_ERR_BUSY:
      return "busy (another host owns the session, or homing is already running)";
    case SCORBOT_ERR_BAD_JOINT:
      return "joint index out of range";
    case SCORBOT_ERR_BAD_PARAM:
      return "parameter out of range";
    case SCORBOT_ERR_NOT_HOMED:
      return "joint is not homed";
    case SCORBOT_ERR_NOT_CONFIRMED:
      return "missing confirmation token";
    case SCORBOT_ERR_FAULTED:
      return "robot is faulted";
    case SCORBOT_ERR_TIMEOUT:
      return "timed out";
    default:
      return "unknown error";
  }
}

ControlClient::~ControlClient() { close(); }

Status ControlClient::open(const Config& cfg) {
  close();
  cfg_ = cfg;

  if (cfg_.robot_ip == 0) {
    return Status::error("control client needs the robot IP");
  }

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return Status::error(std::string("control socket() failed: ") + std::strerror(errno), errno);
  }

  if (!cfg_.ifname.empty()) {
    (void)::setsockopt(fd_, SOL_SOCKET, SO_BINDTODEVICE, cfg_.ifname.c_str(),
                       static_cast<socklen_t>(cfg_.ifname.size()));
  }

  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  peer.sin_addr.s_addr = htonl(cfg_.robot_ip);
  peer.sin_port = htons(cfg_.robot_port);
  if (::connect(fd_, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) != 0) {
    const int err = errno;
    close();
    return Status::error(std::string("control connect() failed: ") + std::strerror(err), err);
  }

  return Status::ok();
}

void ControlClient::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  session_id_ = 0;
}

Status ControlClient::transact(uint16_t opcode, const void* payload, size_t payload_len,
                               void* reply, size_t reply_cap, size_t* reply_len) {
  if (fd_ < 0) {
    return Status::error("control client is not open");
  }
  if (payload_len > SCORBOT_CTRL_MAX_PAYLOAD) {
    return Status::error("control payload too large");
  }

  const uint32_t xid = next_xid_++;

  uint8_t request[SCORBOT_CTRL_MAX_DATAGRAM];
  auto* hdr = reinterpret_cast<scorbot_ctrl_hdr_t*>(request);
  std::memset(hdr, 0, sizeof(*hdr));
  hdr->magic = SCORBOT_MAGIC_CTRL;
  hdr->version = SCORBOT_PROTO_VERSION;
  hdr->opcode = opcode;
  hdr->xid = xid;
  hdr->session_id = session_id_;
  hdr->status = 0;
  hdr->payload_len = static_cast<uint16_t>(payload_len);
  if (payload != nullptr && payload_len > 0) {
    std::memcpy(request + sizeof(*hdr), payload, payload_len);
  }
  const size_t request_len = sizeof(*hdr) + payload_len;

  uint8_t response[SCORBOT_CTRL_MAX_DATAGRAM];

  for (int attempt = 0; attempt <= cfg_.retries; ++attempt) {
    if (::send(fd_, request, request_len, 0) != static_cast<ssize_t>(request_len)) {
      return Status::error(std::string("control send failed: ") + std::strerror(errno), errno);
    }

    // Keep reading until the deadline: a late reply to a previous attempt can
    // still be sitting in the socket, and must be skipped rather than mistaken
    // for this one.
    pollfd pfd{fd_, POLLIN, 0};
    int remaining_ms = static_cast<int>(cfg_.timeout_ms);
    while (remaining_ms > 0) {
      const auto start = ::poll(&pfd, 1, remaining_ms);
      if (start <= 0) {
        break;  // timed out or interrupted; fall through to the next attempt
      }
      const ssize_t n = ::recv(fd_, response, sizeof(response), 0);
      if (n < static_cast<ssize_t>(sizeof(scorbot_ctrl_hdr_t))) {
        continue;
      }
      const auto* rhdr = reinterpret_cast<const scorbot_ctrl_hdr_t*>(response);
      if (rhdr->magic != SCORBOT_MAGIC_CTRL || rhdr->xid != xid || rhdr->opcode != opcode) {
        continue;  // stale or unrelated datagram
      }
      if (rhdr->status != SCORBOT_OK) {
        return Status::error(std::string("robot rejected ") + std::to_string(opcode) + ": " +
                                 status_to_string(rhdr->status),
                             rhdr->status);
      }

      const size_t got = rhdr->payload_len;
      if (sizeof(*rhdr) + got > static_cast<size_t>(n)) {
        return Status::error("truncated control reply");
      }
      if (reply != nullptr && got > 0) {
        if (got > reply_cap) {
          return Status::error("control reply larger than caller's buffer");
        }
        std::memcpy(reply, response + sizeof(*rhdr), got);
      }
      if (reply_len != nullptr) {
        *reply_len = got;
      }
      return Status::ok();
    }
  }

  return Status::error("no reply from robot after " + std::to_string(cfg_.retries + 1) +
                           " attempts (opcode " + std::to_string(opcode) + ")",
                       SCORBOT_ERR_TIMEOUT);
}

Result<scorbot_hello_reply_t> ControlClient::hello() {
  scorbot_hello_reply_t out{};
  size_t got = 0;
  if (Status s = transact(SCORBOT_OP_HELLO, nullptr, 0, &out, sizeof(out), &got); !s) {
    return s;
  }
  if (got != sizeof(out)) {
    return Status::error("unexpected HELLO reply size " + std::to_string(got));
  }
  return out;
}

Result<scorbot_open_reply_t> ControlClient::open_session(const scorbot_open_req_t& req) {
  scorbot_open_reply_t out{};
  size_t got = 0;
  if (Status s = transact(SCORBOT_OP_OPEN_SESSION, &req, sizeof(req), &out, sizeof(out), &got);
      !s) {
    return s;
  }
  if (got != sizeof(out)) {
    return Status::error("unexpected OPEN_SESSION reply size " + std::to_string(got));
  }
  session_id_ = out.session_id;
  return out;
}

Status ControlClient::close_session() {
  Status s = transact(SCORBOT_OP_CLOSE_SESSION, nullptr, 0, nullptr, 0, nullptr);
  session_id_ = 0;
  return s;
}

Status ControlClient::set_mode(const uint8_t modes[SCORBOT_MAX_JOINTS], uint8_t joint_mask) {
  scorbot_set_mode_req_t req{};
  std::memcpy(req.mode, modes, SCORBOT_MAX_JOINTS);
  req.joint_mask = joint_mask;
  return transact(SCORBOT_OP_SET_MODE, &req, sizeof(req), nullptr, 0, nullptr);
}

Status ControlClient::set_gains(const scorbot_gains_t& gains) {
  return transact(SCORBOT_OP_SET_GAINS, &gains, sizeof(gains), nullptr, 0, nullptr);
}

Result<scorbot_gains_t> ControlClient::get_gains(uint8_t joint) {
  scorbot_gains_t req{};
  req.joint = joint;
  scorbot_gains_t out{};
  size_t got = 0;
  if (Status s = transact(SCORBOT_OP_GET_GAINS, &req, sizeof(req), &out, sizeof(out), &got); !s) {
    return s;
  }
  if (got != sizeof(out)) {
    return Status::error("unexpected GET_GAINS reply size " + std::to_string(got));
  }
  return out;
}

Status ControlClient::set_homing_cfg(const scorbot_homing_cfg_t& cfg) {
  return transact(SCORBOT_OP_SET_HOMING_CFG, &cfg, sizeof(cfg), nullptr, 0, nullptr);
}

Result<scorbot_homing_cfg_t> ControlClient::get_homing_cfg(uint8_t joint) {
  scorbot_homing_cfg_t req{};
  req.joint = joint;
  scorbot_homing_cfg_t out{};
  size_t got = 0;
  if (Status s = transact(SCORBOT_OP_GET_HOMING_CFG, &req, sizeof(req), &out, sizeof(out), &got);
      !s) {
    return s;
  }
  if (got != sizeof(out)) {
    return Status::error("unexpected GET_HOMING_CFG reply size " + std::to_string(got));
  }
  return out;
}

Status ControlClient::set_limits(const scorbot_limits_t& limits) {
  return transact(SCORBOT_OP_SET_LIMITS, &limits, sizeof(limits), nullptr, 0, nullptr);
}

Status ControlClient::zero(uint8_t joint, float angle) {
  scorbot_zero_req_t req{};
  req.joint = joint;
  req.angle = angle;
  return transact(SCORBOT_OP_ZERO, &req, sizeof(req), nullptr, 0, nullptr);
}

Status ControlClient::home(uint8_t joint_mask, uint32_t confirm) {
  scorbot_home_req_t req{};
  req.joint_mask = joint_mask;
  req.confirm = confirm;
  return transact(SCORBOT_OP_HOME, &req, sizeof(req), nullptr, 0, nullptr);
}

Result<scorbot_home_status_t> ControlClient::home_status() {
  scorbot_home_status_t out{};
  size_t got = 0;
  if (Status s = transact(SCORBOT_OP_HOME_STATUS, nullptr, 0, &out, sizeof(out), &got); !s) {
    return s;
  }
  if (got != sizeof(out)) {
    return Status::error("unexpected HOME_STATUS reply size " + std::to_string(got));
  }
  return out;
}

Status ControlClient::stream_start(uint32_t state_period_us, uint32_t watchdog_ms) {
  scorbot_stream_req_t req{};
  req.state_period_us = state_period_us;
  req.watchdog_ms = watchdog_ms;
  return transact(SCORBOT_OP_STREAM_START, &req, sizeof(req), nullptr, 0, nullptr);
}

Status ControlClient::stream_stop() {
  return transact(SCORBOT_OP_STREAM_STOP, nullptr, 0, nullptr, 0, nullptr);
}

Status ControlClient::abort() {
  return transact(SCORBOT_OP_ABORT, nullptr, 0, nullptr, 0, nullptr);
}

}  // namespace scorbot
