// scorbot_webd - browser control panel for the arm.
//
// Owns a ScorbotLink and serves a single page over HTTP plus a WebSocket that
// carries JSON both ways. The browser never touches the hot path: a worker
// thread streams poses at the configured rate and telemetry is decimated to
// roughly 30 Hz before it goes out to any connected clients.
//
// This is a lab tool on a private network. There is no authentication, and the
// JSON scanner below understands only the fixed command vocabulary the bundled
// page sends rather than being a general parser.

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "civetweb.h"
#include "scorbot/scorbot_link.hpp"

#ifndef SCORBOT_DEFAULT_STATIC_DIR
#define SCORBOT_DEFAULT_STATIC_DIR "./static"
#endif

namespace {

std::atomic<bool> g_running{true};

// Without this, a plain kill leaves the firmware still holding the last
// commanded pose: the session is never closed, so the motors stay energised and
// the next client finds the robot mid-command instead of idle.
extern "C" void on_terminate_signal(int /*sig*/) { g_running.store(false); }

// --------------------------------------------------------------------------
// Minimal JSON helpers. Emitting is exact; parsing only has to cope with the
// small, fixed set of messages the bundled page produces.
// --------------------------------------------------------------------------

std::string json_escape(const std::string& in) {
  std::string out;
  out.reserve(in.size() + 8);
  for (char c : in) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

// Emits a finite number, or null where JSON cannot represent the value. The
// firmware legitimately reports infinite travel limits on the wrist joints.
std::string json_number(double v) {
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

bool find_key(const std::string& msg, const std::string& key, size_t* value_pos) {
  const std::string needle = "\"" + key + "\"";
  size_t pos = msg.find(needle);
  if (pos == std::string::npos) {
    return false;
  }
  pos = msg.find(':', pos + needle.size());
  if (pos == std::string::npos) {
    return false;
  }
  ++pos;
  while (pos < msg.size() && (msg[pos] == ' ' || msg[pos] == '\t')) {
    ++pos;
  }
  *value_pos = pos;
  return pos < msg.size();
}

bool json_get_string(const std::string& msg, const std::string& key, std::string* out) {
  size_t pos = 0;
  if (!find_key(msg, key, &pos) || msg[pos] != '"') {
    return false;
  }
  ++pos;
  std::string value;
  while (pos < msg.size() && msg[pos] != '"') {
    if (msg[pos] == '\\' && pos + 1 < msg.size()) {
      ++pos;
    }
    value += msg[pos++];
  }
  *out = value;
  return true;
}

bool json_get_double(const std::string& msg, const std::string& key, double* out) {
  size_t pos = 0;
  if (!find_key(msg, key, &pos)) {
    return false;
  }
  try {
    size_t consumed = 0;
    const double v = std::stod(msg.substr(pos), &consumed);
    if (consumed == 0) {
      return false;
    }
    *out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool json_get_int(const std::string& msg, const std::string& key, long* out) {
  double v = 0.0;
  if (!json_get_double(msg, key, &v)) {
    return false;
  }
  *out = static_cast<long>(v);
  return true;
}

// --------------------------------------------------------------------------
// Server
// --------------------------------------------------------------------------

struct PendingCommand {
  std::string verb;
  long joint = -1;
  double value = 0.0;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  long mask = 0xFF;
};

class WebServer {
 public:
  WebServer(scorbot::ScorbotLink& link, bool observe, double stream_hz)
      : link_(link), observe_(observe), stream_period_ns_(static_cast<uint64_t>(1e9 / stream_hz)) {
    const auto& info = link_.info();
    for (uint8_t j = 0; j < info.joint_count && j < SCORBOT_MAX_JOINTS; ++j) {
      // Start every joint holding, so simply loading the page never moves the
      // arm. The user has to actively take a joint out of hold.
      desired_mode_[j] = SCORBOT_MODE_HOLD;
      desired_[j] = 0.0F;
    }
  }

  void add_client(mg_connection* conn) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.push_back(conn);
  }

  void remove_client(mg_connection* conn) {
    const std::lock_guard<std::mutex> lock(clients_mutex_);
    for (size_t i = 0; i < clients_.size(); ++i) {
      if (clients_[i] == conn) {
        clients_.erase(clients_.begin() + static_cast<long>(i));
        return;
      }
    }
  }

  void queue(const PendingCommand& cmd) {
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(cmd);
  }

  // Sent once when a browser connects: everything static about the robot.
  [[nodiscard]] std::string describe() const {
    const auto& info = link_.info();
    std::string out = "{\"type\":\"info\",\"joints\":[";
    for (uint8_t j = 0; j < info.joint_count && j < SCORBOT_MAX_JOINTS; ++j) {
      if (j > 0) {
        out += ",";
      }
      out += "{\"index\":" + std::to_string(j);
      out += ",\"name\":\"" + json_escape(info.joints[j].name) + "\"";
      out += ",\"min\":" + json_number(info.joints[j].min_angle);
      out += ",\"max\":" + json_number(info.joints[j].max_angle);
      out += "}";
    }
    out += "],\"transport\":\"" + std::string(link_.transport_name()) + "\"";
    out += ",\"session\":" + std::to_string(link_.session_id());
    out += ",\"observe\":" + std::string(observe_ ? "true" : "false");
    out += ",\"fw_version\":" + std::to_string(info.fw_version);
    out += ",\"robot_mac\":\"" + info.robot_mac.to_string() + "\"";
    out += "}";
    return out;
  }

  void run_worker();
  void run_broadcast();
  void stop() { g_running.store(false); }

 private:
  void apply_commands();
  [[nodiscard]] std::string telemetry_json();

  scorbot::ScorbotLink& link_;
  bool observe_;
  uint64_t stream_period_ns_;

  // link_ is not thread safe, so only the worker thread touches it. Web handler
  // threads communicate through the command queue and the snapshot.
  std::mutex queue_mutex_;
  std::vector<PendingCommand> queue_;

  std::mutex snapshot_mutex_;
  scorbot_state_t snapshot_{};
  bool have_snapshot_ = false;
  std::string last_error_;
  std::string last_notice_;

  std::mutex clients_mutex_;
  std::vector<mg_connection*> clients_;

  float desired_[SCORBOT_MAX_JOINTS] = {};
  uint8_t desired_mode_[SCORBOT_MAX_JOINTS] = {};
  bool estopped_ = false;
};

void WebServer::apply_commands() {
  std::vector<PendingCommand> batch;
  {
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    batch.swap(queue_);
  }

  for (const auto& cmd : batch) {
    scorbot::Status result = scorbot::Status::ok();
    std::string notice;

    if (cmd.verb == "estop") {
      estopped_ = true;
      for (auto& m : desired_mode_) {
        m = SCORBOT_MODE_IDLE;
      }
      result = link_.abort();
      notice = "E-STOP: every joint idle";
    } else if (cmd.verb == "clear_estop") {
      estopped_ = false;
      for (auto& m : desired_mode_) {
        m = SCORBOT_MODE_HOLD;
      }
      result = link_.set_mode_all(SCORBOT_MODE_HOLD);
      notice = "E-stop cleared; joints holding";
    } else if (cmd.verb == "set_joint") {
      if (cmd.joint >= 0 && cmd.joint < SCORBOT_MAX_JOINTS && !estopped_) {
        desired_[cmd.joint] = static_cast<float>(cmd.value);
        desired_mode_[cmd.joint] = SCORBOT_MODE_POSITION;
      }
    } else if (cmd.verb == "set_mode") {
      if (cmd.joint >= 0 && cmd.joint < SCORBOT_MAX_JOINTS) {
        desired_mode_[cmd.joint] = static_cast<uint8_t>(cmd.value);
        if (static_cast<uint8_t>(cmd.value) == SCORBOT_MODE_HOLD && have_snapshot_) {
          desired_[cmd.joint] = snapshot_.position[cmd.joint];
        }
        result = link_.set_mode(static_cast<uint8_t>(cmd.joint),
                                static_cast<uint8_t>(cmd.value));
      }
    } else if (cmd.verb == "set_mode_all") {
      const auto mode = static_cast<uint8_t>(cmd.value);
      for (auto& m : desired_mode_) {
        m = mode;
      }
      result = link_.set_mode_all(mode);
    } else if (cmd.verb == "home") {
      if (estopped_) {
        result = scorbot::Status::error("clear the e-stop before homing");
      } else {
        result = link_.start_homing(static_cast<uint8_t>(cmd.mask));
        notice = "homing started";
      }
    } else if (cmd.verb == "set_gains") {
      if (cmd.joint >= 0 && cmd.joint < SCORBOT_MAX_JOINTS) {
        auto current = link_.get_gains(static_cast<uint8_t>(cmd.joint));
        if (!current) {
          result = current.status();
        } else {
          scorbot_gains_t g = *current;
          g.joint = static_cast<uint8_t>(cmd.joint);
          g.kp = static_cast<float>(cmd.kp);
          g.ki = static_cast<float>(cmd.ki);
          g.kd = static_cast<float>(cmd.kd);
          result = link_.set_gains(g);
          notice = "gains updated";
        }
      }
    } else if (cmd.verb == "get_gains") {
      auto g = link_.get_gains(static_cast<uint8_t>(cmd.joint));
      if (!g) {
        result = g.status();
      } else {
        const std::lock_guard<std::mutex> lock(snapshot_mutex_);
        last_notice_ = "{\"type\":\"gains\",\"joint\":" + std::to_string(cmd.joint) +
                       ",\"kp\":" + json_number(g->kp) + ",\"ki\":" + json_number(g->ki) +
                       ",\"kd\":" + json_number(g->kd) + "}";
        continue;
      }
    }

    const std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!result) {
      last_error_ = result.message();
    } else if (!notice.empty()) {
      last_notice_ = notice;
    }
  }
}

void WebServer::run_worker() {
  uint64_t next_ns = scorbot::monotonic_ns();

  while (g_running.load()) {
    apply_commands();

    if (!observe_) {
      scorbot_pose_t pose{};
      for (size_t j = 0; j < SCORBOT_MAX_JOINTS; ++j) {
        pose.mode[j] = desired_mode_[j];
        pose.setpoint[j] = desired_[j];
      }
      if (estopped_) {
        pose.flags |= SCORBOT_POSE_FLAG_ESTOP;
      }
      (void)link_.send_pose(pose);
      link_.poll_send();
    }

    scorbot_state_t state{};
    if (link_.poll_state(state)) {
      // Bumpless transfer: any joint we are not actively commanding has its
      // setpoint track the measured position. Without this a joint that moved
      // while idle, held, or homing would lurch back to a stale setpoint the
      // instant it was switched into position mode.
      for (size_t j = 0; j < SCORBOT_MAX_JOINTS; ++j) {
        if (desired_mode_[j] != SCORBOT_MODE_POSITION) {
          desired_[j] = state.position[j];
        }
      }
      const std::lock_guard<std::mutex> lock(snapshot_mutex_);
      snapshot_ = state;
      have_snapshot_ = true;
    }

    next_ns += stream_period_ns_;
    const uint64_t now = scorbot::monotonic_ns();
    if (next_ns > now) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(next_ns - now));
    } else {
      next_ns = now;
    }
  }
}

std::string WebServer::telemetry_json() {
  scorbot_state_t state{};
  bool have = false;
  std::string error;
  std::string notice;
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);
    state = snapshot_;
    have = have_snapshot_;
    error.swap(last_error_);
    notice.swap(last_notice_);
  }

  // A notice that is already a JSON object (the gains reply) is forwarded as-is.
  if (!notice.empty() && notice.front() == '{') {
    return notice;
  }

  const auto& info = link_.info();
  const auto& stats = link_.stats();

  std::string out = "{\"type\":\"state\",\"have\":";
  out += have ? "true" : "false";
  out += ",\"homing_state\":" + std::to_string(state.homing_state);
  out += ",\"homing_joint\":" + std::to_string(state.homing_joint);
  out += ",\"fault\":" + std::to_string(state.fault);
  out += ",\"flags\":" + std::to_string(state.flags);
  out += ",\"limit_mask\":" + std::to_string(state.limit_mask);
  out += ",\"estop\":" + std::string(estopped_ ? "true" : "false");
  out += ",\"rtt_us\":" + json_number(stats.last_rtt_us);
  out += ",\"sent\":" + std::to_string(stats.poses_sent);
  out += ",\"recv\":" + std::to_string(stats.states_received);
  out += ",\"dropped\":" + std::to_string(stats.states_dropped);
  out += ",\"bad\":" + std::to_string(stats.bad_packets);
  out += ",\"joints\":[";
  for (uint8_t j = 0; j < info.joint_count && j < SCORBOT_MAX_JOINTS; ++j) {
    if (j > 0) {
      out += ",";
    }
    out += "{\"position\":" + json_number(state.position[j]);
    out += ",\"velocity\":" + json_number(state.velocity[j]);
    out += ",\"effort\":" + json_number(state.effort[j]);
    out += ",\"mode\":" + std::to_string(state.mode[j]);
    out += ",\"flags\":" + std::to_string(state.jflags[j]);
    out += ",\"target\":" + json_number(desired_[j]);
    out += "}";
  }
  out += "]";
  if (!error.empty()) {
    out += ",\"error\":\"" + json_escape(error) + "\"";
  }
  if (!notice.empty()) {
    out += ",\"notice\":\"" + json_escape(notice) + "\"";
  }
  out += "}";
  return out;
}

void WebServer::run_broadcast() {
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 Hz
    const std::string payload = telemetry_json();

    const std::lock_guard<std::mutex> lock(clients_mutex_);
    for (mg_connection* c : clients_) {
      mg_websocket_write(c, MG_WEBSOCKET_OPCODE_TEXT, payload.c_str(), payload.size());
    }
  }
}

WebServer* g_server = nullptr;

int ws_connect_handler(const mg_connection* /*conn*/, void* /*data*/) {
  return 0;  // accept
}

void ws_ready_handler(mg_connection* conn, void* /*data*/) {
  g_server->add_client(conn);
  const std::string info = g_server->describe();
  mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, info.c_str(), info.size());
}

int ws_data_handler(mg_connection* /*conn*/, int bits, char* data, size_t len, void* /*cbdata*/) {
  if ((bits & 0x0F) == MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE) {
    return 0;
  }
  const std::string msg(data, len);

  PendingCommand cmd;
  if (!json_get_string(msg, "cmd", &cmd.verb)) {
    return 1;
  }
  long joint = -1;
  if (json_get_int(msg, "joint", &joint)) {
    cmd.joint = joint;
  }
  (void)json_get_double(msg, "value", &cmd.value);
  (void)json_get_double(msg, "kp", &cmd.kp);
  (void)json_get_double(msg, "ki", &cmd.ki);
  (void)json_get_double(msg, "kd", &cmd.kd);
  long mask = 0xFF;
  if (json_get_int(msg, "mask", &mask)) {
    cmd.mask = mask;
  }

  g_server->queue(cmd);
  return 1;  // keep the connection open
}

void ws_close_handler(const mg_connection* conn, void* /*data*/) {
  g_server->remove_client(const_cast<mg_connection*>(conn));
}

void print_usage() {
  std::printf(
      "usage: scorbot_webd [options]\n"
      "  --ip ADDR          robot address (default 192.168.0.161)\n"
      "  --iface NAME       local interface (default enp2s0f1np1)\n"
      "  --transport KIND   socket or ibverbs (default socket)\n"
      "  --ibv-dev NAME     RDMA device (default mlx5_1)\n"
      "  --local-port N     local data plane port (default 6011)\n"
      "  --port N           HTTP port to serve on (default 8080)\n"
      "  --rate HZ          setpoint stream rate (default 200)\n"
      "  --static-dir DIR   directory holding index.html\n"
      "  --observe          control plane only; never claim the data plane\n"
      "  --force            take the session from another host\n");
}

}  // namespace

int main(int argc, char** argv) {
  scorbot::ScorbotLink::Config cfg;
  std::string http_port = "8080";
  std::string static_dir = SCORBOT_DEFAULT_STATIC_DIR;
  double stream_hz = 200.0;
  bool observe = false;

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
    } else if (arg == "--transport" && i + 1 < argc) {
      auto kind = scorbot::parse_transport_kind(argv[++i]);
      if (!kind) {
        std::fprintf(stderr, "%s\n", kind.status().c_str());
        return 1;
      }
      cfg.transport = *kind;
    } else if (arg == "--port" && i + 1 < argc) {
      http_port = argv[++i];
    } else if (arg == "--rate" && i + 1 < argc) {
      stream_hz = std::stod(argv[++i]);
    } else if (arg == "--static-dir" && i + 1 < argc) {
      static_dir = argv[++i];
    } else if (arg == "--observe") {
      observe = true;
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

  scorbot::ScorbotLink link;
  scorbot::Status s = observe ? link.open_control(cfg) : link.open(cfg);
  if (!s) {
    std::fprintf(stderr, "could not reach the robot at %s: %s\n", cfg.robot_ip.c_str(), s.c_str());
    return 1;
  }

  WebServer server(link, observe, stream_hz);
  g_server = &server;

  mg_init_library(0);

  const char* options[] = {"document_root",
                           static_dir.c_str(),
                           "listening_ports",
                           http_port.c_str(),
                           "num_threads",
                           "8",
                           "enable_directory_listing",
                           "no",
                           nullptr};

  mg_callbacks callbacks{};
  mg_context* ctx = mg_start(&callbacks, nullptr, options);
  if (ctx == nullptr) {
    std::fprintf(stderr, "could not start the web server on port %s\n", http_port.c_str());
    return 1;
  }

  mg_set_websocket_handler(ctx, "/ws", ws_connect_handler, ws_ready_handler, ws_data_handler,
                           ws_close_handler, nullptr);

  std::signal(SIGINT, on_terminate_signal);
  std::signal(SIGTERM, on_terminate_signal);

  std::thread worker([&server] { server.run_worker(); });
  std::thread broadcaster([&server] { server.run_broadcast(); });

  std::printf("scorbot_webd: serving http://0.0.0.0:%s  (robot %s over %s%s)\n",
              http_port.c_str(), cfg.robot_ip.c_str(), link.transport_name(),
              observe ? ", observe mode" : "");
  std::printf("              static files from %s\n", static_dir.c_str());
  std::printf("              press ctrl-c to stop\n");

  worker.join();
  broadcaster.join();
  mg_stop(ctx);
  mg_exit_library();

  // Leave the arm de-energised rather than frozen holding its last pose.
  if (!observe) {
    (void)link.set_mode_all(SCORBOT_MODE_IDLE);
  }
  link.close();
  std::printf("\nscorbot_webd: stopped, robot released\n");
  return 0;
}
