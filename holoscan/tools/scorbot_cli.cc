// scorbot_cli - command line access to the robot, with no Holoscan involved.
//
// Doubles as the proof that ScorbotLink really is usable stand-alone.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "scorbot/scorbot_link.hpp"

namespace {

const char* mode_name(uint8_t mode) {
  switch (mode) {
    case SCORBOT_MODE_IDLE:
      return "idle";
    case SCORBOT_MODE_HOLD:
      return "hold";
    case SCORBOT_MODE_POSITION:
      return "position";
    case SCORBOT_MODE_VELOCITY:
      return "velocity";
    case SCORBOT_MODE_PWM:
      return "pwm";
    default:
      return "?";
  }
}

uint8_t parse_mode(const std::string& text) {
  if (text == "idle") return SCORBOT_MODE_IDLE;
  if (text == "hold") return SCORBOT_MODE_HOLD;
  if (text == "position") return SCORBOT_MODE_POSITION;
  if (text == "velocity") return SCORBOT_MODE_VELOCITY;
  if (text == "pwm") return SCORBOT_MODE_PWM;
  return 0xFF;
}

const char* homing_name(uint8_t state) {
  switch (state) {
    case SCORBOT_HOMING_IDLE:
      return "idle";
    case SCORBOT_HOMING_PARK:
      return "park";
    case SCORBOT_HOMING_SEEK:
      return "seek";
    case SCORBOT_HOMING_BACKOFF:
      return "backoff";
    case SCORBOT_HOMING_CREEP:
      return "creep";
    case SCORBOT_HOMING_LATCH:
      return "latch";
    case SCORBOT_HOMING_HOLD:
      return "hold";
    case SCORBOT_HOMING_DONE:
      return "done";
    case SCORBOT_HOMING_FAULT:
      return "FAULT";
    default:
      return "?";
  }
}

void print_usage() {
  std::printf(
      "usage: scorbot_cli [options] <command> [args]\n"
      "\n"
      "options:\n"
      "  --ip ADDR          robot address (default 192.168.0.161)\n"
      "  --iface NAME       local interface (default enp2s0f1np1)\n"
      "  --transport KIND   socket or ibverbs (default socket)\n"
      "  --ibv-dev NAME     RDMA device for the ibverbs transport (default mlx5_1)\n"
      "  --local-port N     local data plane port (default 6011)\n"
      "  --force            take the session even if another host holds it\n"
      "\n"
      "commands:\n"
      "  info                       show firmware and joint information\n"
      "  status                     stream telemetry until interrupted\n"
      "  home [mask]                run the homing sequence (default all joints)\n"
      "  home-joint <index>         home a single joint, for attended bring-up\n"
      "  mode <joint|all> <mode>    idle | hold | position | velocity | pwm\n"
      "  move <joint> <degrees>     send one position setpoint\n"
      "  gains <joint> [kp ki kd]   read or write PID gains\n"
      "  pwm <joint> <duty> [secs]  open-loop drive, for bring-up diagnosis\n"
      "  abort                      stop everything and drop to idle\n");
}

int cmd_info(scorbot::ScorbotLink& link) {
  const auto& info = link.info();
  std::printf("firmware      0x%08x\n", info.fw_version);
  std::printf("protocol      v%u\n", info.proto_version);
  std::printf("uptime        %.1f s\n", static_cast<double>(info.uptime_ms) / 1000.0);
  std::printf("robot MAC     %s\n", info.robot_mac.to_string().c_str());
  if (info.active_session != 0) {
    std::printf("session       %u held by %s\n", info.active_session,
                scorbot::ipv4_to_string(info.owner_ip).c_str());
  } else {
    std::printf("session       none\n");
  }
  std::printf("\n%-3s %-16s %10s %10s\n", "id", "joint", "min", "max");
  for (uint8_t j = 0; j < info.joint_count; ++j) {
    std::printf("%-3u %-16s %10.2f %10.2f\n", j, info.joints[j].name.c_str(),
                static_cast<double>(info.joints[j].min_angle),
                static_cast<double>(info.joints[j].max_angle));
  }
  return 0;
}

int cmd_status(scorbot::ScorbotLink& link) {
  const auto& info = link.info();
  std::printf("streaming telemetry, ctrl-c to stop\n\n");

  int lines = 0;
  while (true) {
    scorbot_state_t state{};
    // Keep feeding the watchdog so the firmware does not fault us out while we
    // sit here just watching.
    (void)link.send_keepalive();
    link.poll_send();

    if (link.poll_state(state)) {
      if (lines % 20 == 0) {
        std::printf("%-16s %9s %9s %9s %-9s %s\n", "joint", "pos", "vel", "effort", "mode",
                    "flags");
      }
      for (uint8_t j = 0; j < info.joint_count; ++j) {
        char flags[8] = "----";
        if ((state.jflags[j] & SCORBOT_JFLAG_HOMED) != 0U) flags[0] = 'H';
        if ((state.jflags[j] & SCORBOT_JFLAG_AT_LIMIT) != 0U) flags[1] = 'L';
        if ((state.jflags[j] & SCORBOT_JFLAG_SATURATED) != 0U) flags[2] = 'S';
        if ((state.jflags[j] & SCORBOT_JFLAG_FAULT) != 0U) flags[3] = 'F';
        std::printf("%-16s %9.2f %9.2f %9.3f %-9s %s\n", info.joints[j].name.c_str(),
                    static_cast<double>(state.position[j]),
                    static_cast<double>(state.velocity[j]), static_cast<double>(state.effort[j]),
                    mode_name(state.mode[j]), flags);
      }
      std::printf("homing=%s  rtt=%.0f us  rx=%llu  drop=%llu  bad=%llu\n\n",
                  homing_name(state.homing_state), link.stats().last_rtt_us,
                  static_cast<unsigned long long>(link.stats().states_received),
                  static_cast<unsigned long long>(link.stats().states_dropped),
                  static_cast<unsigned long long>(link.stats().bad_packets));
      ++lines;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}

int cmd_home(scorbot::ScorbotLink& link, uint8_t mask) {
  std::printf("starting homing sequence (mask 0x%02x)\n", mask);
  std::printf("KEEP A HAND ON THE E-STOP\n\n");

  if (scorbot::Status s = link.start_homing(mask); !s) {
    std::fprintf(stderr, "could not start homing: %s\n", s.c_str());
    return 1;
  }

  uint8_t last_state = 0xFF;
  uint8_t last_joint = 0xFF;
  while (true) {
    auto st = link.homing_status();
    if (!st) {
      std::fprintf(stderr, "lost contact during homing: %s\n", st.status().c_str());
      return 1;
    }
    // The firmware suspends its watchdog while homing, but keep feeding it
    // anyway so the transition back to streaming is seamless.
    (void)link.send_keepalive();
    link.poll_send();

    // Watch the moving joint directly. If homing faults it matters a great deal
    // whether the motor was being driven and simply not moving, or was never
    // driven at all, and effort against position answers that immediately.
    scorbot_state_t state{};
    if (link.poll_state(state) && st->joint < SCORBOT_MAX_JOINTS) {
      const uint8_t j = st->joint;
      std::printf("\r  %-14s pos %8.3f  vel %7.2f  effort %+5.2f  limit=%c   ",
                  link.info().joints[j].name.c_str(),
                  static_cast<double>(state.position[j]),
                  static_cast<double>(state.velocity[j]),
                  static_cast<double>(state.effort[j]),
                  (state.jflags[j] & SCORBOT_JFLAG_AT_LIMIT) != 0U ? 'Y' : 'n');
      std::fflush(stdout);
    }

    if (st->state != last_state || st->joint != last_joint) {
      const char* jname = "-";
      if (st->joint < link.info().joint_count) {
        jname = link.info().joints[st->joint].name.c_str();
      }
      std::printf("\n[%6u ms] %-8s joint=%-14s homed=0x%02x\n", st->elapsed_ms,
                  homing_name(st->state), jname, st->homed_mask);
      last_state = st->state;
      last_joint = st->joint;
    }
    if (st->state == SCORBOT_HOMING_DONE) {
      // Keep the session alive briefly. Closing it drops every joint to idle,
      // which would cut short the move off the limit switch that homing ends
      // with and leave the joint parked against it.
      for (int i = 0; i < 150; ++i) {
        (void)link.send_keepalive();
        link.poll_send();
        scorbot_state_t settle{};
        (void)link.poll_state(settle);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      std::printf("\nhoming complete, homed mask 0x%02x\n", st->homed_mask);
      return 0;
    }
    if (st->state == SCORBOT_HOMING_FAULT) {
      std::fprintf(stderr, "\nhoming FAULTED (error %u)\n", st->error);
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace

int main(int argc, char** argv) {
  scorbot::ScorbotLink::Config cfg;
  std::vector<std::string> args;

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
    } else if (arg == "--force") {
      cfg.force_session = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    } else {
      args.push_back(arg);
    }
  }

  if (args.empty()) {
    print_usage();
    return 1;
  }

  const std::string& command = args[0];
  scorbot::ScorbotLink link;

  // info needs nothing but the control plane, so do not disturb whoever may
  // already own the streaming session.
  const bool control_only = (command == "info");

  scorbot::Status s = control_only ? link.open_control(cfg) : link.open(cfg);
  if (!s) {
    std::fprintf(stderr, "could not reach the robot at %s: %s\n", cfg.robot_ip.c_str(), s.c_str());
    return 1;
  }
  if (!control_only) {
    std::printf("connected to %s over %s (session %u)\n", cfg.robot_ip.c_str(),
                link.transport_name(), link.session_id());
  }

  if (command == "info") {
    return cmd_info(link);
  }
  if (command == "status") {
    return cmd_status(link);
  }
  if (command == "abort") {
    if (scorbot::Status a = link.abort(); !a) {
      std::fprintf(stderr, "abort failed: %s\n", a.c_str());
      return 1;
    }
    std::printf("aborted; all joints idle\n");
    return 0;
  }
  if (command == "home") {
    uint8_t mask = 0xFF;
    if (args.size() > 1) {
      mask = static_cast<uint8_t>(std::stoul(args[1], nullptr, 0));
    }
    return cmd_home(link, mask);
  }
  if (command == "home-joint") {
    if (args.size() < 2) {
      std::fprintf(stderr, "home-joint needs a joint index\n");
      return 1;
    }
    const auto joint = static_cast<uint8_t>(std::stoi(args[1]));
    if (joint >= SCORBOT_MAX_JOINTS) {
      std::fprintf(stderr, "joint index out of range\n");
      return 1;
    }
    return cmd_home(link, static_cast<uint8_t>(1U << joint));
  }
  if (command == "mode") {
    if (args.size() < 3) {
      std::fprintf(stderr, "mode needs a joint (or 'all') and a mode name\n");
      return 1;
    }
    const uint8_t mode = parse_mode(args[2]);
    if (mode == 0xFF) {
      std::fprintf(stderr, "unknown mode '%s'\n", args[2].c_str());
      return 1;
    }
    scorbot::Status r = (args[1] == "all")
                            ? link.set_mode_all(mode)
                            : link.set_mode(static_cast<uint8_t>(std::stoi(args[1])), mode);
    if (!r) {
      std::fprintf(stderr, "set mode failed: %s\n", r.c_str());
      return 1;
    }
    std::printf("mode set to %s\n", args[2].c_str());
    return 0;
  }
  if (command == "move") {
    if (args.size() < 3) {
      std::fprintf(stderr, "move needs a joint index and an angle\n");
      return 1;
    }
    const auto joint = static_cast<size_t>(std::stoi(args[1]));
    const auto angle = std::stof(args[2]);
    if (joint >= SCORBOT_MAX_JOINTS) {
      std::fprintf(stderr, "joint index out of range\n");
      return 1;
    }

    // Hold everything else where it is, and drive just the requested joint.
    scorbot_pose_t pose{};
    for (auto& m : pose.mode) {
      m = SCORBOT_MODE_HOLD;
    }
    pose.mode[joint] = SCORBOT_MODE_POSITION;
    pose.setpoint[joint] = angle;

    std::printf("driving %s to %.2f deg\n", link.info().joints[joint].name.c_str(),
                static_cast<double>(angle));
    // Keep streaming so the watchdog stays fed while the joint travels.
    for (int i = 0; i < 400; ++i) {
      if (!link.send_pose(pose)) {
        std::fprintf(stderr, "transmit failed\n");
        return 1;
      }
      scorbot_state_t state{};
      link.poll_send();
      if (link.poll_state(state) && std::fabs(state.position[joint] - angle) < 0.2F) {
        std::printf("arrived at %.2f deg\n", static_cast<double>(state.position[joint]));
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::printf("stopped waiting; the joint may still be moving\n");
    return 0;
  }
  if (command == "pwm") {
    if (args.size() < 3) {
      std::fprintf(stderr, "pwm needs a joint index and a duty in -1.0 .. 1.0\n");
      return 1;
    }
    const auto joint = static_cast<size_t>(std::stoi(args[1]));
    const float duty = std::stof(args[2]);
    const double seconds = args.size() > 3 ? std::stod(args[3]) : 1.0;
    if (joint >= SCORBOT_MAX_JOINTS || duty < -1.0F || duty > 1.0F) {
      std::fprintf(stderr, "joint index or duty out of range\n");
      return 1;
    }

    // Everything except the joint under test stays idle rather than holding, so
    // nothing else is energised while probing.
    scorbot_pose_t pose{};
    for (auto& m : pose.mode) {
      m = SCORBOT_MODE_IDLE;
    }
    pose.mode[joint] = SCORBOT_MODE_PWM;
    pose.setpoint[joint] = duty;

    std::printf("driving %s open loop at duty %+.2f for %.1f s\n",
                link.info().joints[joint].name.c_str(), static_cast<double>(duty), seconds);

    float start_pos = 0.0F;
    bool have_start = false;
    const auto ticks = static_cast<int>(seconds * 100.0);
    for (int i = 0; i < ticks; ++i) {
      (void)link.send_pose(pose);
      link.poll_send();
      scorbot_state_t state{};
      if (link.poll_state(state)) {
        if (!have_start) {
          start_pos = state.position[joint];
          have_start = true;
        }
        std::printf("\r  pos %9.3f  (moved %+8.3f)  vel %8.2f  effort %+5.2f  limit=%c   ",
                    static_cast<double>(state.position[joint]),
                    static_cast<double>(state.position[joint] - start_pos),
                    static_cast<double>(state.velocity[joint]),
                    static_cast<double>(state.effort[joint]),
                    (state.jflags[joint] & SCORBOT_JFLAG_AT_LIMIT) != 0U ? 'Y' : 'n');
        std::fflush(stdout);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Release the joint before dropping the session.
    for (auto& m : pose.mode) {
      m = SCORBOT_MODE_IDLE;
    }
    pose.setpoint[joint] = 0.0F;
    for (int i = 0; i < 20; ++i) {
      (void)link.send_pose(pose);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::printf("\nstopped\n");
    return 0;
  }
  if (command == "gains") {
    if (args.size() < 2) {
      std::fprintf(stderr, "gains needs a joint index\n");
      return 1;
    }
    const auto joint = static_cast<uint8_t>(std::stoi(args[1]));
    if (args.size() >= 5) {
      auto current = link.get_gains(joint);
      if (!current) {
        std::fprintf(stderr, "could not read gains: %s\n", current.status().c_str());
        return 1;
      }
      scorbot_gains_t g = *current;
      g.joint = joint;
      g.kp = std::stof(args[2]);
      g.ki = std::stof(args[3]);
      g.kd = std::stof(args[4]);
      if (scorbot::Status r = link.set_gains(g); !r) {
        std::fprintf(stderr, "could not write gains: %s\n", r.c_str());
        return 1;
      }
    }
    auto g = link.get_gains(joint);
    if (!g) {
      std::fprintf(stderr, "could not read gains: %s\n", g.status().c_str());
      return 1;
    }
    std::printf("joint %u: kp=%.4f ki=%.4f kd=%.4f i_clamp=%.3f out_clamp=%.3f deadband=%.3f "
                "max_vel=%.1f\n",
                joint, static_cast<double>(g->kp), static_cast<double>(g->ki),
                static_cast<double>(g->kd), static_cast<double>(g->i_clamp),
                static_cast<double>(g->out_clamp), static_cast<double>(g->deadband_deg),
                static_cast<double>(g->max_vel_dps));
    return 0;
  }

  std::fprintf(stderr, "unknown command '%s'\n", command.c_str());
  print_usage();
  return 1;
}
