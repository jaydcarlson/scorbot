# Scorbot host stack

Low-latency UDP control for the Scorbot ER-4u motor board, with a stand-alone
comms layer, Holoscan 5 operators on top of it, and a browser control panel.

## Layout

```
include/scorbot/   comms layer headers (no Holoscan dependency)
src/               comms layer implementation
operators/         ScorbotTxOp and ScorbotRxOp for Holoscan 5
apps/              scorbot_stream, an example Holoscan graph
tools/             scorbot_cli, scorbot_ping, scorbot_sim, scorbot_ibvprobe
webui/             scorbot_webd and its static page
```

The wire protocol lives in [`../firmware/Inc/scorbot_proto.h`](../firmware/Inc/scorbot_proto.h)
and is compiled by both sides. There is no second copy to drift.

## Protocol

Two planes, because they want opposite things.

**Control plane** (UDP 6002) is strict call/response with transaction-id
matching and host-side retries. It carries discovery, session setup, motor
modes, PID gains, homing and abort. Every operation is idempotent, so retries
are safe.

**Data plane** (UDP 6001) is two free-running streams, not request/response.
The host emits `scorbot_pose_t` (64 bytes) at whatever rate it likes and the
firmware emits `scorbot_state_t` (184 bytes) on its own cadence. Nothing is
acked: a lost setpoint is superseded a millisecond later, so resending stale
positions would be worse than dropping them. The streams are correlated through
`echo_seq` and `t_echo_ns`, which give round-trip latency and per-setpoint
confirmation without lockstep coupling.

Setting `SCORBOT_POSE_FLAG_REPLY_NOW` makes the firmware answer that pose
immediately instead of on its next tick. Without it a latency measurement also
contains up to a full tick of phase, which at 1 kHz is hundreds of microseconds
of noise on top of a number whose interesting range is tens.

A pose frame is 106 bytes on the wire, which fits inside the ConnectX 128-byte
inline send limit.

## Transports

Selected at runtime with `--transport`:

- `socket` — ordinary connected UDP socket with `SO_BUSY_POLL`. Portable, no
  privileges.
- `ibverbs` — ConnectX raw packet queue pair (`IBV_QPT_RAW_PACKET`). Not RoCE:
  the RDMA stack is used purely as a userspace path for injecting and
  extracting raw Ethernet frames, with an `ibv_create_flow` rule steering just
  our UDP port into the queue pair. Needs `CAP_NET_RAW`.

Both put real UDP on the wire, so one firmware endpoint serves either.

## Building

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The Holoscan 5 SDK is found automatically in a sibling `holoscan-main-5x`
checkout; override with `-DHOLOSCAN_INSTALL_ROOT=...`. Without it the comms
layer, tools and web UI still build, and only the operators are skipped. The
ibverbs backend is compiled whenever `rdma-core` headers are present.

## Trying it without a robot

`scorbot_sim` speaks the robot side of both planes, including a homing state
machine, so everything can be exercised before a motor turns:

```sh
./build/scorbot_sim --verbose &
./build/scorbot_cli   --ip 127.0.0.1 --iface lo info
./build/scorbot_cli   --ip 127.0.0.1 --iface lo home
./build/scorbot_ping  --ip 127.0.0.1 --iface lo --count 2000
./build/apps/scorbot_stream --ip 127.0.0.1 --iface lo --seconds 5
./build/webui/scorbot_webd  --ip 127.0.0.1 --iface lo --port 8080
```

## Against the real robot

```sh
./build/scorbot_cli --ip 192.168.0.161 --iface enp2s0f1np1 info
./build/webui/scorbot_webd --ip 192.168.0.161 --iface enp2s0f1np1
```

For the ibverbs path add `--transport ibverbs --ibv-dev mlx5_1`, and grant the
capability once:

```sh
sudo setcap cap_net_raw,cap_sys_nice+ep build/scorbot_ping
```

`scorbot_ibvprobe` is the diagnostic for that path. `--selftest` validates frame
construction offline; without it, it fires hand-built frames at an arbitrary
MAC and port. Note that mlx5 raw-QP transmit bypasses the kernel, so local
`tcpdump` cannot see it — check `ethtool -S <iface> | grep tx_packets_phy`
instead, or watch the peer.

## Session arbitration

The firmware honours one streaming session at a time, so the web UI and a
Holoscan app cannot both drive the data plane. `OPEN_SESSION` returns `BUSY`
naming the current owner unless `--force` is passed. `scorbot_webd --observe`
uses only the control plane and never claims the data plane.

## Safety

- Homing refuses to start without an explicit confirmation token.
- Every homing state has a timeout and a stall detector; either drops to
  `FAULT` with all motors at zero duty.
- The firmware watchdog drops to a hold state if the pose stream goes quiet,
  and is suspended while homing runs so a firmware-driven sequence is not
  aborted by a host that has nothing to send.
- The web UI keeps a joint's slider locked until the firmware reports it homed,
  and tracks measured position into the setpoint whenever a joint is not being
  commanded, so nothing lurches when it is switched into position mode.
