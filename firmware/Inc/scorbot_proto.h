/*
 * scorbot_proto.h - wire protocol shared by the STM32 firmware and host tools.
 *
 * This header is the single source of truth for every byte on the wire. It is
 * compiled by both the firmware (gnu17, Cortex-M4) and the host (C++20, x86),
 * so it must stay free of anything specific to either side.
 *
 * Two planes:
 *
 *   Data plane (SCORBOT_DATA_PORT, UDP) - free-running streams in both
 *   directions. The host emits scorbot_pose_t at whatever rate it likes; the
 *   firmware emits scorbot_state_t on its own cadence. Nothing is acked or
 *   retransmitted, because a lost setpoint is superseded by the next one. The
 *   streams are correlated through echo_seq/t_echo_ns rather than coupled.
 *
 *   Control plane (SCORBOT_CTRL_PORT, UDP) - strict call/response with xid
 *   matching and host-side retries. Carries session setup, motor modes, PID
 *   gains, homing and abort. Every operation is idempotent so a retried
 *   request is harmless.
 *
 * Both ends are little-endian (Cortex-M4 and x86-64), so multi-byte fields are
 * little-endian on the wire with no byte swapping anywhere. Every struct is
 * packed with all fields naturally aligned, which keeps the firmware's
 * -mno-unaligned-access builds efficient and lets the host map these types
 * directly onto Holoscan POD channel layouts.
 */

#ifndef SCORBOT_PROTO_H_
#define SCORBOT_PROTO_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#define SCORBOT_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define SCORBOT_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

#define SCORBOT_PACKED __attribute__((packed))

/* ------------------------------------------------------------------ */
/* Versions, ports and sizing                                          */
/* ------------------------------------------------------------------ */

#define SCORBOT_PROTO_VERSION 1u

#define SCORBOT_DATA_PORT 6001u
#define SCORBOT_CTRL_PORT 6002u

/*
 * Fixed 8 joint slots so every struct has a compile-time constant layout.
 * 0-5 are the arm joints, 6 is the linear slide, 7 is spare. The firmware
 * reports how many it actually drives in the HELLO reply.
 */
#define SCORBOT_MAX_JOINTS 8u
#define SCORBOT_NAME_LEN 16u

#define SCORBOT_MAGIC_POSE 0x534f5053u  /* 'SPOS' little-endian */
#define SCORBOT_MAGIC_STATE 0x41545353u /* 'SSTA' little-endian */
#define SCORBOT_MAGIC_CTRL 0x4c544353u  /* 'SCTL' little-endian */

/* Guards HOME against an accidental unattended trigger. */
#define SCORBOT_HOME_CONFIRM 0x484f4d45u /* 'HOME' */

/* ------------------------------------------------------------------ */
/* Enumerations (plain int enums; the wire always uses fixed-width)     */
/* ------------------------------------------------------------------ */

/* Per-joint control mode, transported as uint8_t. */
enum scorbot_joint_mode {
  SCORBOT_MODE_IDLE = 0,     /* motor off, free to backdrive          */
  SCORBOT_MODE_HOLD = 1,     /* hold wherever it currently sits       */
  SCORBOT_MODE_POSITION = 2, /* setpoint is degrees                   */
  SCORBOT_MODE_VELOCITY = 3, /* setpoint is degrees/second            */
  SCORBOT_MODE_PWM = 4       /* setpoint is raw duty, -1.0 .. 1.0     */
};

/* scorbot_pose_t.flags */
#define SCORBOT_POSE_FLAG_ESTOP 0x0001u     /* drop everything to IDLE now   */
#define SCORBOT_POSE_FLAG_KEEPALIVE 0x0002u /* feed watchdog, ignore payload */
#define SCORBOT_POSE_FLAG_REPLY_NOW 0x0004u /* emit state immediately        */

/* scorbot_state_t.flags */
#define SCORBOT_STATE_FLAG_STREAMING 0x0001u /* data plane armed             */
#define SCORBOT_STATE_FLAG_WATCHDOG 0x0002u  /* watchdog tripped, in safe    */
#define SCORBOT_STATE_FLAG_ESTOP 0x0004u     /* latched estop                */
#define SCORBOT_STATE_FLAG_HOMING 0x0008u    /* homing sequence running      */

/* scorbot_state_t.jflags[] */
#define SCORBOT_JFLAG_HOMED 0x01u     /* reference established              */
#define SCORBOT_JFLAG_AT_LIMIT 0x02u  /* limit switch currently asserted    */
#define SCORBOT_JFLAG_SATURATED 0x04u /* PID output clamped                 */
#define SCORBOT_JFLAG_FAULT 0x08u     /* joint faulted, forced to IDLE      */

/* scorbot_state_t.fault */
enum scorbot_fault {
  SCORBOT_FAULT_NONE = 0,
  SCORBOT_FAULT_WATCHDOG = 1,   /* pose stream stopped                     */
  SCORBOT_FAULT_HOMING = 2,     /* homing timed out or stalled             */
  SCORBOT_FAULT_ESTOP = 3,      /* estop asserted by host                  */
  SCORBOT_FAULT_OVER_LIMIT = 4, /* commanded past a soft limit repeatedly  */
  SCORBOT_FAULT_SESSION = 5     /* session lost or stolen mid-stream       */
};

/* Homing sequence state, reported in scorbot_state_t.homing_state. */
enum scorbot_homing_state {
  SCORBOT_HOMING_IDLE = 0,
  SCORBOT_HOMING_PARK = 1,    /* quiesce all motors before moving        */
  SCORBOT_HOMING_SEEK = 2,    /* fast approach toward the limit switch   */
  SCORBOT_HOMING_BACKOFF = 3, /* retreat until the switch releases       */
  SCORBOT_HOMING_CREEP = 4,   /* slow re-approach for repeatability      */
  SCORBOT_HOMING_LATCH = 5,   /* zero the encoder, record coupling       */
  SCORBOT_HOMING_HOLD = 6,    /* everything homed, holding home pose     */
  SCORBOT_HOMING_DONE = 7,
  SCORBOT_HOMING_FAULT = 8
};

/* Control plane opcodes. */
enum scorbot_ctrl_opcode {
  SCORBOT_OP_HELLO = 1,
  SCORBOT_OP_OPEN_SESSION = 2,
  SCORBOT_OP_CLOSE_SESSION = 3,
  SCORBOT_OP_SET_MODE = 4,
  SCORBOT_OP_SET_GAINS = 5,
  SCORBOT_OP_GET_GAINS = 6,
  SCORBOT_OP_SET_HOMING_CFG = 7,
  SCORBOT_OP_GET_HOMING_CFG = 8,
  SCORBOT_OP_HOME = 9,
  SCORBOT_OP_HOME_STATUS = 10,
  SCORBOT_OP_SET_LIMITS = 11,
  SCORBOT_OP_ZERO = 12,
  SCORBOT_OP_STREAM_START = 13,
  SCORBOT_OP_STREAM_STOP = 14,
  SCORBOT_OP_ABORT = 15
};

/* Control plane reply status. */
enum scorbot_status {
  SCORBOT_OK = 0,
  SCORBOT_ERR_BAD_MAGIC = 1,
  SCORBOT_ERR_BAD_VERSION = 2,
  SCORBOT_ERR_BAD_OPCODE = 3,
  SCORBOT_ERR_BAD_LENGTH = 4,
  SCORBOT_ERR_BAD_SESSION = 5,
  SCORBOT_ERR_BUSY = 6,        /* another host owns the session          */
  SCORBOT_ERR_BAD_JOINT = 7,
  SCORBOT_ERR_BAD_PARAM = 8,
  SCORBOT_ERR_NOT_HOMED = 9,
  SCORBOT_ERR_NOT_CONFIRMED = 10, /* HOME without the confirm token      */
  SCORBOT_ERR_FAULTED = 11,
  SCORBOT_ERR_TIMEOUT = 12     /* host-side only; never sent by firmware */
};

/* ------------------------------------------------------------------ */
/* Data plane                                                          */
/* ------------------------------------------------------------------ */

/*
 * Host -> robot setpoints. 64 bytes, so the complete Ethernet+IPv4+UDP frame
 * is 106 bytes and fits inside the ConnectX 128-byte inline send limit.
 */
typedef struct SCORBOT_PACKED scorbot_pose {
  uint32_t magic;      /*  0 SCORBOT_MAGIC_POSE                            */
  uint16_t version;    /*  4 SCORBOT_PROTO_VERSION                         */
  uint16_t flags;      /*  6 SCORBOT_POSE_FLAG_*                           */
  uint32_t session_id; /*  8 from OPEN_SESSION; mismatches are dropped     */
  uint32_t seq;        /* 12 monotonic per session                         */
  uint64_t t_tx_ns;    /* 16 host monotonic clock, echoed back verbatim    */
  uint8_t mode[SCORBOT_MAX_JOINTS];   /* 24 enum scorbot_joint_mode        */
  float setpoint[SCORBOT_MAX_JOINTS]; /* 32 deg | deg/s | duty, per mode   */
} scorbot_pose_t;                     /* 64 */

/*
 * Robot -> host telemetry. Emitted on the firmware's own cadence, or
 * immediately when a pose arrives carrying SCORBOT_POSE_FLAG_REPLY_NOW.
 */
typedef struct SCORBOT_PACKED scorbot_state {
  uint32_t magic;            /*   0 SCORBOT_MAGIC_STATE                    */
  uint16_t version;          /*   4                                        */
  uint16_t flags;            /*   6 SCORBOT_STATE_FLAG_*                   */
  uint32_t session_id;       /*   8                                        */
  uint32_t seq;              /*  12 firmware tick counter                  */
  uint64_t t_echo_ns;        /*  16 t_tx_ns of the last applied pose       */
  uint32_t echo_seq;         /*  24 seq of the last applied pose           */
  uint32_t t_fw_us;          /*  28 firmware uptime at sample, microsecond */
  uint16_t limit_mask;       /*  32 bit N = limit switch N asserted        */
  uint8_t homing_state;      /*  34 enum scorbot_homing_state              */
  uint8_t homing_joint;      /*  35 joint currently homing, 0xFF if none   */
  uint8_t fault;             /*  36 enum scorbot_fault                     */
  uint8_t reserved;          /*  37                                        */
  uint16_t missed_deadlines; /*  38 control loop overruns since boot       */
  uint8_t mode[SCORBOT_MAX_JOINTS];    /*  40 mode actually in effect      */
  uint8_t jflags[SCORBOT_MAX_JOINTS];  /*  48 SCORBOT_JFLAG_*              */
  float position[SCORBOT_MAX_JOINTS];  /*  56 degrees                      */
  float velocity[SCORBOT_MAX_JOINTS];  /*  88 degrees/second               */
  float effort[SCORBOT_MAX_JOINTS];    /* 120 applied duty, -1.0 .. 1.0    */
  int32_t encoder[SCORBOT_MAX_JOINTS]; /* 152 raw accumulated counts       */
} scorbot_state_t;                     /* 184 */

/* ------------------------------------------------------------------ */
/* Control plane                                                       */
/* ------------------------------------------------------------------ */

/* Every control datagram starts with this, followed by payload_len bytes. */
typedef struct SCORBOT_PACKED scorbot_ctrl_hdr {
  uint32_t magic;       /*  0 SCORBOT_MAGIC_CTRL                           */
  uint16_t version;     /*  4                                              */
  uint16_t opcode;      /*  6 enum scorbot_ctrl_opcode                     */
  uint32_t xid;         /*  8 echoed in the reply so retries can match     */
  uint32_t session_id;  /* 12 zero for HELLO and OPEN_SESSION              */
  uint16_t status;      /* 16 reply only; enum scorbot_status              */
  uint16_t payload_len; /* 18 bytes following this header                  */
  uint32_t reserved;    /* 20                                              */
} scorbot_ctrl_hdr_t;   /* 24 */

/* HELLO reply. Needs no session and is safe to send at any time. */
typedef struct SCORBOT_PACKED scorbot_hello_reply {
  uint32_t fw_version;
  uint32_t proto_version;
  uint32_t uptime_ms;
  uint32_t active_session; /* 0 when the data plane is unclaimed           */
  uint32_t owner_ip;       /* host currently owning the session            */
  uint8_t robot_mac[6];    /* lets the host skip ARP on the raw QP path    */
  uint8_t joint_count;
  uint8_t reserved;
  char joint_name[SCORBOT_MAX_JOINTS][SCORBOT_NAME_LEN];
  float min_angle[SCORBOT_MAX_JOINTS];
  float max_angle[SCORBOT_MAX_JOINTS];
} scorbot_hello_reply_t; /* 220 */

/*
 * OPEN_SESSION request. Carries the host's L2/L3 identity so the firmware can
 * address the data plane without ARP, mirroring how the existing Hololink
 * data plane is configured.
 */
typedef struct SCORBOT_PACKED scorbot_open_req {
  uint8_t host_mac[6];
  uint16_t host_port;   /* where the host wants state delivered            */
  uint32_t host_ip;     /* host byte order, little-endian on the wire      */
  uint32_t watchdog_ms; /* silence tolerated before dropping to safe       */
  uint8_t force;        /* steal an existing session instead of ERR_BUSY   */
  uint8_t reserved[3];
} scorbot_open_req_t; /* 20 */

typedef struct SCORBOT_PACKED scorbot_open_reply {
  uint32_t session_id;
  uint8_t robot_mac[6];
  uint16_t robot_port;
  uint32_t robot_ip;
} scorbot_open_reply_t; /* 16 */

/* SET_MODE: apply mode[j] to every joint whose bit is set in joint_mask. */
typedef struct SCORBOT_PACKED scorbot_set_mode_req {
  uint8_t mode[SCORBOT_MAX_JOINTS];
  uint8_t joint_mask;
  uint8_t reserved[3];
} scorbot_set_mode_req_t; /* 12 */

/* SET_GAINS / GET_GAINS payload for one joint. */
typedef struct SCORBOT_PACKED scorbot_gains {
  uint8_t joint;
  uint8_t reserved[3];
  float kp;
  float ki;
  float kd;
  float i_clamp;      /* integral accumulator limit, duty units           */
  float out_clamp;    /* maximum |duty| the loop may command              */
  float deadband_deg; /* error below this commands zero duty              */
  float max_vel_dps;  /* slew limit applied to the position setpoint      */
} scorbot_gains_t;    /* 32 */

/* How a joint establishes its reference. */
enum scorbot_home_method {
  /* Seek a limit switch, back off, then creep back onto it slowly. */
  SCORBOT_HOME_METHOD_SWITCH = 0,
  /*
   * Drive gently into a mechanical hard stop and take that as the reference.
   * For joints with no limit switch at all, such as a gripper jaw, where the
   * end of travel is the only repeatable feature there is.
   */
  SCORBOT_HOME_METHOD_STALL = 1
};

/* SET_HOMING_CFG / GET_HOMING_CFG payload for one joint. */
typedef struct SCORBOT_PACKED scorbot_homing_cfg {
  uint8_t joint;
  int8_t direction; /* -1 drives toward the switch on every joint today   */
  uint8_t order;    /* sequence group; equal values home concurrently     */
  uint8_t enabled;
  uint8_t method;   /* enum scorbot_home_method                           */
  uint8_t reserved[3];
  float seek_duty;        /* fast approach duty, sign applied separately  */
  float creep_duty;       /* slow re-approach duty                        */
  float backoff_deg;      /* retreat far enough to release the switch     */
  float home_offset_deg;  /* angle assigned once the switch latches       */
  uint32_t timeout_ms;    /* per-state deadline                           */
  float stall_eps_deg;    /* motion below this counts as stalled          */
  uint32_t stall_window_ms;
} scorbot_homing_cfg_t; /* 36 */

/* HOME request. Refuses to run unless confirm == SCORBOT_HOME_CONFIRM. */
typedef struct SCORBOT_PACKED scorbot_home_req {
  uint8_t joint_mask; /* 0xFF homes everything in configured order        */
  uint8_t reserved[3];
  uint32_t confirm;
} scorbot_home_req_t; /* 8 */

typedef struct SCORBOT_PACKED scorbot_home_status {
  uint8_t state;      /* enum scorbot_homing_state                        */
  uint8_t joint;      /* joint currently moving, 0xFF if none             */
  uint8_t order;      /* order group in progress                          */
  uint8_t error;      /* enum scorbot_fault                               */
  uint8_t homed_mask; /* bit N set once joint N has a reference           */
  uint8_t reserved[3];
  uint32_t elapsed_ms;
} scorbot_home_status_t; /* 12 */

/* SET_LIMITS: soft travel limits for one joint. */
typedef struct SCORBOT_PACKED scorbot_limits {
  uint8_t joint;
  uint8_t reserved[3];
  float min_angle;
  float max_angle;
} scorbot_limits_t; /* 12 */

/* ZERO: declare the current position of one joint to be the given angle. */
typedef struct SCORBOT_PACKED scorbot_zero_req {
  uint8_t joint;
  uint8_t reserved[3];
  float angle;
} scorbot_zero_req_t; /* 8 */

/* STREAM_START: arm the data plane. */
typedef struct SCORBOT_PACKED scorbot_stream_req {
  uint32_t state_period_us; /* 0 selects the firmware default of 1 kHz    */
  uint32_t watchdog_ms;
} scorbot_stream_req_t; /* 8 */

/* Largest control datagram, used to size receive buffers on both ends. */
#define SCORBOT_CTRL_MAX_PAYLOAD 256u
#define SCORBOT_CTRL_MAX_DATAGRAM (sizeof(scorbot_ctrl_hdr_t) + SCORBOT_CTRL_MAX_PAYLOAD)

/* ------------------------------------------------------------------ */
/* Layout assertions - these are the actual contract                   */
/* ------------------------------------------------------------------ */

SCORBOT_STATIC_ASSERT(sizeof(scorbot_pose_t) == 64, "pose must stay 64 bytes");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, magic) == 0, "pose.magic");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, version) == 4, "pose.version");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, flags) == 6, "pose.flags");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, session_id) == 8, "pose.session_id");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, seq) == 12, "pose.seq");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, t_tx_ns) == 16, "pose.t_tx_ns");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, mode) == 24, "pose.mode");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_pose_t, setpoint) == 32, "pose.setpoint");

SCORBOT_STATIC_ASSERT(sizeof(scorbot_state_t) == 184, "state must stay 184 bytes");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, magic) == 0, "state.magic");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, version) == 4, "state.version");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, flags) == 6, "state.flags");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, session_id) == 8, "state.session_id");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, seq) == 12, "state.seq");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, t_echo_ns) == 16, "state.t_echo_ns");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, echo_seq) == 24, "state.echo_seq");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, t_fw_us) == 28, "state.t_fw_us");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, limit_mask) == 32, "state.limit_mask");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, homing_state) == 34, "state.homing_state");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, homing_joint) == 35, "state.homing_joint");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, fault) == 36, "state.fault");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, reserved) == 37, "state.reserved");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, missed_deadlines) == 38, "state.missed_deadlines");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, mode) == 40, "state.mode");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, jflags) == 48, "state.jflags");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, position) == 56, "state.position");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, velocity) == 88, "state.velocity");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, effort) == 120, "state.effort");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_state_t, encoder) == 152, "state.encoder");

SCORBOT_STATIC_ASSERT(sizeof(scorbot_ctrl_hdr_t) == 24, "ctrl header must stay 24 bytes");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_ctrl_hdr_t, xid) == 8, "ctrl.xid");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_ctrl_hdr_t, session_id) == 12, "ctrl.session_id");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_ctrl_hdr_t, status) == 16, "ctrl.status");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_ctrl_hdr_t, payload_len) == 18, "ctrl.payload_len");

SCORBOT_STATIC_ASSERT(sizeof(scorbot_hello_reply_t) == 220, "hello reply size");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_hello_reply_t, joint_name) == 28, "hello.joint_name");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_hello_reply_t, min_angle) == 156, "hello.min_angle");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_hello_reply_t, max_angle) == 188, "hello.max_angle");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_open_req_t) == 20, "open request size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_open_reply_t) == 16, "open reply size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_set_mode_req_t) == 12, "set mode size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_gains_t) == 32, "gains size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_homing_cfg_t) == 36, "homing config size");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_homing_cfg_t, method) == 4, "homing.method");
SCORBOT_STATIC_ASSERT(offsetof(scorbot_homing_cfg_t, seek_duty) == 8, "homing.seek_duty");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_home_req_t) == 8, "home request size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_home_status_t) == 12, "home status size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_limits_t) == 12, "limits size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_zero_req_t) == 8, "zero request size");
SCORBOT_STATIC_ASSERT(sizeof(scorbot_stream_req_t) == 8, "stream request size");

/* A float that is not IEEE-754 binary32 would silently corrupt every angle. */
SCORBOT_STATIC_ASSERT(sizeof(float) == 4, "float must be 32-bit");

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* SCORBOT_PROTO_H_ */
