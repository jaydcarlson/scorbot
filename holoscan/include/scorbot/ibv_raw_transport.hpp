// ConnectX raw packet queue pair backend.
//
// Uses IBV_QPT_RAW_PACKET, which is not RoCE: the RDMA stack is being used
// purely as a userspace path for injecting and extracting raw Ethernet frames.
// A flow steering rule matching our destination MAC, IP and UDP port delivers
// just our traffic to the queue pair, so everything else on the port continues
// to reach the kernel normally.
//
// Transmit uses one pre-built frame with only the payload patched per send,
// posted inline so the whole thing is copied into the work queue entry at
// doorbell time. Receive completions carry NIC hardware timestamps.

#ifndef SCORBOT_IBV_RAW_TRANSPORT_HPP_
#define SCORBOT_IBV_RAW_TRANSPORT_HPP_

#include <cstdint>
#include <vector>

#include "scorbot/transport.hpp"

#ifdef SCORBOT_HAVE_IBVERBS
struct ibv_context;
struct ibv_pd;
struct ibv_cq_ex;
struct ibv_qp;
struct ibv_mr;
struct ibv_flow;
#endif

namespace scorbot {

class IbvRawTransport final : public Transport {
 public:
  IbvRawTransport() = default;
  ~IbvRawTransport() override;

  IbvRawTransport(const IbvRawTransport&) = delete;
  IbvRawTransport& operator=(const IbvRawTransport&) = delete;

  Status open(const TransportConfig& cfg) override;
  void close() override;
  [[nodiscard]] bool is_open() const override { return open_; }

  bool send(const void* payload, size_t len) override;
  size_t poll_recv(void* buf, size_t cap, RxMeta* meta) override;
  void poll_send() override;

  [[nodiscard]] bool has_hw_timestamps() const override { return hw_timestamps_; }
  [[nodiscard]] uint64_t hw_ts_to_ns(uint64_t raw) const override;
  [[nodiscard]] const char* name() const override { return "ibverbs"; }
  [[nodiscard]] uint16_t local_port() const override { return cfg_.src_port; }

  // Completion timestamp of the most recent send, in NIC clock units. Lets the
  // ping tool measure a true wire-to-wire round trip rather than a
  // software-stamped one.
  [[nodiscard]] uint64_t last_send_hw_ts() const { return last_send_hw_ts_; }

 private:
#ifdef SCORBOT_HAVE_IBVERBS
  Status open_device();
  Status create_qp();
  Status alloc_buffers();
  Status install_flow_rule();
  Status to_ready_state();
  Status post_recv(uint32_t slot);

  ibv_context* ctx_ = nullptr;
  ibv_pd* pd_ = nullptr;
  ibv_cq_ex* send_cq_ = nullptr;
  ibv_cq_ex* recv_cq_ = nullptr;
  ibv_qp* qp_ = nullptr;
  ibv_mr* mr_ = nullptr;
  ibv_flow* flow_ = nullptr;

  uint8_t* buffer_ = nullptr;   // registered region: RX slots then the TX frame
  size_t buffer_len_ = 0;
  bool buffer_is_hugepage_ = false;
  uint8_t* tx_frame_ = nullptr;
  size_t tx_frame_len_ = 0;
  size_t slot_len_ = 0;
  uint32_t next_recv_slot_ = 0;
  uint32_t inflight_sends_ = 0;
  double ns_per_tick_ = 0.0;
#endif

  TransportConfig cfg_;
  bool open_ = false;
  bool hw_timestamps_ = false;
  uint64_t last_send_hw_ts_ = 0;
};

}  // namespace scorbot

#endif  // SCORBOT_IBV_RAW_TRANSPORT_HPP_
