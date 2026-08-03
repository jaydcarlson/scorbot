#include "scorbot/ibv_raw_transport.hpp"

#include <cstring>

#include "scorbot/eth_frame.hpp"

#ifdef SCORBOT_HAVE_IBVERBS

#include <arpa/inet.h>
#include <infiniband/verbs.h>
#include <sys/mman.h>

#include <cerrno>
#include <cstdlib>

namespace scorbot {
namespace {

constexpr size_t kHugePageBytes = 2 * 1024 * 1024;

Status verbs_error(const char* what) {
  return Status::error(std::string(what) + ": " + std::strerror(errno), errno);
}

// One packed blob holding the flow attributes and all three match specs, which
// is the layout ibv_create_flow expects.
struct FlowRule {
  ibv_flow_attr attr;
  ibv_flow_spec_eth eth;
  ibv_flow_spec_ipv4 ipv4;
  ibv_flow_spec_tcp_udp udp;
} __attribute__((packed));

}  // namespace

IbvRawTransport::~IbvRawTransport() { close(); }

Status IbvRawTransport::open(const TransportConfig& cfg) {
  close();
  cfg_ = cfg;

  if (cfg_.src_mac.is_zero()) {
    return Status::error("ibverbs transport needs src_mac (there is no ARP on a raw QP)");
  }
  if (cfg_.dst_mac.is_zero()) {
    return Status::error("ibverbs transport needs dst_mac (there is no ARP on a raw QP)");
  }
  if (cfg_.src_ip == 0 || cfg_.dst_ip == 0 || cfg_.src_port == 0 || cfg_.dst_port == 0) {
    return Status::error("ibverbs transport needs both endpoints fully specified");
  }

  if (Status s = open_device(); !s) {
    close();
    return s;
  }
  if (Status s = create_qp(); !s) {
    close();
    return s;
  }
  if (Status s = to_ready_state(); !s) {
    close();
    return s;
  }
  if (Status s = alloc_buffers(); !s) {
    close();
    return s;
  }

  // Post the whole receive ring before steering traffic at it, so no frame can
  // arrive with nowhere to land.
  for (uint32_t i = 0; i < cfg_.rx_depth; ++i) {
    if (Status s = post_recv(i); !s) {
      close();
      return s;
    }
  }

  if (Status s = install_flow_rule(); !s) {
    close();
    return s;
  }

  // Build the outgoing frame once. Only the payload changes per send.
  tx_frame_len_ = build_udp_frame(cfg_.src_mac, cfg_.dst_mac, cfg_.src_ip, cfg_.dst_ip,
                                  cfg_.src_port, cfg_.dst_port, nullptr, 0, tx_frame_);

  open_ = true;
  return Status::ok();
}

Status IbvRawTransport::open_device() {
  int num_devices = 0;
  ibv_device** dev_list = ibv_get_device_list(&num_devices);
  if (dev_list == nullptr) {
    return verbs_error("ibv_get_device_list");
  }

  ibv_device* dev = nullptr;
  std::string available;
  for (int i = 0; i < num_devices; ++i) {
    const char* dev_name = ibv_get_device_name(dev_list[i]);
    if (dev_name == nullptr) {
      continue;
    }
    if (!available.empty()) {
      available += ", ";
    }
    available += dev_name;
    if (cfg_.ibv_device == dev_name) {
      dev = dev_list[i];
    }
  }
  if (dev == nullptr) {
    ibv_free_device_list(dev_list);
    return Status::error("RDMA device '" + cfg_.ibv_device + "' not found; available: " +
                         (available.empty() ? std::string("none") : available));
  }

  ctx_ = ibv_open_device(dev);
  ibv_free_device_list(dev_list);
  if (ctx_ == nullptr) {
    return verbs_error("ibv_open_device");
  }

  pd_ = ibv_alloc_pd(ctx_);
  if (pd_ == nullptr) {
    return verbs_error("ibv_alloc_pd");
  }

  ibv_device_attr_ex dattr{};
  if (ibv_query_device_ex(ctx_, nullptr, &dattr) != 0) {
    return verbs_error("ibv_query_device_ex");
  }
  if (dattr.hca_core_clock != 0) {
    // hca_core_clock is in kHz, so nanoseconds per tick is 1e6 / kHz.
    ns_per_tick_ = 1e6 / static_cast<double>(dattr.hca_core_clock);
    hw_timestamps_ = true;
  }

  return Status::ok();
}

Status IbvRawTransport::create_qp() {
  ibv_cq_init_attr_ex cq_attr{};
  cq_attr.wc_flags = static_cast<uint64_t>(IBV_WC_STANDARD_FLAGS);
  if (hw_timestamps_) {
    cq_attr.wc_flags |= static_cast<uint64_t>(IBV_WC_EX_WITH_COMPLETION_TIMESTAMP);
  }
  cq_attr.cqe = cfg_.tx_depth;
  send_cq_ = ibv_create_cq_ex(ctx_, &cq_attr);
  cq_attr.cqe = cfg_.rx_depth;
  recv_cq_ = ibv_create_cq_ex(ctx_, &cq_attr);
  if (send_cq_ == nullptr || recv_cq_ == nullptr) {
    return verbs_error("ibv_create_cq_ex");
  }

  ibv_qp_init_attr qp_attr{};
  qp_attr.qp_type = IBV_QPT_RAW_PACKET;
  qp_attr.send_cq = ibv_cq_ex_to_cq(send_cq_);
  qp_attr.recv_cq = ibv_cq_ex_to_cq(recv_cq_);
  qp_attr.cap.max_send_wr = cfg_.tx_depth;
  qp_attr.cap.max_recv_wr = cfg_.rx_depth;
  qp_attr.cap.max_send_sge = 1;
  qp_attr.cap.max_recv_sge = 1;
  // 128 bytes covers our 106-byte pose frame, so the common case is an inline
  // send with no DMA read of a separate buffer.
  qp_attr.cap.max_inline_data = 128;

  qp_ = ibv_create_qp(pd_, &qp_attr);
  if (qp_ == nullptr) {
    return Status::error(
        std::string("ibv_create_qp failed: ") + std::strerror(errno) +
            " (raw packet queue pairs need CAP_NET_RAW; try running under sudo or "
            "setcap cap_net_raw,cap_sys_nice+ep)",
        errno);
  }
  return Status::ok();
}

Status IbvRawTransport::to_ready_state() {
  ibv_qp_attr attr{};
  attr.qp_state = IBV_QPS_INIT;
  attr.port_num = cfg_.ibv_port;
  if (ibv_modify_qp(qp_, &attr, IBV_QP_STATE | IBV_QP_PORT) != 0) {
    return verbs_error("ibv_modify_qp to INIT");
  }

  std::memset(&attr, 0, sizeof(attr));
  attr.qp_state = IBV_QPS_RTR;
  if (ibv_modify_qp(qp_, &attr, IBV_QP_STATE) != 0) {
    return verbs_error("ibv_modify_qp to RTR");
  }

  std::memset(&attr, 0, sizeof(attr));
  attr.qp_state = IBV_QPS_RTS;
  if (ibv_modify_qp(qp_, &attr, IBV_QP_STATE) != 0) {
    return verbs_error("ibv_modify_qp to RTS");
  }
  return Status::ok();
}

Status IbvRawTransport::alloc_buffers() {
  slot_len_ = cfg_.max_frame;
  // Receive ring followed by a single transmit frame, all in one registration.
  buffer_len_ = slot_len_ * (static_cast<size_t>(cfg_.rx_depth) + 1U);

  if (cfg_.hugepages) {
    const size_t rounded = (buffer_len_ + kHugePageBytes - 1) & ~(kHugePageBytes - 1);
    void* p = mmap(nullptr, rounded, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
    if (p != MAP_FAILED) {
      buffer_ = static_cast<uint8_t*>(p);
      buffer_len_ = rounded;
      buffer_is_hugepage_ = true;
    }
    // Falling through on failure is deliberate: no huge pages reserved just
    // means slightly more address translation work for the NIC.
  }

  if (buffer_ == nullptr) {
    const size_t rounded = (buffer_len_ + 4095U) & ~static_cast<size_t>(4095U);
    buffer_ = static_cast<uint8_t*>(std::aligned_alloc(4096, rounded));
    if (buffer_ == nullptr) {
      return Status::error("could not allocate DMA buffer");
    }
    buffer_len_ = rounded;
  }
  std::memset(buffer_, 0, buffer_len_);

  mr_ = ibv_reg_mr(pd_, buffer_, buffer_len_, IBV_ACCESS_LOCAL_WRITE);
  if (mr_ == nullptr) {
    return verbs_error("ibv_reg_mr");
  }

  tx_frame_ = buffer_ + slot_len_ * static_cast<size_t>(cfg_.rx_depth);
  return Status::ok();
}

Status IbvRawTransport::install_flow_rule() {
  FlowRule rule{};
  rule.attr.type = IBV_FLOW_ATTR_NORMAL;
  rule.attr.size = sizeof(rule);
  rule.attr.num_of_specs = 3;
  rule.attr.port = cfg_.ibv_port;
  rule.attr.priority = 0;

  // Match only frames addressed to us, on our IP, on our UDP port. Everything
  // else on this physical port keeps flowing to the kernel untouched.
  rule.eth.type = IBV_FLOW_SPEC_ETH;
  rule.eth.size = sizeof(rule.eth);
  std::memcpy(rule.eth.val.dst_mac, cfg_.src_mac.b, 6);
  std::memset(rule.eth.mask.dst_mac, 0xFF, 6);

  rule.ipv4.type = IBV_FLOW_SPEC_IPV4;
  rule.ipv4.size = sizeof(rule.ipv4);
  rule.ipv4.val.dst_ip = htonl(cfg_.src_ip);
  rule.ipv4.mask.dst_ip = 0xFFFFFFFFU;

  rule.udp.type = IBV_FLOW_SPEC_UDP;
  rule.udp.size = sizeof(rule.udp);
  rule.udp.val.dst_port = htons(cfg_.src_port);
  rule.udp.mask.dst_port = 0xFFFFU;

  // attr sits at offset 0 of the packed struct, so taking its address is fine.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  flow_ = ibv_create_flow(qp_, &rule.attr);
#pragma GCC diagnostic pop

  if (flow_ == nullptr) {
    return Status::error(
        std::string("ibv_create_flow failed: ") + std::strerror(errno) +
            " (flow steering unsupported, or missing CAP_NET_RAW)",
        errno);
  }
  return Status::ok();
}

Status IbvRawTransport::post_recv(uint32_t slot) {
  ibv_sge sge{};
  sge.addr = reinterpret_cast<uintptr_t>(buffer_ + static_cast<size_t>(slot) * slot_len_);
  sge.length = static_cast<uint32_t>(slot_len_);
  sge.lkey = mr_->lkey;

  ibv_recv_wr wr{};
  wr.wr_id = slot;
  wr.sg_list = &sge;
  wr.num_sge = 1;

  ibv_recv_wr* bad = nullptr;
  if (ibv_post_recv(qp_, &wr, &bad) != 0) {
    return verbs_error("ibv_post_recv");
  }
  return Status::ok();
}

void IbvRawTransport::close() {
  if (flow_ != nullptr) {
    ibv_destroy_flow(flow_);
    flow_ = nullptr;
  }
  if (qp_ != nullptr) {
    ibv_destroy_qp(qp_);
    qp_ = nullptr;
  }
  if (mr_ != nullptr) {
    ibv_dereg_mr(mr_);
    mr_ = nullptr;
  }
  if (send_cq_ != nullptr) {
    ibv_destroy_cq(ibv_cq_ex_to_cq(send_cq_));
    send_cq_ = nullptr;
  }
  if (recv_cq_ != nullptr) {
    ibv_destroy_cq(ibv_cq_ex_to_cq(recv_cq_));
    recv_cq_ = nullptr;
  }
  if (pd_ != nullptr) {
    ibv_dealloc_pd(pd_);
    pd_ = nullptr;
  }
  if (ctx_ != nullptr) {
    ibv_close_device(ctx_);
    ctx_ = nullptr;
  }
  if (buffer_ != nullptr) {
    if (buffer_is_hugepage_) {
      munmap(buffer_, buffer_len_);
    } else {
      std::free(buffer_);
    }
    buffer_ = nullptr;
  }
  buffer_len_ = 0;
  buffer_is_hugepage_ = false;
  tx_frame_ = nullptr;
  tx_frame_len_ = 0;
  next_recv_slot_ = 0;
  inflight_sends_ = 0;
  open_ = false;
}

bool IbvRawTransport::send(const void* payload, size_t len) {
  if (!open_ || len + kHeadersLen > slot_len_) {
    return false;
  }

  // Keep the send queue from overflowing. Every send is signalled so the ping
  // tool can read a hardware transmit timestamp.
  while (inflight_sends_ >= cfg_.tx_depth) {
    const uint32_t before = inflight_sends_;
    poll_send();
    if (inflight_sends_ == before) {
      // Spin: the NIC has not retired anything yet.
    }
  }

  // Patch the payload into the frame that was built once at open time, and fix
  // up the two length fields that depend on it.
  std::memcpy(tx_frame_ + kHeadersLen, payload, len);
  auto* ip = reinterpret_cast<Ipv4Header*>(tx_frame_ + kEthHeaderLen);
  ip->total_length = host_to_net16(static_cast<uint16_t>(kIpv4HeaderLen + kUdpHeaderLen + len));
  ip->header_checksum = 0;
  ip->header_checksum = ipv4_checksum_bytes(tx_frame_ + kEthHeaderLen);
  auto* udp = reinterpret_cast<UdpHeader*>(tx_frame_ + kEthHeaderLen + kIpv4HeaderLen);
  udp->length = host_to_net16(static_cast<uint16_t>(kUdpHeaderLen + len));

  const size_t frame_len = kHeadersLen + len;

  ibv_sge sge{};
  sge.addr = reinterpret_cast<uintptr_t>(tx_frame_);
  sge.length = static_cast<uint32_t>(frame_len);
  sge.lkey = mr_->lkey;

  ibv_send_wr wr{};
  wr.wr_id = 0;
  wr.sg_list = &sge;
  wr.num_sge = 1;
  wr.opcode = IBV_WR_SEND;
  wr.send_flags = IBV_SEND_SIGNALED;
  // Inline means the NIC copies the frame at doorbell time, so the next send
  // may overwrite tx_frame_ immediately without waiting for completion.
  if (frame_len <= 128) {
    wr.send_flags |= IBV_SEND_INLINE;
  }

  ibv_send_wr* bad = nullptr;
  if (ibv_post_send(qp_, &wr, &bad) != 0) {
    return false;
  }
  ++inflight_sends_;
  return true;
}

void IbvRawTransport::poll_send() {
  if (send_cq_ == nullptr) {
    return;
  }
  ibv_poll_cq_attr attr{};
  int ret = ibv_start_poll(send_cq_, &attr);
  if (ret != 0) {
    return;  // ENOENT means nothing to reap
  }
  do {
    if (hw_timestamps_) {
      last_send_hw_ts_ = ibv_wc_read_completion_ts(send_cq_);
    }
    if (inflight_sends_ > 0) {
      --inflight_sends_;
    }
    ret = ibv_next_poll(send_cq_);
  } while (ret == 0);
  ibv_end_poll(send_cq_);
}

size_t IbvRawTransport::poll_recv(void* buf, size_t cap, RxMeta* meta) {
  if (!open_) {
    return 0;
  }

  ibv_poll_cq_attr attr{};
  if (ibv_start_poll(recv_cq_, &attr) != 0) {
    return 0;  // ENOENT: nothing pending
  }

  const auto slot = static_cast<uint32_t>(recv_cq_->wr_id);
  size_t copied = 0;

  if (recv_cq_->status == IBV_WC_SUCCESS) {
    const uint32_t byte_len = ibv_wc_read_byte_len(recv_cq_);
    const uint64_t hw_ts = hw_timestamps_ ? ibv_wc_read_completion_ts(recv_cq_) : 0;
    const uint8_t* frame = buffer_ + static_cast<size_t>(slot) * slot_len_;

    ParsedFrame parsed;
    if (parse_udp_frame(frame, byte_len, &parsed) && parsed.payload_len <= cap) {
      std::memcpy(buf, parsed.payload, parsed.payload_len);
      copied = parsed.payload_len;
      if (meta != nullptr) {
        meta->hw_timestamp = hw_ts;
        meta->sw_timestamp_ns = 0;
      }
    }
  }

  // Take exactly one completion per call; anything else stays queued for the
  // caller's next poll. End the poll before re-posting so the CQ is released.
  ibv_end_poll(recv_cq_);
  (void)post_recv(slot);

  return copied;
}

uint64_t IbvRawTransport::hw_ts_to_ns(uint64_t raw) const {
  return static_cast<uint64_t>(static_cast<double>(raw) * ns_per_tick_);
}

}  // namespace scorbot

#else  // !SCORBOT_HAVE_IBVERBS

namespace scorbot {

IbvRawTransport::~IbvRawTransport() = default;

Status IbvRawTransport::open(const TransportConfig& /*cfg*/) {
  return Status::error("this build has no ibverbs support; install rdma-core development headers");
}

void IbvRawTransport::close() {}

bool IbvRawTransport::send(const void* /*payload*/, size_t /*len*/) { return false; }

size_t IbvRawTransport::poll_recv(void* /*buf*/, size_t /*cap*/, RxMeta* /*meta*/) { return 0; }

void IbvRawTransport::poll_send() {}

uint64_t IbvRawTransport::hw_ts_to_ns(uint64_t raw) const { return raw; }

}  // namespace scorbot

#endif  // SCORBOT_HAVE_IBVERBS
