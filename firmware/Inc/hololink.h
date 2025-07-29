#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "lwip/api.h"
#include "lwip/prot/udp.h"
#include "lwip/prot/ip4.h"

// we're on LE, so we need to swap the bytes
#define ntohl(x) __builtin_bswap32(x)
#define htonl(x) __builtin_bswap32(x)
#define ntohs(x) __builtin_bswap16(x)
#define htons(x) __builtin_bswap16(x)

typedef struct __attribute__((packed)) {
    uint8_t op;                      // Message op code / message type
    uint8_t hardware_type;           // Hardware address type
    uint8_t hardware_address_length; // Hardware address length
    uint8_t hops;                    // Client sets to zero, optionally used by gateways
    uint32_t transaction_id;         // Transaction ID
    uint16_t seconds;               // Seconds elapsed since client started trying to boot
    uint16_t flags;                 // Flags
    uint32_t client_ip_address;     // Client IP address
    uint32_t your_ip_address;       // 'Your' IP address
    uint32_t server_ip_address;     // Server IP address
    uint32_t gateway_ip_address;    // Gateway IP address
    uint8_t hardware_address[16];   // Client hardware address
    char server_hostname[64];       // Server hostname
    char boot_file_name[128];       // Boot file name
    uint8_t vendor_tag;             // Vendor tag
    uint8_t vendor_tag_length;      // Vendor length
    uint32_t vendor_id;             // Vendor ID
    uint8_t data_plane;             // Data plane
    uint8_t enum_version;           // Enum version
    // everything below is little endian!!
    uint16_t board_id;              // Board ID
    char board_version[20];         // Board version
    char serial_number[7];          // Serial number
    uint16_t cpnx_version;          // CPNX version
    uint16_t cpnx_crc;             // CPNX CRC
    uint16_t clnx_version;          // CLNX version
    uint16_t clnx_crc;             // CLNX CRC
} bootp_request_t;







typedef struct __attribute__((packed)) {
    uint8_t op_code;
    uint8_t flags;
    uint16_t pkey;
    uint32_t qp_becn;
    uint32_t psn_ack;
    uint64_t address;
    uint32_t rkey;
    uint32_t size;
    uint32_t imm_data;
    uint8_t payload[100];
    uint32_t crc32;
} rocev2_packet;


typedef struct __attribute__((packed)) {
    struct ip_hdr iphdr;
    struct udp_hdr udphdr;
    rocev2_packet data;
} data_packet;

typedef struct {
     struct netconn *enumerator;
     struct netconn *control;
     struct netconn *data;
     uint8_t enumeration_packet[sizeof(bootp_request_t)];
     uint8_t control_recv_buf[1024];
     uint16_t host_port;
     ip_addr_t host_ip;
     bool streaming_enabled;
     data_packet packet;
} hololink_client_t;

// constexpr uint32_t HOLOLINK_LITE_BOARD_ID = 2u;
// constexpr uint32_t HOLOLINK_100G_BOARD_ID = 3u;
// constexpr uint32_t MICROCHIP_POLARFIRE_BOARD_ID = 4u;
// constexpr uint32_t HOLOLINK_NANO_BOARD_ID = 5u;

#define HOLOLINK_LITE_BOARD_ID 2u
#define HOLOLINK_100G_BOARD_ID 3u
#define MICROCHIP_POLARFIRE_BOARD_ID 4u
#define HOLOLINK_NANO_BOARD_ID 5u

#define CPNX_VERSION 0x2410


#define HOLOLINK_BOOTP_REQUEST_PORT 12267
#define HOLOLINK_BOOTP_RESPONSE_PORT 12268

#define HOLOLINK_CONTROL_PORT 8192

#define HOLOLINK_VENDOR_ID 0x4E564441
#define HOLOLINK_VENDOR_TAG 0xE0
#define HOLOLINK_ENUM_VERSION 0x01


void hololink_init(hololink_client_t* client);
void hololink_send_enumeration_packet(hololink_client_t* client);
void hololink_task(void* arg);
