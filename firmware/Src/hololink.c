#include "hololink.h"
#include "lwip/api.h"
#include "lwip/udp.h"
#include <string.h>
#include "crc32.h"


// packet command byte
// constexpr uint32_t WR_DWORD = 0x04;
// constexpr uint32_t RD_DWORD = 0x14;

#define CMD_WR_DWORD 0x04
#define CMD_RD_DWORD 0x14

// registers
#define REG_FPGA_VERSION 0x80
#define REG_FPGA_DATE 0x84
#define REG_FPGA_STATUS 0x88
// SPI interfaces
#define REG_CLNX_SPI_CTRL 0x03000000
#define REG_CPNX_SPI_CTRL 0x03000200
// I2C interfaces
#define REG_BL_I2C_CTRL 0x04000000
#define REG_CAM_I2C_CTRL 0x04000200


// configuration
#define REG_NETWORK_CONFIG_BASE 0x2000000
// Note that these are offsets enumeration metadata "configuration_address".
#define REG_DP_PACKET_SIZE 0x304
#define REG_DP_VIP_MASK 0x30C


#define REG_SENSOR_CONFIG_BASE 0x0
// // DMA descriptor registers.
#define REG_DP_QP 0x1000
#define REG_DP_RKEY 0x1004
// // these are all page addresses; the actual byte address in the
// // packet will be this page address * 128.
#define REG_DP_ADDRESS_0 0x1008
#define REG_DP_ADDRESS_1 0x100C
#define REG_DP_ADDRESS_2 0x1010
#define REG_DP_ADDRESS_3 0x1014

// this is in bytes
#define REG_DP_BUFFER_LENGTH 0x1018 

// each bit enables a buffer
#define REG_DP_BUFFER_MASK 0x101C
#define REG_DP_HOST_MAC_LOW 0x1020
#define REG_DP_HOST_MAC_HIGH 0x1024
#define REG_DP_HOST_IP 0x1028
#define REG_DP_HOST_UDP_PORT 0x102C

#define ntohll(x) __builtin_bswap64(x)
#define htonll(x) __builtin_bswap64(x)

#define member_size(type, member) (sizeof( ((type *)0)->member ))

typedef struct __attribute__((packed)) {
    uint32_t size;
    uint32_t reserved;
    uint32_t vip_mask;
} hololink_dp_packet_config_t;

typedef struct __attribute__((packed)) {
    uint32_t qp;
    uint32_t rkey;
    uint32_t address_0;
    uint32_t address_1;
    uint32_t address_2;
    uint32_t address_3;
    uint32_t buffer_length;
    uint32_t buffer_mask;
    uint32_t host_mac_low;
    uint32_t host_mac_high;
    uint32_t host_ip;
    uint32_t host_udp_port;
} hololink_dp_config_t;

hololink_dp_packet_config_t hololink_dp_packet_config;
hololink_dp_config_t hololink_dp_config;

typedef struct __attribute__((packed)) {
uint8_t command;
uint8_t flags;
uint16_t sequence;
uint8_t reserved[2];
uint32_t address;
uint32_t data;
uint16_t latched_sequence;
} hololink_op_rd_wr_t;

static inline void hololink_packet_hton(rocev2_packet* packet)
{
    packet->pkey = htons(packet->pkey);
    packet->qp_becn = htonl(packet->qp_becn);
    packet->psn_ack = htonl(packet->psn_ack);
    packet->address = htonll(packet->address);
    packet->rkey = htonl(packet->rkey);
    packet->size = htonl(packet->size);
    // packet->crc32 = htonl(packet->crc32); // we do this manually after iCRC calculation
}

static inline void hololink_op_rd_wr_hton(hololink_op_rd_wr_t* op)
{
    op->sequence = htons(op->sequence);
    op->address = htonl(op->address);
    op->data = htonl(op->data);
    op->latched_sequence = htons(op->latched_sequence);
}

static inline void hololink_op_rd_wr_ntoh(hololink_op_rd_wr_t* op)
{
    op->sequence = ntohs(op->sequence);
    op->address = ntohl(op->address);
    op->data = ntohl(op->data);
    op->latched_sequence = ntohs(op->latched_sequence);
}

bootp_request_t hololink_request = {
    .hardware_address_length = 6,
    .hops = 0,
    .transaction_id = 0,
    .seconds = 0,
    .flags = 0,
    .client_ip_address = 0,
    .your_ip_address = 0,
    .server_ip_address = 0,
    .gateway_ip_address = 0,
    .hardware_address = {0x00, 0x80, 0xe1, 0x00, 0x00, 0x00},
    .vendor_tag = HOLOLINK_VENDOR_TAG,
    .vendor_tag_length = 0,
    .vendor_id = HOLOLINK_VENDOR_ID,
    .data_plane = 0,
    .enum_version = HOLOLINK_ENUM_VERSION,
    .cpnx_version = CPNX_VERSION,
    .board_id = HOLOLINK_LITE_BOARD_ID
};

static inline void holoscan_bootp_ntoh(bootp_request_t* packet);
static inline void holoscan_bootp_hton(bootp_request_t* packet);

hololink_client_t hololink_client;


void read_word_cmd(hololink_client_t* client, uint8_t* cmd)
{
    hololink_op_rd_wr_t* op = (hololink_op_rd_wr_t*)cmd;
    printf("Requested read of word at address 0x%04x\r\n", op->address);
    switch(op->address)
    {
        case REG_FPGA_VERSION:
            printf("REG_FPGA_VERSION\r\n");
            op->data = 0x00000001;
            break;
        case REG_FPGA_DATE:
            printf("REG_FPGA_DATE\r\n");
            op->data = 0x00000002;
            break;
        default:
            // check to see if address is in the range of (REG_CONFIG_BASE + REG_DP_PACKET_SIZE) and (REG_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t))
            if(op->address >= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE) && op->address <= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t)))
            {
                printf("DP register address 0x%04x\r\n", op->address);
                uint32_t offset = (op->address - (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE)) + (uint32_t)&hololink_dp_packet_config;
                op->data = *(uint32_t*)offset;
            } else 
            // check to see if address is in the range of (REG_CONFIG_BASE + REG_DP_QP) and (REG_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t))
            if(op->address >= (REG_SENSOR_CONFIG_BASE + REG_DP_QP) && op->address <= (REG_SENSOR_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t)))
            {
                printf("DP Config register address 0x%04x\r\n", op->address);
                uint32_t offset = (op->address - (REG_SENSOR_CONFIG_BASE + REG_DP_QP)) + (uint32_t)&hololink_dp_config;
                op->data = *(uint32_t*)offset;  
            } else {
                printf("Unknown register address 0x%04x\r\n", op->address);
                return;
            }
    }
    printf("Returning data 0x%04x\r\n", op->data);
}

void update_from_config(hololink_client_t* client)
{
    if(hololink_dp_packet_config.vip_mask == 1 && client->streaming_enabled == false)
    {
        printf("\r\n\r\n\r\nStart streaming to port %d\r\n\r\n\r\n", hololink_dp_config.host_udp_port);
        client->host_port = hololink_dp_config.host_udp_port;
        client->streaming_enabled = true;
    } else if(hololink_dp_packet_config.vip_mask == 0 && client->streaming_enabled == true) {
        printf("\r\n\r\n\r\nStop streaming\r\n\r\n\r\n");
        client->streaming_enabled = false;
    }
    
}

void write_word_cmd(hololink_client_t* client, uint8_t* cmd)
{
    hololink_op_rd_wr_t* op = (hololink_op_rd_wr_t*)cmd;
    printf("Requested write of word at address 0x%04x\r\n", op->address);
    printf("Data: 0x%04x\r\n", op->data);
    switch(op->address)
    {
        default:
            // check to see if address is in the range of (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE) and (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t))
            if(op->address >= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE) && op->address <= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t)))
            {
                printf("DP register address 0x%04x\r\n", op->address);
                uint32_t offset = (op->address - (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE)) + (uint32_t)&hololink_dp_packet_config;
                *(uint32_t*)offset = op->data;
            } else 
            // check to see if address is in the range of (REG_SENSOR_CONFIG_BASE + REG_DP_QP) and (REG_SENSOR_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t))
            if(op->address >= (REG_SENSOR_CONFIG_BASE + REG_DP_QP) && op->address <= (REG_SENSOR_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t)))
            {
                printf("DP Config register address 0x%04x\r\n", op->address);
                uint32_t offset = (op->address - (REG_SENSOR_CONFIG_BASE + REG_DP_QP)) + (uint32_t)&hololink_dp_config;
                *(uint32_t*)offset = op->data;  
            } else {
                printf("Unknown register address 0x%04x\r\n", op->address);
                return;
            }
    }

    update_from_config(client);
}

void process_packet(hololink_client_t* client, uint8_t* packet)
{
    hololink_op_rd_wr_t* op = (hololink_op_rd_wr_t*)packet;
    hololink_op_rd_wr_ntoh(op);
    // printf("Received command 0x%02x\r\n, sequence 0x%02x\r\n, address 0x%04x\r\n, data 0x%04x\r\n", op->command, op->sequence, op->address, op->data);
    switch(op->command)
    {
        case CMD_RD_DWORD:
            read_word_cmd(client, packet);
            break;
        case CMD_WR_DWORD:
            write_word_cmd(client, packet);
            break;
        default:
            printf("Unknown command 0x%02x\r\n", op->command);
            break;
    }
    // printf("Sending command 0x%02x\r\n, sequence 0x%04x\r\n, address 0x%04x\r\n, data 0x%04x\r\n", op->command, op->sequence, op->address, op->data);
    hololink_op_rd_wr_hton(op);
}

void hololink_init(hololink_client_t* client)
{
    // create a UDP socket using netcon
    client->enumerator = netconn_new(NETCONN_UDP);
    if(client->enumerator == NULL)
    {
        printf("Failed to create hololink enumerator\r\n");
        return;
    }

    if(netconn_bind(client->enumerator, NULL, HOLOLINK_BOOTP_REQUEST_PORT) != ERR_OK)
    {
        printf("Failed to bind hololink enumerator\r\n");
        return;
    }

    client->control = netconn_new(NETCONN_UDP);
    if(client->control == NULL)
    {
        printf("Failed to create hololink control\r\n");
        return;
    }

    if(netconn_bind(client->control, NULL, HOLOLINK_CONTROL_PORT) != ERR_OK)
    {
        printf("Failed to bind hololink control\r\n");
        return;
    }

    client->data = netconn_new(NETCONN_UDP);
    if(client->data == NULL)
    {
        printf("Failed to create hololink data\r\n");
        return;
    }
    if(netconn_bind(client->data, 0, 0) != ERR_OK)
    {
        printf("Failed to bind hololink data\r\n");
        return;
    }
    
}

void hololink_send_enumeration_packet(hololink_client_t* client)
{
     extern struct netif gnetif;
    

    // set the IP address
    hololink_request.your_ip_address = gnetif.ip_addr.addr;

    // create a destination broadcast IP address (255.255.255.255)
    ip_addr_t broadcast_ip;
    IP4_ADDR(&broadcast_ip, 255, 255, 255, 255);

    struct netbuf* buffer = netbuf_new();
    // uint8_t* data = netbuf_alloc(buffer, sizeof(hololink_request));
    memcpy(client->enumeration_packet, &hololink_request, sizeof(hololink_request));
    netbuf_ref(buffer, client->enumeration_packet, sizeof(hololink_request));
    // fix the endianness (works inline)
    holoscan_bootp_hton((bootp_request_t*)client->enumeration_packet);

    // send the packet
    // netconn_send(client->hololink_enumerator, (uint8_t*)&hololink_request, sizeof(hololink_request));
    netconn_sendto(client->enumerator, buffer, &broadcast_ip, HOLOLINK_BOOTP_REQUEST_PORT);

    // netbuf_delete(data);
    netbuf_delete(buffer);
}

void holoscan_enumerator_task(void* arg)
{
  hololink_client_t* hololink_client = (hololink_client_t* )arg;
   while(1)
  {
    // printf("Holoscan Enumerator\r\n");
    hololink_send_enumeration_packet(hololink_client);
    vTaskDelay(1000);
  }
}

void holoscan_data_task(void* arg)
{
    extern struct netif gnetif;
  hololink_client_t* hololink_client = (hololink_client_t* )arg;
  uint32_t counter = 1;
  while(1)
  {
    if(hololink_client->streaming_enabled)
    {
        // shortcut accessors for the data packet
        data_packet* packet = &hololink_client->packet;
        struct ip_hdr* iphdr = &packet->iphdr;
        struct udp_hdr* udphdr = &packet->udphdr;
        rocev2_packet* rocev2 = &packet->data;

        // hololink_client->packet.op_code = 0x2A; // IBV_OPCODE_UC_RDMA_WRITE_ONLY
        // rocev2->op_code = (counter & 1) == 0 ? 0x2A : 0x2B; // IBV_OPCODE_UC_RDMA_WRITE_ONLY_WITH_IMMEDIATE
        rocev2->op_code = 0x2B; // IBV_OPCODE_UC_RDMA_WRITE_ONLY_WITH_IMMEDIATE
        // hololink_client->packet.flags = 0x80; // solicited event = true;
        rocev2->flags = 0x00; // solicited event = false;
        rocev2->pkey = 0xffff;
        // set flags to 1s for invariant CRC calculation
        rocev2->qp_becn = 0xFF << 24 | hololink_dp_config.qp;
        rocev2->rkey = hololink_dp_config.rkey;
        // hololink_client->packet.address = (uint64_t)hololink_dp_config.address_0;
        rocev2->address = (counter & 1) == 0 ? 0 : 100;
        // nasty, hardcode address to 0x00007fffd3a00000
        // hololink_client->packet.address = 0x00007fffd3a00000;
        rocev2->psn_ack = (counter) & 0xffffff; // 24 bits
        rocev2->imm_data = 0;
        rocev2->size = member_size(rocev2_packet, payload);

        rocev2->payload[0] = counter & 0xff;
        rocev2->payload[1] = (counter >> 8) & 0xff;
        rocev2->payload[2] = (counter >> 16) & 0xff;
        rocev2->payload[3] = (counter >> 24) & 0xff;

        hololink_packet_hton(rocev2);
        // Build the UDP header
        udphdr->src = lwip_htons(hololink_client->data->pcb.udp->local_port);
        udphdr->dest = lwip_htons(hololink_client->host_port);
        uint16_t udp_length = sizeof(struct udp_hdr) + sizeof(rocev2_packet);
        udphdr->len = lwip_htons(udp_length);
        // udphdr->chksum = 0; don't do this here, as we're going to overwrite to 0xFFFF for the invariant CRC calculation

        // build the IP header
        IPH_VHL_SET(iphdr, 4, IP_HLEN / 4);
        IPH_LEN_SET(iphdr, lwip_htons(sizeof(data_packet)));
        IPH_OFFSET_SET(iphdr, htons(IP_DF));
        extern u16_t ip_id;
        IPH_ID_SET(iphdr, lwip_htons(ip_id++));        
        IPH_PROTO_SET(iphdr, IP_PROTO_UDP);
        ip4_addr_copy(iphdr->src, gnetif.ip_addr);
        ip4_addr_copy(iphdr->dest, hololink_client->host_ip);

        // fix up the fields for Invariant CRC calculation
        IPH_TOS_SET(iphdr, 0xFF);
        IPH_TTL_SET(iphdr, 0xFF);
        IPH_CHKSUM_SET(iphdr, 0xFFFF);
        udphdr->chksum = 0xFFFF;



        // calculate the Invariant CRC
        // subtract 4 bytes where the CRC will go
        crc32_ctx_t crc_ctx;
        crc32_init(&crc_ctx);
        uint8_t padding[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        crc32_update(&crc_ctx, padding, sizeof(padding));
        crc32_update(&crc_ctx, (uint8_t*)packet, sizeof(data_packet)-4);
        uint32_t crc = crc32_get(&crc_ctx);
        // printf("Invariant CRC: 0x%08x\r\n", crc);
        rocev2->crc32 = crc;
        // rocev2->crc32 = crc32_swap_bytes(crc);


        // OK, now set the fields to their correct values 
        // careful! things are now in network byte order
        rocev2->qp_becn = htonl(hololink_dp_config.qp);
        IPH_TOS_SET(iphdr, 0x00);
        IPH_TTL_SET(iphdr, 64);
        IPH_CHKSUM_SET(iphdr, 0);

        /* in UDP, 0 checksum means 'no checksum' */
        udphdr->chksum = 0x0000;

        // copy the packet into a new lwip pbuf
        uint16_t total_size = sizeof(data_packet);
        struct pbuf *raw_packet = pbuf_alloc(PBUF_LINK, total_size, PBUF_RAM);
        pbuf_take(raw_packet, (uint8_t*)packet, total_size);



        gnetif.output(&gnetif, raw_packet, &hololink_client->host_ip);
        pbuf_free(raw_packet);


        // struct netbuf* buffer = netbuf_new();
        // netbuf_ref(buffer, &hololink_client->packet, sizeof(rocev2_packet));
        // netconn_sendto(hololink_client->control, buffer, &hololink_client->host_ip, hololink_client->host_port);
        // netbuf_delete(buffer);
        counter++;
        // vTaskDelay(1);
    } else {
        counter = 0;
        vTaskDelay(10);
    }

    // vTaskDelay(1000);
  }
}

void hololink_task(void* arg)
{
    hololink_init(&hololink_client);
    sys_thread_new("Hololink Enumerator Task", holoscan_enumerator_task, &hololink_client, 1024, osPriorityNormal);
    sys_thread_new("Hololink Data Plane Task", holoscan_data_task, &hololink_client, 1024, osPriorityNormal);
    int err;
    struct netbuf *inbuf = NULL;
    uint8_t *inbuf_ptr = NULL;
    uint16_t size_inbuf = 0;
    struct netbuf *outbuf = netbuf_new();
    while ((err = netconn_recv(hololink_client.control, &inbuf)) == ERR_OK)
    {
        printf("Hololink Control packet received from %s, port %d\r\n", ipaddr_ntoa(&inbuf->addr), inbuf->port);
        netbuf_data(inbuf, (void**)&inbuf_ptr, &size_inbuf);
        memcpy(hololink_client.control_recv_buf, (void*)inbuf_ptr, size_inbuf);
        process_packet(&hololink_client, hololink_client.control_recv_buf);


        // uint8_t *outbuf_ptr = netbuf_alloc(outbuf, sizeof(hololink_op_rd_wr_t));
        outbuf->addr = inbuf->addr;
        outbuf->port = inbuf->port;
        hololink_client.host_ip = inbuf->addr;
        // memcpy(outbuf_ptr, hololink_client.control_recv_buf, sizeof(hololink_op_rd_wr_t));
        netbuf_ref(outbuf, hololink_client.control_recv_buf, sizeof(hololink_op_rd_wr_t));
        netconn_send( hololink_client.control, 
                       outbuf);
        netbuf_delete(inbuf);
    }
    netbuf_delete(outbuf);

}


static inline void holoscan_bootp_ntoh(bootp_request_t* packet) {
    packet->transaction_id = ntohl(packet->transaction_id);
    packet->seconds = ntohs(packet->seconds);
    packet->flags = ntohs(packet->flags);
    packet->client_ip_address = ntohl(packet->client_ip_address);
    packet->your_ip_address = ntohl(packet->your_ip_address);
    packet->server_ip_address = ntohl(packet->server_ip_address);
    packet->gateway_ip_address = ntohl(packet->gateway_ip_address);
    packet->vendor_id = ntohl(packet->vendor_id);
}

static inline void holoscan_bootp_hton(bootp_request_t* packet) {
    packet->transaction_id = htonl(packet->transaction_id);
    packet->seconds = htons(packet->seconds);
    packet->flags = htons(packet->flags);
    packet->client_ip_address = htonl(packet->client_ip_address);
    packet->your_ip_address = htonl(packet->your_ip_address);
    packet->server_ip_address = htonl(packet->server_ip_address);
    packet->gateway_ip_address = htonl(packet->gateway_ip_address);
    packet->vendor_id = htonl(packet->vendor_id);
} 