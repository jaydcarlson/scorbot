#include "hololink.h"
#include "lwip/api.h"
#include "lwip/udp.h"
#include <string.h>


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

static inline void hololink_packet_hton(hololink_packet_t* packet)
{
    packet->pkey = htons(packet->pkey);
    packet->qp_becn = htonl(packet->qp_becn);
    packet->psn_ack = htonl(packet->psn_ack);
    packet->address = htonll(packet->address);
    packet->rkey = htonl(packet->rkey);
    packet->size = htonl(packet->size);
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
    .hardware_address = {0},
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
    printf("Requested read of word at address 0x%04x\n", op->address);
    switch(op->address)
    {
        case REG_FPGA_VERSION:
            printf("REG_FPGA_VERSION\n");
            op->data = 0x00000001;
            break;
        case REG_FPGA_DATE:
            printf("REG_FPGA_DATE\n");
            op->data = 0x00000002;
            break;
        default:
            // check to see if address is in the range of (REG_CONFIG_BASE + REG_DP_PACKET_SIZE) and (REG_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t))
            if(op->address >= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE) && op->address <= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t)))
            {
                printf("DP register address 0x%04x\n", op->address);
                uint32_t offset = (op->address - (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE)) + (uint32_t)&hololink_dp_packet_config;
                op->data = *(uint32_t*)offset;
            } else 
            // check to see if address is in the range of (REG_CONFIG_BASE + REG_DP_QP) and (REG_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t))
            if(op->address >= (REG_SENSOR_CONFIG_BASE + REG_DP_QP) && op->address <= (REG_SENSOR_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t)))
            {
                printf("DP Config register address 0x%04x\n", op->address);
                uint32_t offset = (op->address - (REG_SENSOR_CONFIG_BASE + REG_DP_QP)) + (uint32_t)&hololink_dp_config;
                op->data = *(uint32_t*)offset;  
            } else {
                printf("Unknown register address 0x%04x\n", op->address);
                return;
            }
    }
    printf("Returning data 0x%04x\n", op->data);
}

void update_from_config(hololink_client_t* client)
{
    if(hololink_dp_packet_config.vip_mask == 1 && client->streaming_enabled == false)
    {
        printf("\n\n\nStart streaming to port %d\n\n\n", hololink_dp_config.host_udp_port);
        client->host_port = hololink_dp_config.host_udp_port;
        client->streaming_enabled = true;
    } else if(hololink_dp_packet_config.vip_mask == 0 && client->streaming_enabled == true) {
        printf("\n\n\nStop streaming\n\n\n");
        client->streaming_enabled = false;
    }
    
}

void write_word_cmd(hololink_client_t* client, uint8_t* cmd)
{
    hololink_op_rd_wr_t* op = (hololink_op_rd_wr_t*)cmd;
    printf("Requested write of word at address 0x%04x\n", op->address);
    printf("Data: 0x%04x\n", op->data);
    switch(op->address)
    {
        default:
            // check to see if address is in the range of (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE) and (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t))
            if(op->address >= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE) && op->address <= (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE + sizeof(hololink_dp_packet_config_t)))
            {
                printf("DP register address 0x%04x\n", op->address);
                uint32_t offset = (op->address - (REG_NETWORK_CONFIG_BASE + REG_DP_PACKET_SIZE)) + (uint32_t)&hololink_dp_packet_config;
                *(uint32_t*)offset = op->data;
            } else 
            // check to see if address is in the range of (REG_SENSOR_CONFIG_BASE + REG_DP_QP) and (REG_SENSOR_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t))
            if(op->address >= (REG_SENSOR_CONFIG_BASE + REG_DP_QP) && op->address <= (REG_SENSOR_CONFIG_BASE + REG_DP_QP + sizeof(hololink_dp_config_t)))
            {
                printf("DP Config register address 0x%04x\n", op->address);
                uint32_t offset = (op->address - (REG_SENSOR_CONFIG_BASE + REG_DP_QP)) + (uint32_t)&hololink_dp_config;
                *(uint32_t*)offset = op->data;  
            } else {
                printf("Unknown register address 0x%04x\n", op->address);
                return;
            }
    }

    update_from_config(client);
}

void process_packet(hololink_client_t* client, uint8_t* packet)
{
    hololink_op_rd_wr_t* op = (hololink_op_rd_wr_t*)packet;
    hololink_op_rd_wr_ntoh(op);
    // printf("Received command 0x%02x\n, sequence 0x%02x\n, address 0x%04x\n, data 0x%04x\n", op->command, op->sequence, op->address, op->data);
    switch(op->command)
    {
        case CMD_RD_DWORD:
            read_word_cmd(client, packet);
            break;
        case CMD_WR_DWORD:
            write_word_cmd(client, packet);
            break;
        default:
            printf("Unknown command 0x%02x\n", op->command);
            break;
    }
    // printf("Sending command 0x%02x\n, sequence 0x%04x\n, address 0x%04x\n, data 0x%04x\n", op->command, op->sequence, op->address, op->data);
    hololink_op_rd_wr_hton(op);
}

void hololink_init(hololink_client_t* client)
{
    // create a UDP socket using netcon
    client->enumerator = netconn_new(NETCONN_UDP);
    if(client->enumerator == NULL)
    {
        printf("Failed to create hololink enumerator\n");
        return;
    }

    if(netconn_bind(client->enumerator, NULL, HOLOLINK_BOOTP_REQUEST_PORT) != ERR_OK)
    {
        printf("Failed to bind hololink enumerator\n");
        return;
    }

    client->control = netconn_new(NETCONN_UDP);
    if(client->control == NULL)
    {
        printf("Failed to create hololink control\n");
        return;
    }

    if(netconn_bind(client->control, NULL, HOLOLINK_CONTROL_PORT) != ERR_OK)
    {
        printf("Failed to bind hololink control\n");
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
 
  hololink_init(hololink_client);
  while(1)
  {
    // printf("Holoscan Enumerator\n");
    hololink_send_enumeration_packet(hololink_client);
    vTaskDelay(1000);
  }
}

void holoscan_data_task(void* arg)
{
  hololink_client_t* hololink_client = (hololink_client_t* )arg;
  uint32_t counter = 0;
  while(1)
  {
    if(hololink_client->streaming_enabled)
    {
        // hololink_client->packet.op_code = 0x2A; // IBV_OPCODE_UC_RDMA_WRITE_ONLY
        hololink_client->packet.op_code = 0x2B; // IBV_OPCODE_UC_RDMA_WRITE_ONLY_WITH_IMMEDIATE
        hololink_client->packet.pkey = 0xffff;
        hololink_client->packet.qp_becn = hololink_dp_config.qp;
        hololink_client->packet.rkey = hololink_dp_config.rkey;
        hololink_client->packet.size = 100;
        hololink_client->packet.payload[0] = counter & 0xff;
        hololink_client->packet.payload[1] = (counter >> 8) & 0xff;
        hololink_client->packet.payload[2] = (counter >> 16) & 0xff;
        hololink_client->packet.payload[3] = (counter >> 24) & 0xff;
        hololink_packet_hton(&hololink_client->packet);
        struct netbuf* buffer = netbuf_new();
        netbuf_ref(buffer, &hololink_client->packet, sizeof(hololink_packet_t));
        netconn_sendto(hololink_client->control, buffer, &hololink_client->host_ip, hololink_client->host_port);
        netbuf_delete(buffer);
        counter++;
        vTaskDelay(1);
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
        printf("Hololink Control packet received from %s, port %d\n", ipaddr_ntoa(&inbuf->addr), inbuf->port);
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