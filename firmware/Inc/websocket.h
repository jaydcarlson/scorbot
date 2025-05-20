// Lightly modified from example code at https://github.com/maxushka/web_socket_stm32f4

#ifndef __WEBSOCKET_H
#define __WEBSOCKET_H

#include "lwip/api.h"
#include "cmsis_os.h"

#define WS_USE_SDRAM                 0
#define WS_PORT                      8080
#define WS_MAX_CLIENTS               3
#define WS_SEND_BUFFER_SIZE          1024
#define WS_MSG_BUFFER_SIZE           512
#define WS_CLIENT_RECV_BUFFER_SIZE   1024

#define WS_GUID     "258EAFA5-E914-47DA-95CA-C5AB0DC85B11\0"

#define WS_SEND_BUFFER_START_ADDR   (0xC0000000)
#define WS_SEND_BUFFER_END_ADDR     (WS_SEND_BUFFER_START_ADDR + WS_SEND_BUFFER_SIZE)

typedef enum
{
  WS_TYPE_STRING = 0x81,
  WS_TYPE_BINARY = 0x82
} ws_type_t;

typedef struct
{
  uint8_t *protocol;
  uint32_t prtcl_size;
  uint8_t *message;
  uint32_t msg_size;
  ws_type_t msg_type;
} ws_msg_t;

typedef struct
{
  TaskHandle_t task_handle;
  struct netconn *accepted_sock;
  void *server_ptr;
  uint8_t recv_buf[WS_CLIENT_RECV_BUFFER_SIZE];
  uint32_t established;
  uint32_t ws_connected;
  uint32_t id;
} ws_client_t;

typedef struct
{
  ws_client_t ws_clients[WS_MAX_CLIENTS];
#if WS_USE_SDRAM == 1
  uint8_t *send_buf;
#else
  uint8_t send_buf[WS_SEND_BUFFER_SIZE];
#endif
  void (*msg_handler)( ws_client_t* client, uint8_t *data, uint32_t len, ws_type_t type );
  uint32_t connected_clients_cnt;
} ws_server_t;


void ws_server_task( void * arg );

/**
 * @brief Send a message to a client, or all clients
 * 
 * @param ws the server instance
 * @param msg the message to send
 * @param client the client to send the message to, or NULL to send to all clients
 */
void ws_send_message( ws_server_t *ws, ws_msg_t *msg, ws_client_t* client);


#endif

/***************************** END OF FILE ************************************/
