/*
 * server.c
 *
 *  Created on: Jan 17, 2018
 *      Author: jay
 */

#include <scorbot.h>
// #include "mqtt_opts.h"
// #include "lwip/apps/mqtt.h"
#include "lwip/api.h"
#include "lwip/netif.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "joint.h"

/* websocket.h: legacy transport, no longer part of the control path */
#include "hololink.h"
#include "homing.h"
#include "scorbot_ctrl.h"
#include "scorbot_udp.h"

/* The pre-UDP transports still build, but are off unless explicitly enabled. */
#ifndef SCORBOT_ENABLE_LEGACY_TRANSPORTS
#define SCORBOT_ENABLE_LEGACY_TRANSPORTS 0
#endif

extern struct netif gnetif;

void Scorbot_MainTask()
{
  printf("scorbot: init joints\r\n");
  joint_init();
  printf("scorbot: init homing\r\n");
  homing_init();
  printf("scorbot: init data plane\r\n");
  scorbot_udp_init();
  printf("scorbot: init control plane\r\n");
  scorbot_ctrl_init();

  /*
   * All three run at osPriorityNormal, the same priority as the LwIP TCP/IP
   * thread (TCPIP_THREAD_PRIO in lwipopts.h).
   *
   * Putting them above it is tempting for latency but is a trap: these tasks
   * are fed by the TCP/IP thread, so any stall or spin in them starves the very
   * thread that would deliver their next packet, and the DHCP timers along with
   * it. The Ethernet receive thread sits at osPriorityRealtime and still
   * preempts everything, so incoming frames are never delayed by this.
   */
  sys_thread_t rx = sys_thread_new("ScorbotRx", (lwip_thread_fn)scorbot_udp_rx_task, NULL, 512,
                                   osPriorityNormal);
  sys_thread_t tx = sys_thread_new("ScorbotTx", (lwip_thread_fn)scorbot_udp_tx_task, NULL, 512,
                                   osPriorityNormal);
  sys_thread_t ct = sys_thread_new("ScorbotCtl", (lwip_thread_fn)scorbot_ctrl_task, NULL, 512,
                                   osPriorityNormal);
  printf("scorbot: free heap %u\r\n", (unsigned)xPortGetFreeHeapSize());

#if SCORBOT_ENABLE_LEGACY_TRANSPORTS
  /* Hololink predates the UDP protocol and is kept so the Holoscan sensor
   * bridge tooling still has something to talk to. */
  sys_thread_new("Hololink", hololink_task, NULL, 1024, osPriorityNormal);
#endif

  for(;;)
  {
    const uint32_t ip = gnetif.ip_addr.addr;
    printf("scorbot: netif flags=0x%02x ip=%u.%u.%u.%u heap=%u\r\n",
           gnetif.flags,
           (unsigned)(ip & 0xFF), (unsigned)((ip >> 8) & 0xFF),
           (unsigned)((ip >> 16) & 0xFF), (unsigned)((ip >> 24) & 0xFF),
           (unsigned)xPortGetFreeHeapSize());
    vTaskDelay(2000);
  }
}
	// xTaskCreate(update_positions, "update_positions", 1024, NULL, 0, NULL);

	// IP4_ADDR(&broker_ip, 192, 168, 0, 10);
	// MQTT_state = MQTT_STATE_DO_CONNECT;
	// mqtt_client = mqtt_client_new();
	// for(;;) {
	// 	MqttDoStateMachine(mqtt_client, &broker_ip);
	// 	vTaskDelay(1000);
	// }
// }



// typedef enum {
//   MQTT_STATE_INIT,
//   MQTT_STATE_IDLE,
//   MQTT_STATE_DO_CONNECT,
//   MQTT_STATE_WAIT_FOR_CONNECTION,
//   MQTT_STATE_CONNECTED,
//   MQTT_STATE_DO_PUBLISH,
//   MQTT_STATE_DO_SUBSCRIBE,
//   MQTT_STATE_DO_DISCONNECT
// } MQTT_State_t;

// mqtt_client_t* mqtt_client;
// ip4_addr_t broker_ip;
// static MQTT_State_t MQTT_state = MQTT_STATE_INIT;

// private methods
// static int mqtt_do_connect(mqtt_client_t *client, ip4_addr_t *broker_ipaddr);

// static void mqtt_sub_request_cb(void *arg, err_t result) {
//   /* Just print the result code here for simplicity,
//      normal behaviour would be to take some action if subscribe fails like
//      notifying user, retry subscribe or disconnect from server */
//   LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Subscribe result: %d\n", result));
// }

// static void my_mqtt_subscribe(mqtt_client_t *client, void *arg) {
//   err_t err;

//   /* Subscribe to a topic named topic with QoS level 1, call mqtt_sub_request_cb with result */
//   err = mqtt_subscribe(client, "motors/setpoint/#", 1, mqtt_sub_request_cb, arg);
//   if(err != ERR_OK) {
//     LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_subscribe return: %d\n", err));
//   }
//   err = mqtt_subscribe(client, "motors/home/#", 1, mqtt_sub_request_cb, arg);
//   if(err != ERR_OK) {
//     LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_subscribe return: %d\n", err));
//   }
// //  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Subscribed to topic \"%s\", res: %d\r\n", topic, (int)err));
// }

// static char* nextChar(const char* message, char* match)
// {
// 	char* found = strstr(message, match);
// 	if(found == NULL)
// 		return NULL;

// 	return found+strlen(match);
// }

// Buffer to store incoming payload
// static char payload_buffer[256];
// static int payload_index = 0;

// static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
//     // Copy the incoming data to our buffer
//     if (payload_index + len < sizeof(payload_buffer)) {
//         memcpy(payload_buffer + payload_index, data, len);
//         payload_index += len;
//     }
    
//     // If this is the last fragment, process the complete payload
//     if (flags & MQTT_DATA_FLAG_LAST) {
//         payload_buffer[payload_index] = '\0'; // Null terminate the string
//         payload_index = 0; // Reset for next message
//     }
// }
// static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
//     // check for motors
//     char* ptr;
//     if((ptr = nextChar(topic, "motors/")) != NULL)
//     {
//         if((ptr = nextChar(topic, "setpoint/")) != NULL)
//         {
//             int motor = atoi(ptr);
//             int newSetpoint = atoi((const char*)payload_buffer);
//             printf("Setting joint angle on joint %d to %d\r\n", motor, newSetpoint);
//             joint_set_angle(&joints[motor], newSetpoint);
//         }
//         else if((ptr = nextChar(topic, "home/")) != NULL)
//         {
//             int motor = atoi(ptr);
//             printf("Homing motor %d\r\n", motor);
//             joint_home(&joints[motor]);
//         }
//     }
// }

// static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
//     if(status == MQTT_CONNECT_ACCEPTED) {
//         LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connection_cb: Successfully connected\n"));

//         /* Setup callback for incoming publish requests */
//         mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

//         my_mqtt_subscribe(client, arg);
//     } else {
//         LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connection_cb: Disconnected, reason: %d\n", status));
//         MQTT_state = MQTT_STATE_IDLE;
//     }
// }

// static int mqtt_do_connect(mqtt_client_t *client, ip4_addr_t *broker_ipaddr) {
//   struct mqtt_connect_client_info_t ci;
//   err_t err;

//   // memset(client, 0, sizeof(mqtt_client_t)); /* initialize all fields */

//   /* Setup an empty client info structure */
//   memset(&ci, 0, sizeof(ci));
//   /* Minimal amount of information required is client identifier, so set it here */
//   ci.client_id = CONFIG_CLIENT_ID_NAME;
// //  ci.client_user = CONFIG_CLIENT_USER_NAME;
// //  ci.client_pass = CONFIG_CLIENT_USER_PASSWORD;
//   ci.keep_alive = 60; /* timeout */

//   /* Initiate client and connect to server, if this fails immediately an error code is returned
//      otherwise mqtt_connection_cb will be called with connection result after attempting
//      to establish a connection with the server.
//      For now MQTT version 3.1.1 is always used */

//   err = mqtt_client_connect(client, broker_ipaddr, MQTT_PORT, mqtt_connection_cb, 0, &ci);
//   /* For now just print the result code if something goes wrong */
//   if(err != ERR_OK) {
//     LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connect return %d\n", err));
//     return -1; /* error */
//   }
//   return 0; /* ok */
// }

// static void MqttDoStateMachine(mqtt_client_t *mqtt_client, ip4_addr_t *broker_ipaddr) {

//   switch(MQTT_state) {
//     case MQTT_STATE_INIT:
//     case MQTT_STATE_IDLE:
//     	MQTT_state = MQTT_STATE_DO_CONNECT;
//       break;
//     case MQTT_STATE_DO_CONNECT:
//       LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE, ("Connecting to broker\r\n"));
//       if (mqtt_do_connect(mqtt_client, broker_ipaddr)==0) {
//         MQTT_state = MQTT_STATE_WAIT_FOR_CONNECTION;
//       } else {
//         LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Failed to connect to broker\r\n"));
//       }
//       break;
//     case MQTT_STATE_WAIT_FOR_CONNECTION:
//       if (mqtt_client_is_connected(mqtt_client)) {
//         LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Client is connected\r\n"));
//         MQTT_state = MQTT_STATE_CONNECTED;
//       } else {
//       }
//       break;
//     case MQTT_STATE_CONNECTED:
//       if (!mqtt_client_is_connected(mqtt_client)) {
//         LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Client got disconnected?!?\r\n"));
//         MQTT_state = MQTT_STATE_DO_CONNECT;
//       }
//       break;
//     case MQTT_STATE_DO_SUBSCRIBE:
//       LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Subscribe from broker\r\n"));
//       my_mqtt_subscribe(mqtt_client, NULL);
//       MQTT_state = MQTT_STATE_CONNECTED;
//       break;

//     case MQTT_STATE_DO_DISCONNECT:
//       LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Disconnect from broker\r\n"));
//       mqtt_disconnect(mqtt_client);
//       MQTT_state = MQTT_STATE_IDLE;
//       break;
// 	default:
// 	  break;
//   }
// }

// char position_topic[50];
// char position_payload[8];

// void position_cb(void *arg, err_t err)
// {
// 	if(err != ERR_OK)
// 	{
// 		printf("%d\r\n", err);
// 	}
// }

// void update_positions()
// {
// 	for(;;) {
// 			if(MQTT_state == MQTT_STATE_CONNECTED)
// 			{
// 				for(int i = 0;i < 7; i++)
// 				{
// 					sprintf(position_topic, "motors/position/%d", i);
// 					sprintf(position_payload, "%d", motor_get_current_position(&motors[i]));
//           // printf("Publishing position %s to %s\r\n", position_payload, position_topic);
// 					mqtt_publish(mqtt_client, position_topic, position_payload, strlen(position_payload), 0, 0, position_cb, NULL);
// 				}

// 			}
// 			vTaskDelay(100);
// 		}
// }

