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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "joint.h"

#include "websocket.h"

void msg_handler( ws_client_t* client, uint8_t *data, uint32_t len, ws_type_t type );

ws_server_t ws_server = {
  .connected_clients_cnt = 0,
  .msg_handler = msg_handler
};

void msg_handler( ws_client_t* client, uint8_t *data, uint32_t len, ws_type_t type )
{
  printf("got message from client %d, type %s, %s\n", client->id, type == WS_TYPE_STRING ? "string" : "binary", data);

  // execute the commands
  scorbot_cmd_t* cmd = (scorbot_cmd_t*)data;
  joint_execute_cmd(&joints[0], &cmd->shoulder_pan);
  joint_execute_cmd(&joints[1], &cmd->shoulder_lift);
  joint_execute_cmd(&joints[2], &cmd->elbow);
  joint_execute_cmd(&joints[3], &cmd->wrist_1);
  joint_execute_cmd(&joints[4], &cmd->wrist_2);
  joint_execute_cmd(&joints[5], &cmd->gripper);

  // scorbot_status_t status;
  // joint_get_status(&joints[0], &status.shoulder_pan);
  // joint_get_status(&joints[1], &status.shoulder_lift);
  // joint_get_status(&joints[2], &status.elbow);
  // joint_get_status(&joints[3], &status.wrist_1);
  // joint_get_status(&joints[4], &status.wrist_2);
  // joint_get_status(&joints[5], &status.gripper);

  // ws_msg_t msg = {
  //   .msg_type = WS_TYPE_BINARY,
  //   .message = (uint8_t*)&status,
  //   .msg_size = sizeof(scorbot_status_t)
  // };

  // ws_send_message(&ws_server, &msg, client);
}

void broadcast_status()
{
  scorbot_status_t status;
  joint_get_status(&joints[0], &status.shoulder_pan);
  joint_get_status(&joints[1], &status.shoulder_lift);
  joint_get_status(&joints[2], &status.elbow);
  joint_get_status(&joints[3], &status.wrist_1);
  joint_get_status(&joints[4], &status.wrist_2);
  joint_get_status(&joints[5], &status.gripper);

  ws_msg_t msg = {
    .msg_type = WS_TYPE_BINARY,
    .message = (uint8_t*)&status,
    .msg_size = sizeof(scorbot_status_t)
  };

  ws_send_message(&ws_server, &msg, NULL);
}

void Scorbot_MainTask()
{
  sys_thread_new("WS", ws_server_task, (void*)&ws_server, 1024, osPriorityNormal);
  int i = 0;
  for(;;)
  {
    printf("Broadcasting status %d\n", i++);
    broadcast_status();
    vTaskDelay(100);
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

