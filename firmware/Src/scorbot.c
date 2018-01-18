/*
 * server.c
 *
 *  Created on: Jan 17, 2018
 *      Author: jay
 */

#include <scorbot.h>
#include "mqtt.h"
#include "lwip/api.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "motor.h"

typedef enum {
  MQTT_STATE_INIT,
  MQTT_STATE_IDLE,
  MQTT_STATE_DO_CONNECT,
#if MQTT_USE_TLS
  MQTT_STATE_DO_TLS_HANDSHAKE,
#endif
  MQTT_STATE_WAIT_FOR_CONNECTION,
  MQTT_STATE_CONNECTED,
  MQTT_STATE_DO_PUBLISH,
  MQTT_STATE_DO_SUBSCRIBE,
  MQTT_STATE_DO_DISCONNECT
} MQTT_State_t;

mqtt_client_t mqtt_client;
ip4_addr_t broker_ip;
static MQTT_State_t MQTT_state = MQTT_STATE_INIT;

const char *topic = "motors/setpoint/#";

// private methods
static int mqtt_do_connect(mqtt_client_t *client, ip4_addr_t *broker_ipaddr);

static void mqtt_sub_request_cb(void *arg, err_t result) {
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Subscribe result: %d\n", result));
}

static void my_mqtt_subscribe(mqtt_client_t *client, void *arg) {
  err_t err;

  /* Subscribe to a topic named topic with QoS level 1, call mqtt_sub_request_cb with result */
  err = mqtt_subscribe(client, topic, 1, mqtt_sub_request_cb, arg);
  if(err != ERR_OK) {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_subscribe return: %d\n", err));
  }
  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Subscribed to topic \"%s\", res: %d\r\n", topic, (int)err));
}


static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t topic_len, const u8_t *payload, u16_t payload_len) {
  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Incoming publish at topic \"%s\", data: %s\n", topic, payload));
  if(strstr(topic, "setpoint") != NULL)
  {
	  if(strstr(topic, "/6") != NULL)
	  {
		  int newSetpoint = atoi(payload);
		  motor_set_position(6, newSetpoint);
		  printf("New setpoint for motor 6: %d\r\n", newSetpoint);
	  }
  }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
  //err_t err;

  if(status == MQTT_CONNECT_ACCEPTED) {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connection_cb: Successfully connected\n"));

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, arg);

    my_mqtt_subscribe(client, arg);
  } else {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connection_cb: Disconnected, reason: %d\n", status));
    MQTT_state = MQTT_STATE_IDLE;
    /* Its more nice to be connected, so try to reconnect */
//    mqtt_do_connect(client, &broker_ip);
  }
}

static int mqtt_do_connect(mqtt_client_t *client, ip4_addr_t *broker_ipaddr) {
  struct mqtt_connect_client_info_t ci;
  err_t err;

  memset(client, 0, sizeof(mqtt_client_t)); /* initialize all fields */

  /* Setup an empty client info structure */
  memset(&ci, 0, sizeof(ci));
  /* Minimal amount of information required is client identifier, so set it here */
  ci.client_id = CONFIG_CLIENT_ID_NAME;
//  ci.client_user = CONFIG_CLIENT_USER_NAME;
//  ci.client_pass = CONFIG_CLIENT_USER_PASSWORD;
  ci.keep_alive = 60; /* timeout */

  /* Initiate client and connect to server, if this fails immediately an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting
     to establish a connection with the server.
     For now MQTT version 3.1.1 is always used */
#if MQTT_USE_TLS
  client->ssl_context = &ssl;
  err = mqtt_client_connect(client, broker_ipaddr, MQTT_PORT_TLS, mqtt_connection_cb, 0, &ci);
#else
  err = mqtt_client_connect(client, broker_ipaddr, MQTT_PORT, mqtt_connection_cb, 0, &ci);
#endif
  /* For now just print the result code if something goes wrong */
  if(err != ERR_OK) {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connect return %d\n", err));
    return -1; /* error */
  }
  return 0; /* ok */
}

static void MqttDoStateMachine(mqtt_client_t *mqtt_client, ip4_addr_t *broker_ipaddr) {

  switch(MQTT_state) {
    case MQTT_STATE_INIT:
    case MQTT_STATE_IDLE:
    	MQTT_state = MQTT_STATE_DO_CONNECT;
      break;
    case MQTT_STATE_DO_CONNECT:
      #if MQTT_USE_TLS
        if (TLS_Init()!=0) { /* failed? */
          LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("ERROR: failed to initialize for TLS!\r\n"));
          for(;;) {} /* stay here in case of error */
        }
      #endif
      LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE, ("Connecting to broker\r\n"));
      if (mqtt_do_connect(mqtt_client, broker_ipaddr)==0) {
#if MQTT_USE_TLS
        MQTT_state = MQTT_STATE_DO_TLS_HANDSHAKE;
#else
        MQTT_state = MQTT_STATE_WAIT_FOR_CONNECTION;
#endif
      } else {
        LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Failed to connect to broker\r\n"));
      }
      break;
#if MQTT_USE_TLS
    case MQTT_STATE_DO_TLS_HANDSHAKE:
      if (mqtt_do_tls_handshake(mqtt_client)==0) {
        LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("TLS handshake completed\r\n"));
        mqtt_start_mqtt(mqtt_client);
        MQTT_state = MQTT_STATE_WAIT_FOR_CONNECTION;
      }
      break;
#endif
    case MQTT_STATE_WAIT_FOR_CONNECTION:
      if (mqtt_client_is_connected(mqtt_client)) {
        LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Client is connected\r\n"));
        MQTT_state = MQTT_STATE_CONNECTED;
      } else {
#if MQTT_USE_TLS
        mqtt_recv_from_tls(mqtt_client);
#endif
      }
      break;
    case MQTT_STATE_CONNECTED:
      if (!mqtt_client_is_connected(mqtt_client)) {
        LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Client got disconnected?!?\r\n"));
        MQTT_state = MQTT_STATE_DO_CONNECT;
      }
#if MQTT_USE_TLS
      else {
        mqtt_tls_output_send(mqtt_client); /* send (if any) */
        mqtt_recv_from_tls(mqtt_client); /* poll if we have incoming packets */
      }
#endif
      break;
    case MQTT_STATE_DO_SUBSCRIBE:
      LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Subscribe from broker\r\n"));
      my_mqtt_subscribe(mqtt_client, NULL);
      MQTT_state = MQTT_STATE_CONNECTED;
      break;
#if 0
    case MQTT_STATE_DO_PUBLISH:
      LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Publish to broker\r\n"));
      my_mqtt_publish(mqtt_client, NULL);
      MQTT_state = MQTT_STATE_CONNECTED;
      break;
#endif
    case MQTT_STATE_DO_DISCONNECT:
      LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Disconnect from broker\r\n"));
      mqtt_disconnect(mqtt_client);
      MQTT_state = MQTT_STATE_IDLE;
      break;
	default:
	  break;
  }
}

void Server_MainTask()
{
	IP4_ADDR(&broker_ip, 192, 168, 0, 1);
	MQTT_state = MQTT_STATE_DO_CONNECT;
	for(;;) {
		MqttDoStateMachine(&mqtt_client, &broker_ip);
		vTaskDelay(1000);
	}
}
