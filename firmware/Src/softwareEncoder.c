/*
 * softwareEncoder.c
 *
 *  Created on: Jan 13, 2018
 *      Author: jay
 */

#include "softwareEncoder.h"
#include "main.h"

int16_t motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder, motor5_encoder, motor6_encoder;

/*
 * Encoder pin activity monitor.
 *
 * Sampling the GPIO input registers over SWD from the host is far too slow: a
 * hand-turned encoder produces hundreds of edges a second and the debugger
 * manages a few tens of samples a second, so a channel that is working can
 * easily look dead. Counting transitions here instead, inside the 5 kHz control
 * loop, is fast enough to catch every edge and needs no assumption about
 * whether the pin's EXTI is configured or even pointed at the right port.
 *
 * Purely diagnostic; nothing in the control path reads these.
 */
/*
 * Every pin of GPIOA..GPIOE is watched rather than just the twelve the firmware
 * believes are encoders. Restricting the search to the assumed pinout can only
 * ever confirm the assumption: if a signal has been bodged to a different pin,
 * the narrow version reports "dead" and the wide version shows you where it
 * actually went. Index is port * 16 + pin.
 */
volatile uint32_t encoder_pin_edges[ENCODER_DEBUG_CHANNELS];
volatile uint32_t encoder_debug_samples;
static uint16_t encoder_last_levels[ENCODER_DEBUG_PORTS];

void encoder_debug_sample(void)
{
	const uint16_t now[ENCODER_DEBUG_PORTS] = {
		(uint16_t)GPIOA->IDR,
		(uint16_t)GPIOB->IDR,
		(uint16_t)GPIOC->IDR,
		(uint16_t)GPIOD->IDR,
		(uint16_t)GPIOE->IDR,
	};

	++encoder_debug_samples;

	for(int port = 0; port < ENCODER_DEBUG_PORTS; port++) {
		uint16_t changed = (uint16_t)(now[port] ^ encoder_last_levels[port]);
		encoder_last_levels[port] = now[port];
		while(changed != 0u) {
			/* Only iterate over bits that actually moved, which is almost
			 * always none, so the common case costs one compare per port. */
			const int pin = __builtin_ctz(changed);
			changed &= (uint16_t)(changed - 1u);
			encoder_pin_edges[port * 16 + pin]++;
		}
	}
}

void encoder_debug_reset(void)
{
	for(int i = 0; i < ENCODER_DEBUG_CHANNELS; i++) {
		encoder_pin_edges[i] = 0;
	}
	encoder_debug_samples = 0;
}
