/*
 * softwareEncoder.h
 *
 *  Created on: Jan 13, 2018
 *      Author: jay
 */

#ifndef SOFTWAREENCODER_H_
#define SOFTWAREENCODER_H_

#include <stdint.h>

extern int16_t motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder, motor5_encoder, motor6_encoder;

#define ENC_A_M1_INT	GPIO_PIN_10
#define	ENC_B_M1_INT	GPIO_PIN_9

#define ENC_A_M2_INT	GPIO_PIN_15

#define ENC_A_M3_INT	GPIO_PIN_12

#define ENC_A_M4_INT	GPIO_PIN_4

#define ENC_A_M5_INT	GPIO_PIN_13

#define ENC_A_M6_INT	GPIO_PIN_8

//#define ENC_A_M7_INT	GPIO_PIN_10

//#define ENC_A_M8_INT	GPIO_PIN_13
//#define ENC_B_M8_INT	GPIO_PIN_14

/*
 * Diagnostic edge counters for every pin of GPIOA..GPIOE, sampled in the 5 kHz
 * control loop. Read them over SWD to find out which lines are physically
 * alive, independent of whether a pin's EXTI is configured, or configured for
 * the right port, or whether the signal is even on the pin we expect.
 *
 * Index is port * 16 + pin, with port 0 = GPIOA.
 */
#define ENCODER_DEBUG_PORTS 5
#define ENCODER_DEBUG_CHANNELS (ENCODER_DEBUG_PORTS * 16)

extern volatile uint32_t encoder_pin_edges[ENCODER_DEBUG_CHANNELS];
extern volatile uint32_t encoder_debug_samples;

void encoder_debug_sample(void);
void encoder_debug_reset(void);

#endif /* SOFTWAREENCODER_H_ */
