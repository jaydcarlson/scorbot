/*
 * softwareEncoder.h
 *
 *  Created on: Jan 13, 2018
 *      Author: jay
 */

#ifndef SOFTWAREENCODER_H_
#define SOFTWAREENCODER_H_

#include <stdint.h>

extern int32_t motor1_encoder, motor6_encoder, motor8_encoder;

#define ENC_A_M1_INT	GPIO_PIN_10
#define	ENC_B_M1_INT	GPIO_PIN_9

#define ENC_A_M6_INT	GPIO_PIN_8

#define ENC_A_M8_INT	GPIO_PIN_13
#define ENC_B_M8_INT	GPIO_PIN_14

#endif /* SOFTWAREENCODER_H_ */
