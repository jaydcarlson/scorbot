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

#endif /* SOFTWAREENCODER_H_ */
