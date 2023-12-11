/*
 * Neraiv_SerialIO.h
 *
 *  Created on: Dec 4, 2023
 *      Author: hakan
 */

#ifndef INC_NERAIV_SERIALIO_H_
#define INC_NERAIV_SERIALIO_H_

#include "main.h"
#include "string.h"

#define PIN_COUNT 			3
#define POT_COUNT 			3


#define buffer_length 		6
#define SWITCH 	11
#define POT		22
#define DATABTN	33

typedef enum{
	SIO_HIGH,
	SIO_LOW,
}SerialIO_StatusTypeDef;

typedef struct{
	uint16_t 	ID;
	SerialIO_StatusTypeDef 	state;
}SerialPin;

// Pot
typedef struct{
	SerialPin 	pin;
	float 		value;
}SerialPot;

typedef struct {
    enum { PIN_TYPE, POT_TYPE } type;
    uint8_t index;
} ElementType;

typedef struct{
	UART_HandleTypeDef 	*huart;
	uint8_t buffer[buffer_length];
#ifdef PIN_COUNT
	uint8_t 			pinsCreated;
	SerialPin*			pins[PIN_COUNT];
#endif
#ifdef POT_COUNT
	uint8_t 			potsCreated;
	SerialPot*			pots[POT_COUNT];
#endif
}SerialIOUart;


void sioInit(SerialIOUart* sio, UART_HandleTypeDef* huart);
void sioHandle(SerialIOUart* sio);

void sioCreatePot(SerialIOUart* sio, SerialPot* pot, uint16_t ID);
void sioCreatePin(SerialIOUart* sio, SerialPin* pin, uint16_t ID);


// Compare function
#endif /* INC_NERAIV_SERIALIO_H_ */
