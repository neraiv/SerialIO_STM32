/*
 * Neraiv_SerialIO.c
 *
 *  Created on: Dec 4, 2023
 *      Author: hakan
 */
#include "Neraiv_SerialIO.h"

void sioInit(SerialIOUart* sio, UART_HandleTypeDef* huart){
	sio->huart = huart;
#ifdef PIN_COUNT
	sio->pinsCreated = 0;
#endif
#ifdef POT_COUNT
	sio->potsCreated = 0;
#endif

	HAL_UART_Receive_IT(sio->huart, sio->buffer, buffer_length);
}

#ifdef POT_COUNT
void sioCreatePot(SerialIOUart* sio, SerialPot* pot, uint16_t ID){
	pot->pin.ID = ID;
	if(sio->potsCreated < (POT_COUNT)){
		sio->pots[sio->potsCreated] = pot;
		sio->potsCreated++;
	}else{
		Error_Handler();
	}
}
#endif

#ifdef PIN_COUNT
void sioCreatePin(SerialIOUart* sio, SerialPin* pin, uint16_t ID){
	pin->ID = ID;
	if(sio->potsCreated < (PIN_COUNT)){
		sio->pins[sio->pinsCreated] = pin;
		sio->pinsCreated++;
	}else{
		Error_Handler();
	}
}
#endif

// Function to search for ID in the arrays and return the element type
ElementType getElementByID(SerialIOUart* uartStruct, int targetID) {
    ElementType element;

    element.index = 0;

    for (int i = 0; i < uartStruct->pinsCreated; ++i) {
        if ((uartStruct->pins[i]->ID) == targetID) {
            element.type = PIN_TYPE;
            element.index = i;
            return element;
        }
    }

    for (int i = 0; i < uartStruct->potsCreated; ++i) {
        if ((uartStruct->pots[i]->pin.ID) == targetID) {
            element.type = POT_TYPE;
            element.index = i;
            return element;
        }
    }

    element.type = -1; // Indicating element not found
    return element;
}

void sioHandle(SerialIOUart* sio){
	uint8_t sioPinCount = 0;

#ifdef PIN_COUNT
	sioPinCount += PIN_COUNT;
#endif

#ifdef POT_COUNT
	sioPinCount += POT_COUNT;
#endif

	if(sioPinCount == 0){
		Error_Handler();
	}

	uint16_t ID;

	ID = sio->buffer[1] | sio->buffer[0];


	// Perform the search and get the element type
	ElementType foundElement = getElementByID(sio, ID);

	// Output the result
	if (foundElement.type == PIN_TYPE) { // Process pinElement
		if(sio->buffer[2] == 0){

			sio->pins[foundElement.index]->state = SIO_LOW;
		}else if(sio->buffer[2] == 1){
			sio->pins[foundElement.index]->state = SIO_HIGH;
		}

	} else if (foundElement.type == POT_TYPE) { // Process potElement
		sio->pots[foundElement.index]->pin.state = SIO_HIGH;

		union {
			uint8_t uint_bytes[4];
			float float_val;
		} data;

		// Assuming bytes[0] is the least significant byte (LSB) and bytes[3] is the most significant byte (MSB)
		data.uint_bytes[0] = sio->buffer[2];
		data.uint_bytes[1] = sio->buffer[3];
		data.uint_bytes[2] = sio->buffer[4];
		data.uint_bytes[3] = sio->buffer[5];

		sio->pots[foundElement.index]->value = data.float_val;

	} else {
		Error_Handler();
	}
	HAL_UART_Receive_IT(sio->huart, sio->buffer, buffer_length);
}
