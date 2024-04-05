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
#define SIGNAL_COUNT 			1


#define buffer_length 		7
#define SWITCH 	11
#define POT		22
#define PWM		33

typedef struct _SerialIOUart SerialIOUart;

typedef enum{
	SIO_ERR_MAX_NUMBER_OF_PIN_EXCEED,
	SIO_ERR_NO_PIN_TO_EXECUTE,
	SIO_ERR_UNRECOGNIZED_TYPE,
	SIO_ERR_CANT_SET_TIM				// U may choose different timer 
}SerialIO_ErrorTypeDef;

typedef enum{
	SIO_SIGNAL_AMPLITUDE_MAX
}SerialIO_SignalPreferenceTypeDef;

typedef enum{
	SIO_SIGNAL_SINE,
	SIO_SIGNAL_COSINE,
	SIO_SIGNAL_PWM,
	SIO_SIGNAL_SAWTOOTH,
}SerialIO_SignalTypeDef;

typedef enum{
	SIO_HIGH,
	SIO_LOW,
}SerialIO_StatusTypeDef;

typedef union{
	struct{
		float frequancy;
		float amplitude;
	} sine;

	struct{
		float frequancy;
		float amplitude;
	} cosine;

	struct{
		float frequancy;
		float amplitude;
		float duty_cyclce;
	} pwm;

	struct{
		float frequancy;
		float amplitude;
	} sawtooth;

}SerialIO_Signal;

typedef struct{
	uint16_t 	ID;
	SerialIO_StatusTypeDef 	state;
	GPIO_TypeDef* port;
	uint16_t pin;
	int (*functionPtr)(SerialIOUart);
}SerialPin;

// Pot
typedef struct{
	SerialPin 	pin;
	float 		value;
}SerialPot;

// Pot
typedef struct{	
	SerialIO_Signal signal;
	SerialIO_SignalTypeDef type;
}SerialSignal;

typedef struct{	
	TIM_HandleTypeDef* htim;
	uint32_t timerCnt;
}SerialSignal_Timer;


typedef struct {
    enum { PIN_TYPE, POT_TYPE } type;
    uint8_t index;
} ElementType;

struct _SerialIOUart{
	UART_HandleTypeDef 	*huart;
	uint8_t buffer[buffer_length];
#ifdef PIN_COUNT
	uint8_t 			pinsCreated;
	SerialPin			pins[PIN_COUNT];
#endif
#ifdef POT_COUNT
	uint8_t 			potsCreated;
	SerialPot			pots[POT_COUNT];
#endif
#ifdef SIGNAL_COUNT
	uint8_t 			signalsCreated;	
	uint8_t 			timerCreated;
	SerialSignal_Timer  signal_timer[SIGNAL_COUNT];
	SerialSignal		signals[SIGNAL_COUNT];
#endif
};


void sioErrorHandler(SerialIO_ErrorTypeDef err);

void sioInit(SerialIOUart* _sio, UART_HandleTypeDef* huart);
void sioHandle();

void sioCreatePot	(uint16_t ID, GPIO_TypeDef* gpio_port, uint16_t pin_number);
void sioCreatePin	(uint16_t ID, GPIO_TypeDef* gpio_port, uint16_t pin_number);
void sioCreateSignal(uint16_t ID, GPIO_TypeDef* gpio_port, uint16_t pin_number, SerialSignal* signal,TIM_HandleTypeDef* htim, IRQn_Type timer_irq);

void calcCRC();


// Compare function
#endif /* INC_NERAIV_SERIALIO_H_ */
