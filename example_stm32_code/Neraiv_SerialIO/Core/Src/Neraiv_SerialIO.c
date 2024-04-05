/*
 * Neraiv_SerialIO.c
 *
 *  Created on: Dec 4, 2023
 *      Author: hakan
 */
#include "Neraiv_SerialIO.h"

static SerialIOUart *sio;

void sioErrorHandler(SerialIO_ErrorTypeDef err){
	while(1){

	}
}
void sioInit(SerialIOUart* _sio, UART_HandleTypeDef* huart){
	_sio->huart = huart;
#ifdef PIN_COUNT
	_sio->pinsCreated = 0;
#endif
#ifdef POT_COUNT
	_sio->potsCreated = 0;
#endif

	HAL_UART_Receive_IT(sio->huart, sio->buffer, buffer_length);
	sio = _sio;
}



#ifdef PIN_COUNT
void sioCreatePin(uint16_t ID, GPIO_TypeDef* gpio_port, uint16_t pin_number){
	if(sio->potsCreated < (PIN_COUNT)){
		sio->pins[sio->pinsCreated].ID = ID;
		sio->pins[sio->pinsCreated].port = gpio_port;
		sio->pins[sio->pinsCreated].pin = pin_number;
		sio->pinsCreated++;
	}else{
		sioErrorHandler(SIO_ERR_MAX_NUMBER_OF_PIN_EXCEED);
	}
}
#endif
#ifdef POT_COUNT
void sioCreatePot	(uint16_t ID, GPIO_TypeDef* gpio_port, uint16_t pin_number){
	if(sio->potsCreated < (POT_COUNT)){
		sio->pots[sio->potsCreated].pin.ID = ID;
		sio->pots[sio->potsCreated].pin.port = gpio_port;
		sio->pots[sio->potsCreated].pin.pin = pin_number;
		sio->potsCreated++;
	}else{
		sioErrorHandler(SIO_ERR_MAX_NUMBER_OF_PIN_EXCEED);
	}
}
#endif

#ifdef SIGNAL_COUNT
uint32_t get_freq(SerialSignal signal){
	switch (signal.type)
	{
	case SIO_SIGNAL_PWM:
		return signal.signal.pwm.frequancy;
		break;
	case SIO_SIGNAL_COSINE:
		return signal.signal.cosine.frequancy;
		break;
	case SIO_SIGNAL_SINE:
		return signal.signal.sine.frequancy;
		break;
	case SIO_SIGNAL_SAWTOOTH:
		return signal.signal.sawtooth.frequancy;
		break;
	}
	return 0;
}
uint8_t check_for_same_timer(int8_t* signal_index){ // size 20
	for(uint8_t i = 0; i < sio->timerCreated; i++){
		if(sio->signal_timer[sio->timerCreated].htim->Instance == sio->signal_timer[sio->timerCreated].htim->Instance){
			
		}
	}
}

// Function to get the clock frequency of a timer
uint32_t get_timer_clk_freq(TIM_TypeDef* timer) {
    RCC_ClkInitTypeDef clkconfig;
    uint32_t pclk_freq;

    // Get the current clock configuration
    HAL_RCC_GetClockConfig(&clkconfig, NULL);

    // Determine the clock frequency based on the timer
    switch ((uint32_t)timer) {
        case (uint32_t)TIM1:
        case (uint32_t)TIM8:
        case (uint32_t)TIM9:
        case (uint32_t)TIM10:
        case (uint32_t)TIM11:
            pclk_freq = HAL_RCC_GetPCLK2Freq(); // These timers are on APB2 bus
            break;
        case (uint32_t)TIM2:
        case (uint32_t)TIM3:
        case (uint32_t)TIM4:
        case (uint32_t)TIM5:
        case (uint32_t)TIM6:
        case (uint32_t)TIM7:
        case (uint32_t)TIM12:
        case (uint32_t)TIM13:
        case (uint32_t)TIM14:
            pclk_freq = HAL_RCC_GetPCLK1Freq(); // These timers are on APB1 bus
            break;
        // Add cases for other timers if needed
        default:
            pclk_freq = 0; // Return 0 for unsupported timers
            break;
    }

    return pclk_freq;
}

// Function to calculate the greatest common divisor (GCD) using Euclid's algorithm
uint32_t calculateGCD(uint32_t a, uint32_t b) {
    while (b != 0) {
        uint32_t temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

// Function to calculate the least common multiple (LCM) of two numbers
uint32_t calculateLCM(uint32_t a, uint32_t b) {
    return (a * b) / calculateGCD(a, b);
}

// Function to calculate the least common multiple (LCM) of an array of frequencies
uint32_t calculateLCMFrequency(int8_t* signal_index) {
	
    uint32_t lcm = get_freq(sio->signals[signal_index[0]]);

	size_t i = 1;
	while(signal_index[i] != -1){
		uint32_t current_frequency = get_freq(sio->signals[signal_index[i]]);
        lcm = calculateLCM(lcm, current_frequency);
		i++;
	}
    return lcm;
}

/* Ortak frekans katlarını hesaplama*/
uint8_t calculateTimerParams(int8_t* signal_index){
	uint32_t prescaler, period, clock_div, pclk_freq, freq = 0;

	if(signal_index[1] != -1){
		freq = calculateLCMFrequency(signal_index);
	}else{
		freq = get_freq(sio->signals[sio->signalsCreated]);
	}
	
	pclk_freq = get_timer_clk_freq(sio->signal_timer[sio->timerCreated].htim->Instance);

	if(sio->signals[sio->signalsCreated].type == SIO_SIGNAL_PWM){
		freq = sio->signals[sio->signalsCreated].signal.pwm.frequancy;
	}

	prescaler = (pclk_freq / freq) - 1;
    period = (pclk_freq / (prescaler + 1)) / freq;
	clock_div = TIM_CLOCKDIVISION_DIV1;

    if (prescaler > 65535) { // If prescaler_value exceeds 16-bit range
        prescaler = (prescaler / 2) - 1;
        clock_div = TIM_CLOCKDIVISION_DIV2;
		if (prescaler > 65535) { 
			prescaler = (prescaler / 2) - 1;
			clock_div = TIM_CLOCKDIVISION_DIV4;
		if (prescaler > 65535) sioErrorHandler(SIO_ERR_CANT_SET_TIM); //STM32F407 için daha fazla div olmadığından tanımlanamaz
		}
    }

	sio->signal_timer[sio->timerCreated].htim->Init.Prescaler = prescaler; 
	sio->signal_timer[sio->timerCreated].htim->Init.CounterMode = TIM_COUNTERMODE_UP; 
	sio->signal_timer[sio->timerCreated].htim->Init.Period = period; 
	sio->signal_timer[sio->timerCreated].htim->Init.ClockDivision = clock_div; 
    HAL_TIM_Base_Init((sio->signal_timer[sio->timerCreated].htim));

	return 0;
}

// Sinyalleri oluşturacak olan fonksiyon
void singal_timer_interrupt_handler(){
	for(uint8_t i = 0; i < sio->timerCreated; i++){
		if (sio->signal_timer[sio->timerCreated].htim->Instance->SR & TIM_SR_UIF){
			
			sio->signal_timer[sio->timerCreated].htim->Instance->SR &= ~TIM_SR_UIF;
		}
	}
}

void sioCreateSignal(uint16_t ID, GPIO_TypeDef* gpio_port, uint16_t pin_number, SerialSignal* signal,TIM_HandleTypeDef* htim, IRQn_Type timer_irq){

	if(check_for_same_timer(htim)){

	}
	sio->signal_timer[sio->timerCreated].htim = htim;

	int8_t same_signal_check_arr[20];
	memset(same_signal_check_arr, -1, sizeof(same_signal_check_arr));
	check_for_same_timer(same_signal_check_arr);
	
	calculateTimerParams(same_signal_check_arr);
	NVIC_SetVector(timer_irq, (uint32_t)singal_timer_interrupt_handler);
	sio->signalsCreated++;	
}


#endif

// Function to search for ID in the arrays and return the element type
ElementType getElementByID(SerialIOUart* uartStruct, int targetID) {
    ElementType element;

    element.index = 0;

    for (int i = 0; i < uartStruct->pinsCreated; ++i) {
        if ((uartStruct->pins[i].ID) == targetID) {
            element.type = PIN_TYPE;
            element.index = i;
            return element;
        }
    }

    for (int i = 0; i < uartStruct->potsCreated; ++i) {
        if ((uartStruct->pots[i].pin.ID) == targetID) {
            element.type = POT_TYPE;
            element.index = i;
            return element;
        }
    }

    element.type = -1; // Indicating element not found
    return element;
}

void sioHandle(){
	uint8_t sioPinCount = 0;

#ifdef PIN_COUNT
	sioPinCount += PIN_COUNT;
#endif

#ifdef POT_COUNT
	sioPinCount += POT_COUNT;
#endif

	if(sioPinCount == 0){
		sioErrorHandler(SIO_ERR_NO_PIN_TO_EXECUTE);
	}

	uint16_t ID;

	ID = sio->buffer[1] | sio->buffer[0];


	// Perform the search and get the element type
	ElementType foundElement = getElementByID(sio, ID);

	// Output the result
	if (foundElement.type == PIN_TYPE) { // Process pinElement
		if(sio->buffer[2] == 0){

			sio->pins[foundElement.index].state = SIO_LOW;
		}else if(sio->buffer[2] == 1){
			sio->pins[foundElement.index].state = SIO_HIGH;
		}

	} else if (foundElement.type == POT_TYPE) { // Process potElement
		sio->pots[foundElement.index].pin.state = SIO_HIGH;

		union {
			uint8_t uint_bytes[4];
			float float_val;
		} data;

		// Assuming bytes[0] is the least significant byte (LSB) and bytes[3] is the most significant byte (MSB)
		data.uint_bytes[0] = sio->buffer[2];
		data.uint_bytes[1] = sio->buffer[3];
		data.uint_bytes[2] = sio->buffer[4];
		data.uint_bytes[3] = sio->buffer[5];

		sio->pots[foundElement.index].value = data.float_val;

	} else {
		sioErrorHandler(SIO_ERR_UNRECOGNIZED_TYPE);
	}
	HAL_UART_Receive_IT(sio->huart, sio->buffer, buffer_length);
}
