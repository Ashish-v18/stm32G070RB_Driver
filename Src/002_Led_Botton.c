/*
 * 002_Led_Botton.c
 *
 *  Created on: Oct 7, 2024
 *      Author: Ashish kumar
 */

#include "stm32g0xx.h"

#define HIGH 0
#define BottonPressed HIGH

void delay(void)
{
	for(uint32_t i = 0; i <= 5000000; i++ );
}

int main(void)
{
	GPIO_Handle_t  GpioLed, GpioBotton;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPubPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GpioLed.pGPIOx = GPIOA;
	GPIO_Init(&GpioLed);


	GpioBotton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBotton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBotton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBotton.GPIO_PinConfig.GPIO_PinPubPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GpioBotton.pGPIOx = GPIOC;
	GPIO_Init(&GpioBotton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BottonPressed)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		}

	}
}


