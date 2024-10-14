/*
 * stm32g0xx_gpio_driver.c
 *
 *  Created on: Sep 27, 2024
 *      Author: Ashish Kumar
 */
#include "stm32g0xx_gpio_driver.h"


/*******************************************************
* @fn               - GPIO_PeriClockControl
*
* @brief            - Enables or disables the peripheral clock for the specified GPIO port.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral (GPIOA, GPIOB, etc.).
* @param[in]        - EnorDi: Macro that specifies whether to enable or disable the clock (ENABLE or DISABLE).
*
* @return           - None
*
* @note             - This function is essential for enabling the clock before any GPIO configuration or operation. Without enabling the clock, GPIO functions may not work as expected.
*******************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
    }
}


/*******************************************************
 * @fn               - GPIO_Init
 *
 * @brief            - Initializes the specified GPIO pin by configuring the mode, speed, pull-up/pull-down settings, output type, and alternate functionality (if applicable).
 *
 * @param[in]        - pGPIOHandle: Pointer to the GPIO handle structure that contains the configuration information for the specified pin.
 *
 * @return           - None
 *
 * @note             - This function configures a GPIO pin's mode (input, output, alternate function, or analog), speed, pull-up/pull-down resistor settings, output type, and alternate functionality (if the pin is set to alternate mode).
 *                     It does not handle interrupt modes, which need to be configured separately.
 *******************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0; // temporary register

    // 1. Configure the mode of the GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
        pGPIOHandle->pGPIOx->MODER |= temp; // Setting
    }
    else
    {
        // TODO: Handle interrupt mode configuration here
    	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
    	{
    		// 1.Configure the FTSR
    		EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		//Clear the corresponding RTSR bit
    		EXTI ->RTSR1  &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



    	}
    	else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
    		//1.Configure the RTSR
    		EXTI-> RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		//Clear the corresponding FTSR bit
    		EXTI ->FTSR1  &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
    	else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
    	{
    		//1. Configure both FTSR and RTSR
    		EXTI ->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    		EXTI ->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	}
    	//2.Configure the GPIO port selection in SYSCFG
    	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
    	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
    	uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
    	EXTI ->EXTICR[temp1] &= ~(0xF << (temp2 * 4));  // Clear previous bits
    	EXTI ->EXTICR[temp1] |= (portCode << (temp2 * 4));  // Set GPIO port

    	//3.Congigure the exti interrupt delivery using IMR
    	EXTI ->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    }

    temp = 0;

    // 2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp; // Setting

    temp = 0;

    // 3. Configure the pull-up/pull-down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPubPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp; // Setting

    temp = 0;

    // 4. Configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing 1 bit
    pGPIOHandle->pGPIOx->OTYPER |= temp; // Setting

    temp = 0;

    // 5. Configure the alternate functionality (if applicable)
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Setting
    }
}


/*******************************************************
* @fn               - GPIO_DeInit
*
* @brief            - Resets the specified GPIO port, returning its registers to their default state.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral (GPIOA, GPIOB, etc.).
*
* @return           - None
*
* @note             - This function resets all registers of the specified GPIO port, essentially "de-initializing" the port and bringing it back to its default state. Use with caution as it will clear all configurations for that GPIO port.
*******************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
    	GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
}


/*******************************************************
* @fn               - GPIO_ReadFromInputPin
*
* @brief            - Reads the value from a specific GPIO input pin.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral from which the pin value is to be read.
* @param[in]        - PinNumber: The pin number to read from (0 to 15 depending on the port).
*
* @return           - uint8_t: Returns the pin state (0 or 1).
*
* @note             - This function reads the state of a specified GPIO pin and returns either 0 (low) or 1 (high).
*******************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*******************************************************
* @fn               - GPIO_ReadFromInputPort
*
* @brief            - Reads the entire 16-bit value from the input data register of a GPIO port.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral from which the port value is to be read.
*
* @return           - uint16_t: Returns the 16-bit value representing the state of all the pins in the specified GPIO port.
*
* @note             - This function reads the input data register (IDR) of the specified GPIO port and returns the value of all the pins in the port.
*******************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*******************************************************
* @fn               - GPIO_WriteToOutputPin
*
* @brief            - Writes a value (high or low) to the specified GPIO output pin.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral to which the pin belongs.
* @param[in]        - PinNumber: The pin number to which the value is to be written (0 to 15 depending on the port).
* @param[in]        - Value: The value to be written to the pin (GPIO_PIN_SET or GPIO_PIN_RESET).
*
* @return           - None
*
* @note             - This function directly modifies the output data register (ODR) to set or clear the state of a specific GPIO pin.
*******************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
		{
			// Write 1 to the output data registor at the bit field corresponding to the pin number
			pGPIOx->ODR |= (1<<PinNumber);
		}
		else
		{
			// Write 0
			pGPIOx->ODR &= ~(1<<PinNumber);
		}
}

/*******************************************************
* @fn               - GPIO_WriteToOutputPort
*
* @brief            - Writes a 16-bit value to the entire GPIO output port.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral to which the port belongs.
* @param[in]        - Value: The 16-bit value to be written to the output data register (ODR) of the port.
*
* @return           - None
*
* @note             - This function directly writes the provided 16-bit value to the output data register (ODR), affecting all pins in the specified GPIO port.
*******************************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*******************************************************
* @fn               - GPIO_ToggleOutputPin
*
* @brief            - Toggles the state of the specified GPIO output pin.
*
* @param[in]        - pGPIOx: Base address of the GPIO peripheral to which the pin belongs.
* @param[in]        - PinNumber: The pin number to be toggled (0 to 15 depending on the port).
*
* @return           - None
*
* @note             - This function toggles the state of the specified GPIO pin by using a bitwise XOR operation on the output data register (ODR).
*******************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*******************************************************
* @fn               - GPIO_IRQInterruptConfig
*
* @brief            - Configures the interrupt for a specified GPIO IRQ line.
*
* @param[in]        - IRQNumber: The number of the interrupt request to configure.
*                     This corresponds to the specific GPIO interrupt line (0 to 31).
* @param[in]        - EnorDi: Specifies whether to enable or disable the interrupt.
*                     Use ENABLE to enable the interrupt or DISABLE to disable it.
*
* @return           - None
*
* @note             - This function directly manipulates the NVIC (Nested Vector Interrupt Controller)
*                     registers to enable or disable the specified interrupt. Ensure that the
*                     IRQNumber is within the valid range (0 to 31) before calling this function.
*******************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// Program ISER register
			*NVIC_ISER |= (1 << IRQNumber);
		}

	}else
		if(IRQNumber <= 31)
		{
			// Program ICER register
			*NVIC_ICER |= (1 << IRQNumber);
		}
}

/*******************************************************
* @fn               - GPIO_IRQPriorityConfig
*
* @brief            - Configures the priority of a specified GPIO interrupt.
*
* @param[in]        - IRQNumber: The number of the interrupt request to configure.
*                     This corresponds to the specific GPIO interrupt line.
* @param[in]        - IRQPriority: The priority level to assign to the specified IRQ.
*                     This value should be set according to the application's priority requirements.
*
* @return           - None
*
* @note             - The priority value should be set within the limits defined by the
*                     microcontroller's NVIC. The number of priority bits implemented
*                     is defined by `NO_PR_BITS_IMPLEMENTED`. Ensure that the priority
*                     value does not exceed this limit.
*******************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. First lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}



/***********************************************************************************
 * @fn          - GPIO_IRQHandling
 * @brief       - Handles GPIO interrupt requests for a specified pin.
 *
 * This function checks the interrupt status for the specified GPIO pin and handles
 * rising and falling edge interrupts. It clears the interrupt flags after handling
 * the interrupts to ensure that the system is ready for future interrupts.
 *
 * @param[in]   PinNumber - The pin number for which the interrupt is being handled.
 *                           This should correspond to the GPIO pin number associated
 *                           with the EXTI (External Interrupt) configuration.
 *
 * @return      - None
 *
 * @note        - The function should contain the specific interrupt handling code
 *                where indicated. The interrupt flags in the EXTI registers must
 *                be checked to determine the type of interrupt that occurred (rising
 *                or falling).
 *************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Check if the interrupt flag for the specified pin is set
	    if (EXTI->RPR1 & (1 << PinNumber))  // Check Rising Pending Register
	    {
	        // Handle the rising edge interrupt for the specified pin here
	        // TODO: Add your specific interrupt handling code here

	        // Clear the interrupt flag for the rising edge
	        EXTI->RPR1 |= (1 << PinNumber);
	    }

	    // Check for falling edge interrupt
	    if (EXTI->FPR1 & (1 << PinNumber))  // Check Falling Pending Register
	    {
	        // Handle the falling edge interrupt for the specified pin here
	        // TODO: Add your specific interrupt handling code here

	        // Clear the interrupt flag for the falling edge
	        EXTI->FPR1 |= (1 << PinNumber);
	    }
}


