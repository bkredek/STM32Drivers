/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 31 maj 2020
 *      Author: Bartłomiej Kredek
 */

#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 */

/********************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macro
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * GPIOx port Init
 */

/********************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- Initializing GPIO port
 *
 * @param[in]			- handle to the GPIO port
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temp. register

	// enable the GPIO peripherals clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configuring mode for GPIO pin
	if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// this is interrupt mode

		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		{
			//1. configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the RFTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//setting the corresponding RTSR bit
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4 );

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	temp = 0;
	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	// 3. configure pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// 5. configure the alt functionality
	if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure alternate function register
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFR[0] &= ~( 0xF << 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
			temp = 0;
		}
		else
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8)));
			pGPIOHandle->pGPIOx->AFR[1] &= ~( 0xF << (4 * ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8)));
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
			temp = 0;
		}
	}


}

/*
 * GPIOx port De-init
 */

/********************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				- Reseting the GPIOx port
 *
 * @param[in]			- handle to the GPIO port RCC reset
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */

/********************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Reading from GPIO pin
 *
 * @param[in]			- handle to the GPIO port IDR register
 * @param[in]			- pin number to read
 * @param[in]			- none
 *
 * @return				- value from GPIOx pin (0 or 1)
 *
 * @Note				- none
 *
 ********************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = ( uint8_t )((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/********************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Reading from GPIO port
 *
 * @param[in]			- handle to the GPIOx port IDR register
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- value from GPIOx port as uint16_t
 *
 * @Note				- none
 *
 ********************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = ( uint8_t )(pGPIOx->IDR);
	return value;
}

/********************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- Writing to GPIOx pin
 *
 * @param[in]			- handle to the GPIOx port ODR register
 * @param[in]			- pin number
 * @param[in]			- value to write ( 0 or 1 )
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to output data register at the bit field corresponding to pin number
		pGPIOx->ODR |= ( 1 << PinNumber );
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber );
	}
}

/********************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- Writing to GPIOx port
 *
 * @param[in]			- handle to the GPIOx port ODR register
 * @param[in]			- value to write
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/********************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- Toggle GPIOx pin
 *
 * @param[in]			- handle to the GPIOx port
 * @param[in]			- pin number to toggle
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */

/********************************************************
 * @fn					- GPIO_IRQInterruptConfig
 *
 * @brief				- Enable or disable the IRQ
 *
 * @param[in]			- IRQ number
 * @param[in]			- Enable or Disable value
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if ( IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
		}
		else if (IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if ( IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );
		}
		else if (IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );
		}
	}
}

/********************************************************
 * @fn					- GPIO_IRQConfig
 *
 * @brief				- Enable or disable the IRQ
 *
 * @param[in]			- IRQ number
 * @param[in]			- Enable or Disable value
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );

	*(NVIC_PR_BASE_ADDR + (iprx)) |= ( IRQPriority << shift_amount );
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr corresponding to pin number
	if (EXTI->PR & ( 1 << PinNumber ))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber );
	}
}
