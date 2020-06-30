/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 19 cze 2020
 *      Author: Komputer PC
 */


#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_overrun_error_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * SPI_GetFlagStatus generic function
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/********************************************************
 * @fn					- SPI_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given SPIx
 *
 * @param[in]			- base address of the SPIx peripheral
 * @param[in]			- ENABLE or DISABLE macro
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/********************************************************
 * @fn					- SPI_Init
 *
 * @brief				- Initializing SPIx
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// SPI peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure the SPI_CR1 register

	uint32_t tempreg = 0;

	// 1.configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. bus configuration
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		// RXONLY should be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/********************************************************
 * @fn					- SPI_DeInit
 *
 * @brief				- DeInitializing SPIx
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_DeInit(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIHandle->pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIHandle->pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/********************************************************
 * @fn					- SPI_SendData
 *
 * @brief				- API to send data SPIx
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- transfer data pointer
 * @param[in]			- length of data
 *
 * @return				- none
 *
 * @Note				- This is blocking call
 *
 ********************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while( Len > 0)
	{
		// 1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check DFF bit in CR1

		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// 1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 bit DFF
			// 1. load the data into the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/********************************************************
 * @fn					- SPI_ReciveData
 *
 * @brief				- API to receive data SPIx
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- transfer data pointer
 * @param[in]			- length of data
 *
 * @return				- none
 *
 * @Note				- This is blocking call
 *
 ********************************************************/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while( Len > 0)
	{
		// 1. wait until RXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. check DFF bit in CR1

		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// 1. load the data into the Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 bit DFF
			// 1. load the data into the Rxbuffer address
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/********************************************************
 * @fn					- SPI_SendDataIT
 *
 * @brief				- Enable or disable the IRQ
 *
 * @param[in]			- handle to SPI peripheral
 * @param[in]			- pointer to TX buffer
 * @param[in]			- data length
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		// 1. Save the TX buffer address and length information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TX flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	// 4. Data transmission will be handled by ISR code

	return state;
}

/********************************************************
 * @fn					- SPI_ReciveDataIT
 *
 * @brief				- Enable or disable the IRQ
 *
 * @param[in]			- handle to SPI peripheral
 * @param[in]			- pointer to RX buffer
 * @param[in]			- data length
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {
		// 1. Save the RX buffer address and length information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RX flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}

	// 4. Data transmission will be handled by ISR code

	return state;
}

/********************************************************
 * @fn					- SPI_PeripheralControl
 *
 * @brief				- API to enable SPI
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- enable or disable
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}

/********************************************************
 * @fn					- SPI_SSIConfig
 *
 * @brief				- API to enable SPI
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- enable or disable
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}

/********************************************************
 * @fn					- SPI_SSOEConfig
 *
 * @brief				- API to enable SPI
 *
 * @param[in]			- handle to the SPIx
 * @param[in]			- enable or disable
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
		}
}

/********************************************************
 * @fn					- SPI_IRQInterruptConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn					- SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );

	*(NVIC_PR_BASE_ADDR + (iprx)) |= ( IRQPriority << shift_amount );
}

/********************************************************
 * @fn					- SPI_IRQHandling
 *
 * @brief				- handling the IRQ
 *
 * @param[in]			- handle to SPI
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t temp1, temp2;
	// check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		// handle txe
		spi_txe_interrupt_handle(pHandle);
	}

	// check for rxne
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// handle rxne
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SR & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2) {
		// handle overrun error
		spi_overrun_error_interrupt_handle(pHandle);
	}

}

// helper functions

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

}

static void spi_overrun_error_interrupt_handle(SPI_Handle_t *pSPIHandle) {

}







