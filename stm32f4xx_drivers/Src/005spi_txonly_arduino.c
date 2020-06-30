/*
 * 004spi_tx_testing.c
 *
 *  Created on: 21 cze 2020
 *      Author: Komputer PC
 */

#include "stm32f407xx.h"
#include <string.h>

#define HIGH			1
#define	LOW				0
#define BTN_PRESSED		LOW

void delay(void)
{
	for (uint32_t i = 0; i < 800000; i++);
}

/*
 * PB12 --> SPI2_NSS
 * PB13 --> SPI2_SCK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 *
 * ALTF --> 5
 *
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2_Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2_Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GpioBtn);
}

int main(void)
{
	GPIO_ButtonInit();
	// this function is used to initialize GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize SPI2
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11)){
			delay();

			char user_data[] = "Option bytes to configure read and write protection, BOR level, watchdog software/hardware and reset when the device is in Standby or Stop mode.";

			// enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			//first send length information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2, &dataLen, 1);

			// send data
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

			// confirm SPI not busy
			while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

			// disable the SPI peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}

	return 0;
}

