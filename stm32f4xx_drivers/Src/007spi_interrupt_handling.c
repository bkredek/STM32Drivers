/*
 * 007spi_interrupt_handling.c
 *
 *  Created on: 27 cze 2020
 *      Author: Komputer PC
 */
#include "stm32f407xx.h"
#include <string.h>

//command codes
#define COMMAND_LED_CTRL          	0x50
#define COMMAND_SENSOR_READ       	0x51
#define COMMAND_LED_READ          	0x52
#define COMMAND_PRINT          	 	0x53
#define COMMAND_ID_READ         	0x54

#define LED_ON     					1
#define LED_OFF   				 	0

#define LED_PIN						9

//arduino analog pins
#define ANALOG_PIN0   				0
#define ANALOG_PIN1  				1
#define ANALOG_PIN2   				2
#define ANALOG_PIN3   				3
#define ANALOG_PIN4   				4
#define HIGH						1
#define	LOW							0
#define BTN_PRESSED					LOW

uint8_t RcvBuffer[100];
uint8_t ReadByte;
uint8_t RxContFlag = RESET;

SPI_Handle_t SPI2handle;

void delay(void)
{
	for (uint32_t i = 0; i < 500000; i++);
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_SIMPLEX_RXONLY;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}

int main(void)
{
	GPIO_ButtonInit();
	// this function is used to initialize GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize SPI2
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while( GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11));
	delay();

	SPI_PeripheralControl(SPI2, ENABLE);

	RxContFlag = SET;

	while(RxContFlag == SET) {
		while (! (SPI_ReceiveDataIT(&SPI2handle, &ReadByte, 1)) == SPI_READY );
	}

	while (SPI_GetFlagStatus(SPI2,  SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}

void SPI2_IRQHandler(void) {
	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {
	static uint32_t i =0;
	static uint8_t  rcv_start = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		if(ReadByte == 0XF1)
		{
			rcv_start = 1;
		}else
		{
			if(rcv_start)
			{
				if(ReadByte == '\r')
				{
					RxContFlag = RESET;
					rcv_start =0;
					RcvBuffer[i++] = ReadByte; //place the \r
					i=0;
				}else
				{
					RcvBuffer[i++] = ReadByte;

				}
			}
		}


	}
}
