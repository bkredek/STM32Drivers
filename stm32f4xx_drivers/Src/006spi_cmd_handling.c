/*
 * 004spi_tx_testing.c
 *
 *  Created on: 21 cze 2020
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

uint8_t SPI_SlaveLedStatus(void) {
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	uint8_t commandcode = COMMAND_LED_READ;

	uint8_t ackbyte;
	uint8_t args[2];

	SPI_SendData(SPI2, &commandcode, 1);

	SPI_ReceiveData(SPI2, &dummy_read, 1);

	SPI_SendData(SPI2, &dummy_write, 1);

	SPI_ReceiveData(SPI2, &ackbyte, 1);

	if(SPI_VerifyResponse(ackbyte)) {
		args[0] = LED_PIN;
		SPI_SendData(SPI2, args, 1);
	}

	SPI_ReceiveData(SPI2, &dummy_read, 1);
	SPI_SendData(SPI2, &dummy_write, 1);

	delay();

	uint8_t led_status;
	SPI_ReceiveData(SPI2, &led_status, 1);

	return led_status;
}

void SPI_Print(void) {
	uint8_t commandcode = COMMAND_PRINT;
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	uint8_t message[] = "Hello there ARDUINO";

	uint8_t ackbyte;
	uint8_t args[2];

	SPI_SendData(SPI2, &commandcode, 1);
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	SPI_SendData(SPI2, &dummy_write, 1);
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	if (SPI_VerifyResponse(ackbyte)) {
		args[0] = strlen((char*)message);
		SPI_SendData(SPI2, args, 1);
		SPI_SendData(SPI2, message, args[0]);
	}
}

void SPI_SlaveID(void) {
	uint8_t commandcode = COMMAND_ID_READ;
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	uint8_t ackbyte;
	uint8_t id[11];

	SPI_SendData(SPI2, &commandcode, 1);
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	SPI_SendData(SPI2, &dummy_write, 1);
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	if (SPI_VerifyResponse(ackbyte)) {
		for(uint32_t i = 0; i < 10; i++) {
			SPI_SendData(SPI2, &dummy_write, 1);
			SPI_ReceiveData(SPI2, &id[i], 1);
		}

		id[11] = '\0';
	}
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	uint8_t led_flag = 0;

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

			// enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			// 1. command number 1 CMD_LED_CTRL  <pin no(1)>  <value(1)>
			uint8_t commandcode = COMMAND_LED_CTRL;
			uint8_t ackbyte;
			uint8_t args[2];

			// send command
			SPI_SendData(SPI2, &commandcode, 1);

			//do dummy read to clear off RXEN
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// send some dummy bits (1byte) to fetch the response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			// read ack byte receive
			SPI_ReceiveData(SPI2, &ackbyte, 1);

			if(SPI_VerifyResponse(ackbyte))
			{
				//send arguments
				args[0] = LED_PIN;
				args[1] = led_flag == 0 ? LED_ON : LED_OFF;
				SPI_SendData(SPI2, args, 2);
			}

			// 2. command number 2 CMD_SENSOR_READ   <analog pin number (1)>

			// wait till button is pressed
			while(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_11));
			delay();

			commandcode = COMMAND_SENSOR_READ;
			// send command
			SPI_SendData(SPI2, &commandcode, 1);

			//do dummy read to clear off RXEN
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// send some dummy bits (1byte) to fetch the response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			// read ack byte receive
			SPI_ReceiveData(SPI2, &ackbyte, 1);

			if(SPI_VerifyResponse(ackbyte))
			{
				//send arguments
				args[0] = ANALOG_PIN0;
				SPI_SendData(SPI2, args, 1);
			}

			//do dummy read to clear off RXEN
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//delay to slave can read adc
			delay();

			// send some dummy bits (1byte) to fetch the response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);

			while(GPIO_ReadFromInputPin(GPIOD,  GPIO_PIN_NO_11));
			delay();

			uint8_t ledStatus = SPI_SlaveLedStatus();
			led_flag = ledStatus;

			while(GPIO_ReadFromInputPin(GPIOD,  GPIO_PIN_NO_11));
			delay();
			SPI_Print();

			while(GPIO_ReadFromInputPin(GPIOD,  GPIO_PIN_NO_11));
			delay();
			SPI_SlaveID();

			// confirm SPI not busy
			while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

			// disable the SPI peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}

	return 0;
}

