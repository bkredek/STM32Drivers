/*
 * 008i2c_master_tx_testing.c
 *
 *  Created on: 18 lip 2020
 *      Author: Komputer PC
 */

#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

// semihosting printf debug option
//extern void initialise_monitor_handles(void);

#define MY_ADDR					0x61
#define SLAVE_ADDR				0x68

// receive buffer

uint8_t rcv_buff[32];

/*
 * PB6 - SCL line
 * PB7 - SDA line
 */

I2C_Handle_t I2C1Handle;

void delay(void)
{
	for (uint32_t i = 0; i < 500000; i++);
}

void I2C1_GPIOInits(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void) {
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIObtn;

	GPIObtn.pGPIOx = GPIOA;
	GPIObtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIObtn);
}

int main(void) {

	//initialise_monitor_handles();

	uint8_t commandcode;
	uint8_t len;

	//i2c pins init
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	GPIO_ButtonInit();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// enable the ack
	I2C_ManageAcking(I2C1, ENABLE);

	// wait for btn press
	while(1) {
		// wait for btn pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();

		commandcode = 0x51;
		// sending command code
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);
		// len of rcv data
		I2C_MasterReciveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		// receive data
		I2C_MasterReciveData(&I2C1Handle, rcv_buff, len, SLAVE_ADDR, I2C_DISABLE_SR);

		printf("Message from the Arduino: %s\n", rcv_buff);
	}


	return 0;
}
