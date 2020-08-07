/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 5 lip 2020
 *      Author: Komputer PC
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint32_t I2C_DeviceAddress;
	uint32_t I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct {
	I2C_RegDef_t	 *pI2Cx;
	I2C_Config_t	 I2C_Config;
	uint8_t 		 *pTxBuffer;
	uint8_t 		 *pRxBuffer;
	uint32_t    	 TxLen;
	uint32_t		 RxLen;
	uint8_t			 TxRxState;
	uint8_t			 DevAddr;
	uint32_t		 RxSize;
	uint8_t			 Sr;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM4K			400000
#define I2C_SCL_SPEED_FM2K			200000

/*
 * @I2C_ACKControl
 */

#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

/*
 * @I2C_STATUS_FLAG related status flags definitions
 */

#define I2C_FLAG_TXE				(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE				(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB					(1 << I2C_SR1_SB)
#define I2C_FLAG_BTF				(1 << I2C_SR1_BTF)
#define I2C_FLAG_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_OVR				(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT			(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_AF					(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO				(1 << I2C_SR1_ARLO)
#define I2C_FLAG_ADDR				(1 << I2C_SR1_ADDR)
#define I2C_FLAG_ADD10				(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF				(1 << I2C_SR1_STOPF)
#define I2C_FLAG_SMBALERT			(1 << I2C_SR1_SMBALERT)

/*
 * I2C application states
 */

#define I2C_READY					0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2

/*********************************************************************************************
 * 							APIs supported by this driver									 *
 * 			For more information about the APIs check the function definitions				 *
 *********************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

/*
 * Data Send and Recive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReciveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReciveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2cHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2cHandle);
/*
 * IRQ Configuration and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * API to get status flag
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Other peripherals control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#define I2C_DISABLE_SR					RESET
#define I2C_ENABLE_SR					SET

#define I2C_EV_TX_CMPLT					0
#define I2C_EV_RX_CMPLT					1
#define I2C_EV_STOP						2

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
