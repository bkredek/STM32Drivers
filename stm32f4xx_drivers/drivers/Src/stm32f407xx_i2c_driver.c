/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 5 lip 2020
 *      Author: Komputer PC
 */

#include "stm32f407xx_i2c_driver.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPLLOutputClock(void);
// private function declaration

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

// prescaler for ahb

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

/********************************************************
 * @fn					- I2C_Init
 *
 * @brief				- Initializing I2Cx
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t tempreg = 0;

	// enable the clock for I2Cx
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// enable acking

	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure SCL speed

	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// configure device address

	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// configuring I2C mode

	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
		// mode is standard
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else {
		// mode is fast
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		// check duty cycle
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else {
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	// configure rise time for I2C pins
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// standard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) +1;
	}else {
		// fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);


}

/********************************************************
 * @fn					- I2C_DeInit
 *
 * @brief				- DeInitializing I2Cx
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2CHandle->pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2CHandle->pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/********************************************************
 * @fn					- I2C_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given I2Cx
 *
 * @param[in]			- base address of the I2Cx peripheral
 * @param[in]			- ENABLE or DISABLE macro
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/********************************************************
 * @fn					- I2C_PeripheralControl
 *
 * @brief				- API to enable I2C
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- enable or disable
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE);
	}
}

/********************************************************
 * @fn					- I2C_MasterSendData
 *
 * @brief				- API to send data from master to slave
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- tx buffer pointer
 * @param[in]			- length of data
 * @param[in]			- slave address
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr) {
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send the data until Len become 0
	while(Len > 0) {
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// 7. When Len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// Note: TXE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF = 1 SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: Generating STOP, automatically clears BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/********************************************************
 * @fn					- I2C_GetFlagStatus
 *
 * @brief				- API to check status flag is set
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- flag name
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- bool value
 *
 * @Note				- none
 *
 ********************************************************/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


// Helper functions

uint32_t RCC_GetPLLOutputClock(void) {
	uint32_t clk = 0;
	return clk;
}

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, apb1p, ahbp;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {
		SystemClk = 16000000;
	}else if(clksrc == 1) {
		SystemClk = 8000000;
	}else if(clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8) {
		ahbp = 1;
	}else {
		ahbp = AHB_PreScaler[temp - 8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4) {
		apb1p = 1;
	}else {
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/********************************************************
 * @fn					- I2C_GenerateStartCondition
 *
 * @brief				- API to generate start condition
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- none
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2cx) {
	pI2cx->CR1 |= (1 << I2C_CR1_START);
}

/********************************************************
 * @fn					- I2C_GenerateStopCondition
 *
 * @brief				- API to generate stop condition
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- none
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/********************************************************
 * @fn					- I2C_ExecuteAddressPhase
 *
 * @brief				- API to set slave address
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- slave address
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

/********************************************************
 * @fn					- I2C_ClearADDRFlag
 *
 * @brief				- API to clear ADDR flag
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- none
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx) {
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}
