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
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

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
 * @param[in]			- repeated start condition
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle);

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
	if(Sr == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/********************************************************
 * @fn					- I2C_MasterReciveData
 *
 * @brief				- API to send data from master to slave
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- rx buffer pointer
 * @param[in]			- length of data
 * @param[in]			- slave address
 * @param[in]			- repeated start condition
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_MasterReciveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that start generation is completed by checking the SB flag in SR1 register
	// Note: Until SB is cleared SCL will be stretched (pulled to low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
	// 3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	// 4. Wait until address phase is completed by checking the ADDR flag in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only one byte
	if(Len == 1) {
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//Wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		//Generate STOP condition
		if(Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read more than one byte
	if(Len > 1) {
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes 0
		for(uint32_t i = 0; i < Len; i++) {
			//wait until RXEN becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if(i == (Len - 1)) { //if last 2 bytes are remaining
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				//generate stop condition
				if(Sr == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			//read the data from data register in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxbuffer++;
		}
	}

	// re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/********************************************************
 * @fn					- I2C_MasterSendDataIT
 *
 * @brief				- API to send data from master to slave
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- tx buffer pointer
 * @param[in]			- length of data
 * @param[in]			- slave address
 * @param[in]			- repeated start condition
 * @return				- bool state value
 *
 * @Note				- none
 *
 ********************************************************/

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}

	return busystate;
}

/********************************************************
 * @fn					- I2C_MasterReciveDataIT
 *
 * @brief				- API to send data from master to slave
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- rx buffer pointer
 * @param[in]			- length of data
 * @param[in]			- slave address
 * @param[in]			- repeated start condition
 *
 * @return				- bool state value
 *
 * @Note				- none
 *
 ********************************************************/

uint8_t I2C_MasterReciveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pRxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN );

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN );

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN );
	}

	return busystate;
}

/********************************************************
 * @fn					- I2C_EV_IRQHandling
 *
 * @brief				- API to check status flag is set
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
// Helper Function for RXNE and TXE cases
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
		if(pI2CHandle->TxLen > 0){
			//1. Load data into DR
			pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
			//2. Decrement the TxLen
			pI2CHandle->TxLen--;
			//3. Increment the buffer address
			pI2CHandle->pTxBuffer++;
		}
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
	if(pI2CHandle->RxSize == 1){
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxSize--;
	}

	if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2) {
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxSize--;
	}

	if(pI2CHandle->RxLen == 0) {
		//close the I2C reception

		//1. Generate STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2. Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}
// ==============================
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );

	//1. Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode
	if (temp1 && temp3) {
		//SB flag is set
		//Executing address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2. Handle for interrupt generated by ADDR event
	// Note: When master mode: Address is sent
	//		 When slave mode: Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );

	if (temp1 && temp3) {
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF (byte transfer finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );

	if (temp1 && temp3) {
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE )){
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0){
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					//2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);
					//3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			;
		}
	}

	//4. Handle for interrupt generated by STOPF event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );

	if (temp1 && temp3) {
		//STOPF flag is set

		//Clear STOPF flag
		//1. Read SR1 2. Write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify that STOP is generated by the master
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );

	if (temp1 && temp2 && temp3) {
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
			//TXE flag is set
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
	}

	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );

	if (temp1 && temp2 && temp3) {
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL )){
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

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

/********************************************************
 * @fn					- I2C_IRQInterruptConfig
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
 *******************************************************/

void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi) {
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
 * @fn					- I2C_IRQPriorityConfig
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED );

	*(NVIC_PR_BASE_ADDR + (iprx)) |= ( IRQPriority << shift_amount );
}

// Helper functions

void I2C_CloseReceiveData(I2C_Handle_t *pI2cHandle) {
	//Disable the ITBUFEN control bit
	pI2cHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Disable the ITEVFEN control bit
	pI2cHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2cHandle->TxRxState = I2C_READY;
	pI2cHandle->pRxBuffer = NULL;
	pI2cHandle->RxLen = 0;
	pI2cHandle->RxSize = 0;
	if(pI2cHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2cHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2cHandle) {
	//Disable the ITBUFEN control bit
	pI2cHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN );

	//Disable the ITEVFEN control bit
	pI2cHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN );

	pI2cHandle->TxRxState = I2C_READY;
	pI2cHandle->pTxBuffer = NULL;
	pI2cHandle->TxLen = 0;
}

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
 * @fn					- I2C_ExecuteAddressPhaseWrite
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

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

/********************************************************
 * @fn					- I2C_ExecuteAddressPhaseWrite
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

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
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

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	//Check device master or slave mode
	uint32_t dummyread;
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)) {
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxSize == 1) {
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear ADDR flag by reading SR1 and SR2
				dummyread = pI2CHandle->pI2Cx->SR1;
				dummyread = pI2CHandle->pI2Cx->SR2;
				(void)dummyread;
			}
		}else {
			//clear ADDR flag by reading SR1 and SR2
			dummyread = pI2CHandle->pI2Cx->SR1;
			dummyread = pI2CHandle->pI2Cx->SR2;
			(void)dummyread;
		}
	}else {
		//device is in slave mode
		//clear ADDR flag by reading SR1 and SR2
		dummyread = pI2CHandle->pI2Cx->SR1;
		dummyread = pI2CHandle->pI2Cx->SR2;
		(void)dummyread;
	}
}

/********************************************************
 * @fn					- I2C_ManageAcking
 *
 * @brief				- API to manage ACKING
 *
 * @param[in]			- handle to the I2Cx
 * @param[in]			- EnOrDi
 * @param[in]			- none
 * @param[in]			- none
 *
 * @return				- none
 *
 * @Note				- none
 *
 ********************************************************/

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
	if(EnOrDi == I2C_ACK_ENABLE) {
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else {
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
