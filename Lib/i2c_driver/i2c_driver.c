/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdbool.h>

#include "i2c_driver.h"

#include "stm32f4xx_ll_i2c.h"

#include "board.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
volatile uint8_t *p_i2c_TX_buffer;
volatile uint8_t i2c_TX_buffer_size;
volatile uint8_t i2c_WRITE_complete = 0;

volatile uint8_t *p_i2c_RX_buffer;
volatile uint8_t i2c_RX_buffer_size;
volatile uint8_t i2c_READ_complete = 0;

volatile uint8_t i2c_address;
volatile uint8_t i2c_mem_address;
volatile uint8_t i2c_stage;
volatile uint8_t i2c_write_or_read;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: I2C Command :::::::: */
void I2C_Mem_Write_IT(I2C_TypeDef *I2Cx, uint8_t devAddress,
		uint8_t memAddress, uint8_t *data, uint8_t size)
{
	i2c_address = devAddress;
	i2c_mem_address = memAddress;
	p_i2c_TX_buffer = data;
	i2c_TX_buffer_size = size;
	i2c_WRITE_complete = 0;
	i2c_stage = 0;
	i2c_write_or_read = 0;

	// Generate Start Condition
	LL_I2C_GenerateStartCondition(I2Cx);

	// Enable Buffer and Event Interrupts
	LL_I2C_EnableIT_TX(I2Cx);
	LL_I2C_EnableIT_EVT(I2Cx);
}

void I2C_Mem_Read_IT(I2C_TypeDef *I2Cx, uint8_t devAddress,
		uint8_t memAddress, uint8_t *data, uint8_t size)
{
	i2c_address = devAddress;
	i2c_mem_address = memAddress;
	p_i2c_RX_buffer = data;
	i2c_RX_buffer_size = size;
	i2c_READ_complete = 0;
	i2c_stage = 0;
	i2c_write_or_read = 1;

	// Generate Start Condition
	LL_I2C_GenerateStartCondition(I2Cx);

	// Enable Buffer and Event Interrupts
	LL_I2C_EnableIT_RX(I2Cx);
	LL_I2C_EnableIT_EVT(I2Cx);
}

bool Is_I2C_Write_Complete()
{
	if (i2c_WRITE_complete == true)
	{
		i2c_WRITE_complete = false;
		return 1;
	}
	
	return 0;
}

bool Is_I2C_Read_Complete()
{
	if (i2c_READ_complete == true)
	{
		i2c_READ_complete = false;
		return 1;
	}
	
	return 0;
}

void I2C_EV_IRQHandler(void)
{
	switch (i2c_stage)
	{

// Stage 0: Send Device Address with Write
	case 0:
	{
		if (LL_I2C_IsActiveFlag_SB(I2C_HANDLE) == false)
		{
			return;
		}

		LL_I2C_TransmitData8(I2C_HANDLE, i2c_address);
		i2c_stage = 1;
		return;
	}

// Stage 1: Send Memory Address
	case 1:
	{
		if (LL_I2C_IsActiveFlag_ADDR(I2C_HANDLE) == false)
		{
			return;
		}

		LL_I2C_ClearFlag_ADDR(I2C_HANDLE);
		LL_I2C_TransmitData8(I2C_HANDLE, i2c_mem_address);
		i2c_stage = 2;
		return;
	}

	case 2: {
		// Stage 2: Send Data for Writing
		if (i2c_write_or_read == 0)
		{
			if (LL_I2C_IsActiveFlag_TXE(I2C_HANDLE) == false)
			{
				return;
			}

			if (i2c_TX_buffer_size > 0)
			{
				LL_I2C_TransmitData8(I2C_HANDLE, *p_i2c_TX_buffer++);
				i2c_TX_buffer_size--;
			} 
			else
			{
				i2c_WRITE_complete = 1;
				i2c_stage = 3;
				LL_I2C_GenerateStopCondition(I2C_HANDLE);
				LL_I2C_DisableIT_TX(I2C_HANDLE);
				LL_I2C_DisableIT_EVT(I2C_HANDLE);
				return;
			}

			if (i2c_TX_buffer_size > 0)
			{
				i2c_mem_address += 1;
				i2c_stage = 0;
				LL_I2C_GenerateStartCondition(I2C_HANDLE);
				return;
			}
		}
		// Stage 2: Generate Re-Start for Reading
		else
		{
			if (LL_I2C_IsActiveFlag_TXE(I2C_HANDLE) == false)
			{
				return;
			}

			LL_I2C_GenerateStartCondition(I2C_HANDLE);
			i2c_stage = 3;
			return;
		}

	}

// Stage 3: Send Device Address with Read
	case 3:
	{
		if (LL_I2C_IsActiveFlag_SB(I2C_HANDLE) == false)
		{
			return;
		}

		LL_I2C_TransmitData8(I2C_HANDLE, i2c_address | 1);
		i2c_stage = 4;
		return;
	}

// Stage 4: Check for the read mem addr and start read mem
	case 4:
	{
		if (LL_I2C_IsActiveFlag_ADDR(I2C_HANDLE) == false)
		{
			return;
		}

		LL_I2C_ClearFlag_ADDR(I2C_HANDLE);

		i2c_stage = 5;
		return;
	}

	case 5:
	{
		if (LL_I2C_IsActiveFlag_RXNE(I2C_HANDLE) == false)
		{
			return;
		}

		if (i2c_RX_buffer_size == 1)
		{
			LL_I2C_AcknowledgeNextData(I2C_HANDLE, LL_I2C_NACK);
		}

		if (i2c_RX_buffer_size > 0)
		{
			*p_i2c_RX_buffer++ = LL_I2C_ReceiveData8(I2C_HANDLE);
			i2c_RX_buffer_size--;
		}

		if (i2c_RX_buffer_size == 0)
		{
			i2c_READ_complete = 1;
			i2c_stage = 6;
			LL_I2C_GenerateStopCondition(I2C_HANDLE);
			LL_I2C_DisableIT_RX(I2C_HANDLE);
			LL_I2C_DisableIT_EVT(I2C_HANDLE);
			return;
		}

		if (i2c_RX_buffer_size > 0)
		{
			i2c_mem_address += 1;
			i2c_stage = 0;
			LL_I2C_GenerateStartCondition(I2C_HANDLE);
			return;
		}

		return;
	}

	default:
		break;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
