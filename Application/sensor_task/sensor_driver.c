/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <math.h>

#include "app.h"

#include "sensor_driver.h"
#include "stm32f4xx_ll_i2c.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum _Sensor_common_read_state_typedef_
{
	SENSOR_READ_RESET_STATE,
	SENSOR_CHECK_STATUS_STATE,
	SENSOR_READ_DATA_STATE,
	SENSOR_PROCESS_DATA_STATE,

} Sensor_common_read_state_typedef;

typedef struct _BMP390_uncomp_data_typedef_
{
	uint16_t NVM_PAR_T1;
	uint16_t NVM_PAR_T2;
	uint16_t NVM_PAR_T3;
	uint16_t NVM_PAR_P1;
	uint16_t NVM_PAR_P2;
	uint16_t NVM_PAR_P3;
	uint16_t NVM_PAR_P4;
	uint16_t NVM_PAR_P5;
	uint16_t NVM_PAR_P6;
	uint16_t NVM_PAR_P7;
	uint16_t NVM_PAR_P8;
	uint16_t NVM_PAR_P9;
	uint16_t NVM_PAR_P10;
	uint16_t NVM_PAR_P11;

} BMP390_uncomp_data_typedef;

typedef struct _BMP390_comp_data_typedef_
{
	double PAR_T1;
	double PAR_T2;
	double PAR_T3;
	double PAR_P1;
	double PAR_P2;
	double PAR_P3;
	double PAR_P4;
	double PAR_P5;
	double PAR_P6;
	double PAR_P7;
	double PAR_P8;
	double PAR_P9;
	double PAR_P10;
	double PAR_P11;

}  BMP390_comp_data_typedef;

typedef struct _BMP390_data_typedef_
{
	BMP390_uncomp_data_typedef 	uncomp_data;
	BMP390_comp_data_typedef 	comp_data;
	//double 				        linearized_temp;

} BMP390_data_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
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

static uint8_t Sensor_RX_buffer[30];
static uint8_t Sensor_Read_State = 0;
static uint8_t Sensor_Init_State = 0;

static BMP390_data_typedef 	BMP390_data;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: LSM6DSOX Command :::::::: */
static bool LSM6DSOX_init(void);
static bool LSM6DSOX_read_value(Sensor_Read_typedef read_type);
static bool LSM6DSOX_is_value_ready(Sensor_Read_typedef read_type, uint8_t *p_LSM6DSOX_RX_buffer);
static void LSM6DSOX_read_uncompensated_value(Sensor_Read_typedef read_type, uint8_t *p_LSM6DSOX_RX_buffer);
static void	LSM6DSOX_compensate_value(Sensor_Read_typedef read_type, uint8_t *p_LSM6DSOX_RX_buffer);

/* :::::::::: BMP390 Command :::::::: */
static bool BMP390_init(void);
static bool	BMP390_read_value(Sensor_Read_typedef read_type);
static bool	BMP390_is_value_ready(Sensor_Read_typedef read_type, uint8_t *p_status_value);
static void BMP390_read_uncompensated_value(Sensor_Read_typedef read_type, uint8_t *p_BMP390_RX_buffer);
static void BMP390_compensate_value(Sensor_Read_typedef read_type, uint8_t *p_BMP390_RX_buffer);
static void BMP390_compensate_temp(double uncomp_temp);
static void BMP390_compensate_pressure(double uncomp_pressure);
static void BMP390_compensate_altitude(void);

/* :::::::::: I2C Command :::::::: */
static void I2C_Mem_Write_IT(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress, uint8_t *data, uint8_t size);
static void I2C_Mem_Read_IT(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress, uint8_t *data, uint8_t size);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
bool						is_sensor_read_enable = false;

LSM6DSOX_data_typedef 		Sensor_Gyro;
LSM6DSOX_data_typedef 		Sensor_Accel;

double Sensor_Temp      	= 0.0;
double Sensor_Pressure  	= 0.0;
double Sensor_Altitude  	= 0.0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
bool Sensor_Init(void)
{
switch (Sensor_Init_State)
{
case 0:
{
	if (LSM6DSOX_init() == 1)
	{
		Sensor_Init_State = 1;
	}
	return 0;
}

case 1:
{
	if (BMP390_init() == 1)
	{
		Sensor_Init_State = 0;
		return 1;
	}
	return 0;
}

default:
	return 0;
}
}

bool Sensor_Read_Driver_Process(Sensor_Read_typedef read_type)
{
switch (read_type)
{

case SENSOR_READ_GYRO:
case SENSOR_READ_ACCEL:
case SENSOR_READ_LSM6DSOX:
{
	return LSM6DSOX_read_value(read_type);
}

case SENSOR_READ_TEMP:
case SENSOR_READ_PRESSURE:
case SENSOR_READ_ALTITUDE:
case SENSOR_READ_BMP390:
{
	return BMP390_read_value(read_type);
}

default:
	return 0;
}
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


case 2:
{
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
/* :::::::::: LSM6DSOX Command :::::::: */
static bool LSM6DSOX_init(void)
{
switch (Sensor_Read_State)
{
case 0:
{
	memset(Sensor_RX_buffer, 0, sizeof(Sensor_RX_buffer));

	//init LMD6 (0xD4 address)
	Sensor_RX_buffer[0] = 0x58;
	I2C_Mem_Write_IT(I2C_HANDLE, LSM6DSOX_ADDR, 0x10, Sensor_RX_buffer , 1);
	Sensor_Read_State = 1;
	return 0;
}
	

case 1:
{
	if (i2c_WRITE_complete == false)
	{
		return 0;
	}

	i2c_WRITE_complete = false;

	Sensor_RX_buffer[0] = 0x54;
	I2C_Mem_Write_IT(I2C_HANDLE, LSM6DSOX_ADDR, 0x11, Sensor_RX_buffer , 1);
	Sensor_Read_State = 2;
	return 0;
}


case 2:
{
	if (i2c_WRITE_complete == false)
	{
		return 0;
	}

	i2c_WRITE_complete = false;

	Sensor_Read_State = 0;
	return 1;
}
	
	
default:
	return 0;
}
}

static bool LSM6DSOX_read_value(Sensor_Read_typedef read_type)
{
switch (Sensor_Read_State)
{

case SENSOR_READ_RESET_STATE:
{
	memset(Sensor_RX_buffer, 0, sizeof(Sensor_RX_buffer));

	Sensor_Read_State = SENSOR_CHECK_STATUS_STATE;
	return 0;
}
	

case SENSOR_CHECK_STATUS_STATE:
{
	I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_STATUS_REG_ADDR, Sensor_RX_buffer , 1);

	Sensor_Read_State = SENSOR_READ_DATA_STATE;
	return 0;
}


case SENSOR_READ_DATA_STATE:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	if (LSM6DSOX_is_value_ready(read_type, Sensor_RX_buffer) == false)
	{
		Sensor_Read_State = SENSOR_CHECK_STATUS_STATE;
		return 0;
	}

	LSM6DSOX_read_uncompensated_value(read_type, &Sensor_RX_buffer[1]);

	Sensor_Read_State = SENSOR_PROCESS_DATA_STATE;
	return 0;
}
	

case SENSOR_PROCESS_DATA_STATE:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	LSM6DSOX_compensate_value(read_type, &Sensor_RX_buffer[1]);

	//is_sensor_read_enable = false;
	Sensor_Read_State = SENSOR_READ_RESET_STATE;

	return 1;
}
	

default:
	return 0;
}
}

static bool LSM6DSOX_is_value_ready(Sensor_Read_typedef read_type, uint8_t *p_status_value)
{
	switch (read_type)
	{
	case SENSOR_READ_GYRO:
		return (*p_status_value & (1 << 1));

	case SENSOR_READ_ACCEL:
		return (*p_status_value & (1 << 0));

 	case SENSOR_READ_LSM6DSOX:
		return (*p_status_value & (1 << 0)) && (*p_status_value & (1 << 1));
		
	default:
		break;
	}
	return 0;
}

static void LSM6DSOX_read_uncompensated_value(Sensor_Read_typedef read_type, uint8_t *p_LSM6DSOX_RX_buffer)
{
	switch (read_type)
	{
	case SENSOR_READ_GYRO:
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_GYRO_REG_ADDR_BASE, p_LSM6DSOX_RX_buffer , LSM6DSOX_GYRO_REG_SIZE);
		break;

	case SENSOR_READ_ACCEL:
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_ACCEL_REG_ADDR_BASE, p_LSM6DSOX_RX_buffer , LSM6DSOX_ACCEL_REG_SIZE);
		break;

 	case SENSOR_READ_LSM6DSOX:
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_READ_ALL_ADDR_BASE, p_LSM6DSOX_RX_buffer , LSM6DSOX_READ_ALL_SIZE);
		break;
		
	default:
		break;
	}
}

static void LSM6DSOX_compensate_value(Sensor_Read_typedef read_type, uint8_t *p_LSM6DSOX_RX_buffer)
{
uint8_t *p_LSM6DSOX_data;

switch (read_type)
{

case SENSOR_READ_GYRO:
{
	p_LSM6DSOX_data = (uint8_t*)&Sensor_Gyro;

	for (uint8_t i = 0; i < 6; i += 2)
	{
		p_LSM6DSOX_data[i] = (int)((p_LSM6DSOX_RX_buffer[i + 1] << 8) | p_LSM6DSOX_RX_buffer[i]) * GYRO_SENSITIVITY_500DPS;
	}

	break;
}
	

case SENSOR_READ_ACCEL:
{
	p_LSM6DSOX_data = (uint8_t*)&Sensor_Accel;

	for (uint8_t i = 0; i < 6; i += 2)
	{
		p_LSM6DSOX_data[i] = (int)((p_LSM6DSOX_RX_buffer[i + 1] << 8) | p_LSM6DSOX_RX_buffer[i]) * LSM6DSOX_ACCEL_FS_8G;
	}

	break;
}
	

case SENSOR_READ_LSM6DSOX:
{
	p_LSM6DSOX_data = (uint8_t*)&Sensor_Gyro;

	for (uint8_t i = 0; i < 6; i += 2)
	{
		p_LSM6DSOX_data[i] = (int)((p_LSM6DSOX_RX_buffer[i + 1] << 8) | p_LSM6DSOX_RX_buffer[i]) * GYRO_SENSITIVITY_500DPS;
	}

	p_LSM6DSOX_data = (uint8_t*)&Sensor_Accel;

	for (uint8_t i = 0; i < 6; i += 2)
	{
		p_LSM6DSOX_data[i] = (int)((p_LSM6DSOX_RX_buffer[i + 7] << 8) | p_LSM6DSOX_RX_buffer[i + 6]) * LSM6DSOX_ACCEL_FS_8G;
	}

	break;
}
	
	
default:
	break;
}
}

/* :::::::::: BMP390 Command :::::::: */
static bool BMP390_init(void)
{
switch (Sensor_Read_State)
{
case 0:
{
	memset(&BMP390_data, 0, sizeof(BMP390_data));
	memset(Sensor_RX_buffer, 0, sizeof(Sensor_RX_buffer));

	//set oversampling
	Sensor_RX_buffer[0] = 11;
	I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1C, Sensor_RX_buffer , 1);
	Sensor_Read_State = 1;
	return 0;
}


case 1:
{
	if (i2c_WRITE_complete == false)
	{
		return 0;
	}

	i2c_WRITE_complete = false;

	//set filter
	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x1F, Sensor_RX_buffer , 1);
	Sensor_Read_State = 2;
	return 0;
}


case 2:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}

	i2c_READ_complete = false;

	Sensor_RX_buffer[0] |= 0b00001110;
	I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1F, Sensor_RX_buffer , 1);
	Sensor_Read_State = 3;
	return 0;
}


case 3:
{
	if (i2c_WRITE_complete == false)
	{
		return 0;
	}

	i2c_WRITE_complete = false;

	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x31, Sensor_RX_buffer , 21);
	Sensor_Read_State = 4;
	return 0;
}


case 4:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}

	i2c_READ_complete = false;

	BMP390_data.uncomp_data.NVM_PAR_T1  = (Sensor_RX_buffer[1] <<8)  | Sensor_RX_buffer[0];
	BMP390_data.uncomp_data.NVM_PAR_T2  = (Sensor_RX_buffer[3] <<8)  | Sensor_RX_buffer[2];
	BMP390_data.uncomp_data.NVM_PAR_T3  =  Sensor_RX_buffer[4];
	BMP390_data.uncomp_data.NVM_PAR_P1  = (Sensor_RX_buffer[6] <<8)  | Sensor_RX_buffer[5];
	BMP390_data.uncomp_data.NVM_PAR_P2  = (Sensor_RX_buffer[8] <<8)  | Sensor_RX_buffer[7];
	BMP390_data.uncomp_data.NVM_PAR_P3  =  Sensor_RX_buffer[9];
	BMP390_data.uncomp_data.NVM_PAR_P4  =  Sensor_RX_buffer[10];
	BMP390_data.uncomp_data.NVM_PAR_P5  = (Sensor_RX_buffer[12] <<8) | Sensor_RX_buffer[11];
	BMP390_data.uncomp_data.NVM_PAR_P6  = (Sensor_RX_buffer[14] <<8) | Sensor_RX_buffer[13];
	BMP390_data.uncomp_data.NVM_PAR_P7  =  Sensor_RX_buffer[15];
	BMP390_data.uncomp_data.NVM_PAR_P8  =  Sensor_RX_buffer[16];
	BMP390_data.uncomp_data.NVM_PAR_P9  = (Sensor_RX_buffer[18] <<8) | Sensor_RX_buffer[17];
	BMP390_data.uncomp_data.NVM_PAR_P10 =  Sensor_RX_buffer[19];
	BMP390_data.uncomp_data.NVM_PAR_P11 =  Sensor_RX_buffer[20];

	double comp_value = 0.00390625f;
	BMP390_data.comp_data.PAR_T1  = ((double)(BMP390_data.uncomp_data.NVM_PAR_T1) / comp_value);

	comp_value = 1073741824.0f;
	BMP390_data.comp_data.PAR_T2  = ((double)(BMP390_data.uncomp_data.NVM_PAR_T2) / comp_value);

	comp_value = 281474976710656.0f;
	BMP390_data.comp_data.PAR_T3  = ((double)(BMP390_data.uncomp_data.NVM_PAR_T3) / comp_value);

	comp_value = 1048576.0f;
	BMP390_data.comp_data.PAR_P1  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P1 - 16384) / comp_value);

	comp_value = 536870912.0f;
	BMP390_data.comp_data.PAR_P2  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P2 - 16384) / comp_value);

	comp_value = 4294967296.0f;
	BMP390_data.comp_data.PAR_P3  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P3) / comp_value);

	comp_value = 137438953472.0f;
	BMP390_data.comp_data.PAR_P4  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P4) / comp_value);

	comp_value = 0.125f;
	BMP390_data.comp_data.PAR_P5  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P5) / comp_value);

	comp_value = 64.0f;
	BMP390_data.comp_data.PAR_P6  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P6) / comp_value);

	comp_value = 256.0f;
	BMP390_data.comp_data.PAR_P7  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P7) / comp_value);

	comp_value = 32768.0f;
	BMP390_data.comp_data.PAR_P8  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P8) / comp_value);

	comp_value = 281474976710656.0f;
	BMP390_data.comp_data.PAR_P9  = ((double)(BMP390_data.uncomp_data.NVM_PAR_P9) / comp_value);

	BMP390_data.comp_data.PAR_P10 = ((double)(BMP390_data.uncomp_data.NVM_PAR_P10) /comp_value);

	comp_value = 36893488147419103232.0f;
	BMP390_data.comp_data.PAR_P11 = ((double)(BMP390_data.uncomp_data.NVM_PAR_P11)/comp_value);

	Sensor_Read_State = 5;
	return 0;
}


case 5:
{
	memset(Sensor_RX_buffer, 0, sizeof(Sensor_RX_buffer));

	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_RX_buffer , 1);
	Sensor_Read_State = 6;
	return 0;
}


case 6:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	Sensor_RX_buffer[0] |= 0b00010011;
	I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_RX_buffer , 1);
	Sensor_Read_State = 7;
	return 0;
}


case 7:
{
	if (i2c_WRITE_complete == false)
	{
		return 0;
	}
	
	i2c_WRITE_complete = false;

	BMP390_read_uncompensated_value(SENSOR_READ_BMP390, &Sensor_RX_buffer[1]);
	Sensor_Read_State = 8;
	return 0;
}


case 8:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	BMP390_compensate_value(SENSOR_READ_BMP390, &Sensor_RX_buffer[1]);

	//is_sensor_read_enable = false;
	Sensor_Read_State = 0;

	return 1;
}

default:
	return 0;
}
}

static bool BMP390_read_value(Sensor_Read_typedef read_type)
{	
switch (Sensor_Read_State)
{
case 0:
{
	memset(Sensor_RX_buffer, 0, sizeof(Sensor_RX_buffer));

	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_RX_buffer , 1);
	Sensor_Read_State = 1;
	return 0;
}


case 1:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	Sensor_RX_buffer[0] |= 0b00010011;
	I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_RX_buffer , 1);
	Sensor_Read_State = 2;
	return 0;
}


case 2:
{
	if (i2c_WRITE_complete == false)
	{
		return 0;
	}
	
	i2c_WRITE_complete = false;

	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, BMP390_STATUS_REG_ADDR, Sensor_RX_buffer , 1);

	Sensor_Read_State = 3;
	return 0;
}


case 3:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	if (BMP390_is_value_ready(read_type, Sensor_RX_buffer) == false)
	{
		Sensor_Read_State = 0;    
		return 0;
	}

	BMP390_read_uncompensated_value(read_type, &Sensor_RX_buffer[1]);

	Sensor_Read_State = 4;
	return 0;
}


case 4:
{
	if (i2c_READ_complete == false)
	{
		return 0;
	}
	
	i2c_READ_complete = false;

	BMP390_compensate_value(read_type, &Sensor_RX_buffer[1]);

	//is_sensor_read_enable = false;
	Sensor_Read_State = 0;

	return 1;
}

default:
	return 0;
}
}

static bool BMP390_is_value_ready(Sensor_Read_typedef read_type, uint8_t *p_status_value)
{
	switch (read_type)
	{
	case SENSOR_READ_PRESSURE:
	case SENSOR_READ_ALTITUDE:
		return (*p_status_value & (1 << 4)) && (*p_status_value & (1 << 5));

	case SENSOR_READ_TEMP:
		return (*p_status_value & (1 << 4)) && (*p_status_value & (1 << 6));

 	case SENSOR_READ_BMP390:
		return (*p_status_value & (1 << 4)) && (*p_status_value & (1 << 5)) && (*p_status_value & (1 << 6));
		
	default:
		break;
	}
	return 0;
}

static void BMP390_read_uncompensated_value(Sensor_Read_typedef read_type, uint8_t *p_BMP390_RX_buffer)
{	
	if (read_type == SENSOR_READ_TEMP)
	{
		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, BMP390_TEMP_REG_ADDR_BASE, p_BMP390_RX_buffer , BMP390_TEMP_REG_SIZE);
		return;
	}
	
	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, BMP390_READ_ALL_ADDR_BASE, p_BMP390_RX_buffer , BMP390_READ_ALL_REG_SIZE);
}

static void BMP390_compensate_value(Sensor_Read_typedef read_type, uint8_t *p_BMP390_RX_buffer)
{
	switch (read_type)
	{
	case SENSOR_READ_TEMP:
		BMP390_compensate_temp((p_BMP390_RX_buffer[2] << 16) | (p_BMP390_RX_buffer[1] << 8) | p_BMP390_RX_buffer[0]);
		break;

	case SENSOR_READ_PRESSURE:
		BMP390_compensate_pressure((p_BMP390_RX_buffer[2] << 16) | (p_BMP390_RX_buffer[1] << 8) | p_BMP390_RX_buffer[0]);
		BMP390_compensate_temp((p_BMP390_RX_buffer[5] << 16) | (p_BMP390_RX_buffer[4] << 8) | p_BMP390_RX_buffer[3]);
		break;

	case SENSOR_READ_ALTITUDE:
	case SENSOR_READ_BMP390:
		BMP390_compensate_pressure((p_BMP390_RX_buffer[2] << 16) | (p_BMP390_RX_buffer[1] << 8) | p_BMP390_RX_buffer[0]);
		BMP390_compensate_temp((p_BMP390_RX_buffer[5] << 16) | (p_BMP390_RX_buffer[4] << 8) | p_BMP390_RX_buffer[3]);
		BMP390_compensate_altitude();
		break;
	
	default:
		break;
	}
}

static void BMP390_compensate_temp(double uncomp_temp)
{
	double PAR_T1;
	double PAR_T2;

	PAR_T1 = (uncomp_temp	- BMP390_data.comp_data.PAR_T1);
	PAR_T2 = (PAR_T1        * BMP390_data.comp_data.PAR_T2);

	Sensor_Temp = PAR_T2 + (PAR_T1 * PAR_T1) * BMP390_data.comp_data.PAR_T3;
}

static void BMP390_compensate_pressure(double uncomp_pressure)
{
	double PAR_T1;
	double PAR_T2;
	double PAR_T3;

	double PAR_OUT1;
	double PAR_OUT2;
    double PAR_OUT3;

	PAR_T1   = BMP390_data.comp_data.PAR_P6 * (Sensor_Temp);
	PAR_T2   = BMP390_data.comp_data.PAR_P7 * (Sensor_Temp * Sensor_Temp);
	PAR_T3   = BMP390_data.comp_data.PAR_P8 * (Sensor_Temp * Sensor_Temp * Sensor_Temp);
	PAR_OUT1 = BMP390_data.comp_data.PAR_P5 + PAR_T1 + PAR_T2 + PAR_T3;

	PAR_T1   = BMP390_data.comp_data.PAR_P2 * (Sensor_Temp);
	PAR_T2   = BMP390_data.comp_data.PAR_P3 * (Sensor_Temp * Sensor_Temp);
	PAR_T3   = BMP390_data.comp_data.PAR_P4 * (Sensor_Temp * Sensor_Temp * Sensor_Temp);
	PAR_OUT2 = uncomp_pressure * (BMP390_data.comp_data.PAR_P1 + PAR_T1 + PAR_T2+ PAR_T3);

	PAR_T1   = uncomp_pressure * uncomp_pressure;
	PAR_T2   = BMP390_data.comp_data.PAR_P9 + BMP390_data.comp_data.PAR_P10 * Sensor_Temp;
	PAR_T3   = PAR_T1 * PAR_T2;
	PAR_OUT3 = PAR_T3 + (uncomp_pressure * uncomp_pressure * uncomp_pressure)* BMP390_data.comp_data.PAR_P11;

	Sensor_Pressure = PAR_OUT1 + PAR_OUT2 + PAR_OUT3;
}

static void BMP390_compensate_altitude(void)
{
	//Sensor_Altitude = BAROMETRIC_CONSTANT * (1.0 - pow(Sensor_Pressure / ATM_PRESSURE_PA, 0.19029495718));
	Sensor_Altitude = (29.32914804) * (Sensor_Temp) * log(Sensor_Pressure / 101325.0);
}

/* :::::::::: I2C Command :::::::: */
static void I2C_Mem_Write_IT(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress, uint8_t *data, uint8_t size)
{
	i2c_address 		= devAddress;
    i2c_mem_address 	= memAddress;
    p_i2c_TX_buffer 	= data;
    i2c_TX_buffer_size 	= size;
    i2c_WRITE_complete 	= 0;
    i2c_stage 			= 0;
	i2c_write_or_read 	= 0;

    // Generate Start Condition
    LL_I2C_GenerateStartCondition(I2Cx);
    
    // Enable Buffer and Event Interrupts
    LL_I2C_EnableIT_TX(I2Cx);
    LL_I2C_EnableIT_EVT(I2Cx);
}

static void I2C_Mem_Read_IT(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress, uint8_t *data, uint8_t size)
{
	i2c_address 		= devAddress;
    i2c_mem_address 	= memAddress;
    p_i2c_RX_buffer 	= data;
    i2c_RX_buffer_size 	= size;
    i2c_READ_complete 	= 0;
    i2c_stage 			= 0;
	i2c_write_or_read 	= 1;

    // Generate Start Condition
    LL_I2C_GenerateStartCondition(I2Cx);

    // Enable Buffer and Event Interrupts
    LL_I2C_EnableIT_RX(I2Cx);
    LL_I2C_EnableIT_EVT(I2Cx);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
