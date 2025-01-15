/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "board.h"

#include "sensor_interface.h"

#include "bmp390.h"

#include "i2c_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define BMP390_ADDR                     0xEE
#define BMP390_STATUS_REG_ADDR			0x03

#define BMP390_PRESSURE_REG_ADDR_BASE	0x04
#define BMP390_PRESSURE_REG_SIZE		3

#define BMP390_TEMP_REG_ADDR_BASE		0x07
#define BMP390_TEMP_REG_SIZE			3

#define BMP390_ALTITUDE_REG_ADDR_BASE	0x07
#define BMP390_ALTITUDE_REG_SIZE		3

#define BMP390_READ_ALL_ADDR_BASE		0x04
#define BMP390_READ_ALL_REG_SIZE		6

#define SEALEVEL_PRESSURE_HPA 			1037.48

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

} BMP390_comp_data_typedef;

typedef struct _BMP390_data_typedef_
{
	BMP390_uncomp_data_typedef uncomp_data;
	BMP390_comp_data_typedef   comp_data;

} BMP390_data_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint8_t Sensor_temp_buffer[30];
static uint8_t Sensor_Read_State = 0;
//static uint8_t Sensor_Read_Value_State = 0;
//static uint8_t Sensor_Init_State = 0;

static BMP390_data_typedef BMP390_data;

static bool is_BMP390_Init_Complete  = false;
static bool is_BMP390_Read_Complete  = false;
static bool is_BMP390_Calib_Complete = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool BMP390_is_value_ready
(
    Sensor_Read_typedef read_type,
	uint8_t *p_status_value
);

static void BMP390_read_uncompensated_value
(
    Sensor_Read_typedef read_type,
	uint8_t *p_BMP390_RX_buffer
);

static void BMP390_compensate_value
(
    Sensor_Read_typedef read_type,
	uint8_t *p_BMP390_RX_buffer
);

static void BMP390_compensate_temp(double uncomp_temp);
static void BMP390_compensate_pressure(double uncomp_pressure);
static void BMP390_compensate_altitude(void);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
double Sensor_Temp = 0.0;
double Sensor_Pressure = 0.0;
double Sensor_Altitude = 0.0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: BMP390 Command :::::::: */
bool BMP390_init(void)
{
	switch (Sensor_Read_State)
    {
	case 0:
    {
		memset(&BMP390_data, 0, sizeof(BMP390_data));
		memset(Sensor_temp_buffer, 0, sizeof(Sensor_temp_buffer));

		//set oversampling
		Sensor_temp_buffer[0] = 11;
		I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1C, Sensor_temp_buffer, 1);
		Sensor_Read_State = 1;
		return 0;
	}

	case 1:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		//set filter
		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x1F, Sensor_temp_buffer, 1);
		Sensor_Read_State = 2;
		return 0;
	}

	case 2:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		Sensor_temp_buffer[0] |= 0b00001110;
		I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1F, Sensor_temp_buffer, 1);
		Sensor_Read_State = 3;
		return 0;
	}

	case 3:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x31, Sensor_temp_buffer, 21);
		Sensor_Read_State = 4;
		return 0;
	}

	case 4:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		BMP390_data.uncomp_data.NVM_PAR_T1 = (Sensor_temp_buffer[1] << 8)
				| Sensor_temp_buffer[0];
		BMP390_data.uncomp_data.NVM_PAR_T2 = (Sensor_temp_buffer[3] << 8)
				| Sensor_temp_buffer[2];
		BMP390_data.uncomp_data.NVM_PAR_T3 = Sensor_temp_buffer[4];
		BMP390_data.uncomp_data.NVM_PAR_P1 = (Sensor_temp_buffer[6] << 8)
				| Sensor_temp_buffer[5];
		BMP390_data.uncomp_data.NVM_PAR_P2 = (Sensor_temp_buffer[8] << 8)
				| Sensor_temp_buffer[7];
		BMP390_data.uncomp_data.NVM_PAR_P3 = Sensor_temp_buffer[9];
		BMP390_data.uncomp_data.NVM_PAR_P4 = Sensor_temp_buffer[10];
		BMP390_data.uncomp_data.NVM_PAR_P5 = (Sensor_temp_buffer[12] << 8)
				| Sensor_temp_buffer[11];
		BMP390_data.uncomp_data.NVM_PAR_P6 = (Sensor_temp_buffer[14] << 8)
				| Sensor_temp_buffer[13];
		BMP390_data.uncomp_data.NVM_PAR_P7 = Sensor_temp_buffer[15];
		BMP390_data.uncomp_data.NVM_PAR_P8 = Sensor_temp_buffer[16];
		BMP390_data.uncomp_data.NVM_PAR_P9 = (Sensor_temp_buffer[18] << 8)
				| Sensor_temp_buffer[17];
		BMP390_data.uncomp_data.NVM_PAR_P10 = Sensor_temp_buffer[19];
		BMP390_data.uncomp_data.NVM_PAR_P11 = Sensor_temp_buffer[20];

		double comp_value = 0.00390625f;
		BMP390_data.comp_data.PAR_T1 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_T1) / comp_value);

		comp_value = 1073741824.0f;
		BMP390_data.comp_data.PAR_T2 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_T2) / comp_value);

		comp_value = 281474976710656.0f;
		BMP390_data.comp_data.PAR_T3 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_T3) / comp_value);

		comp_value = 1048576.0f;
		BMP390_data.comp_data.PAR_P1 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P1 - 16384)
						/ comp_value);

		comp_value = 536870912.0f;
		BMP390_data.comp_data.PAR_P2 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P2 - 16384)
						/ comp_value);

		comp_value = 4294967296.0f;
		BMP390_data.comp_data.PAR_P3 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P3) / comp_value);

		comp_value = 137438953472.0f;
		BMP390_data.comp_data.PAR_P4 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P4) / comp_value);

		comp_value = 0.125f;
		BMP390_data.comp_data.PAR_P5 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P5) / comp_value);

		comp_value = 64.0f;
		BMP390_data.comp_data.PAR_P6 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P6) / comp_value);

		comp_value = 256.0f;
		BMP390_data.comp_data.PAR_P7 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P7) / comp_value);

		comp_value = 32768.0f;
		BMP390_data.comp_data.PAR_P8 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P8) / comp_value);

		comp_value = 281474976710656.0f;
		BMP390_data.comp_data.PAR_P9 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P9) / comp_value);

		BMP390_data.comp_data.PAR_P10 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P10) / comp_value);

		comp_value = 36893488147419103232.0f;
		BMP390_data.comp_data.PAR_P11 =
				((double) (BMP390_data.uncomp_data.NVM_PAR_P11) / comp_value);

		Sensor_Read_State = 5;
		return 0;
	}

	case 5:
    {
		memset(Sensor_temp_buffer, 0, sizeof(Sensor_temp_buffer));

		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_temp_buffer, 1);
		Sensor_Read_State = 6;
		return 0;
	}

	case 6:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		Sensor_temp_buffer[0] |= 0b00010011;
		I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_temp_buffer, 1);
		Sensor_Read_State = 7;
		return 0;
	}

	case 7:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		BMP390_read_uncompensated_value(SENSOR_READ_BMP390, &Sensor_temp_buffer[1]);
		Sensor_Read_State = 8;
		return 0;
	}

	case 8:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		BMP390_compensate_value(SENSOR_READ_BMP390, &Sensor_temp_buffer[1]);

		//is_sensor_read_enable = false;
		Sensor_Read_State = 0;

        is_BMP390_Init_Complete = true;
		return 1;
	}

	default:
		return 0;
	}
}

bool BMP390_read_value(Sensor_Read_typedef read_type)
{
	switch (Sensor_Read_State)
    {
	case 0:
    {
		memset(Sensor_temp_buffer, 0, sizeof(Sensor_temp_buffer));

		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_temp_buffer, 1);
		Sensor_Read_State = 1;
		return 0;
	}

	case 1:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		Sensor_temp_buffer[0] |= 0b00010011;
		I2C_Mem_Write_IT(I2C_HANDLE, BMP390_ADDR, 0x1B, Sensor_temp_buffer, 1);
		Sensor_Read_State = 2;
		return 0;
	}

	case 2:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, BMP390_STATUS_REG_ADDR,
				Sensor_temp_buffer, 1);

		Sensor_Read_State = 3;
		return 0;
	}

	case 3:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		if (BMP390_is_value_ready(read_type, Sensor_temp_buffer) == false)
        {
			Sensor_Read_State = 0;
			return 0;
		}

		BMP390_read_uncompensated_value(read_type, &Sensor_temp_buffer[1]);

		Sensor_Read_State = 4;
		return 0;
	}

	case 4:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		BMP390_compensate_value(read_type, &Sensor_temp_buffer[1]);

		//is_sensor_read_enable = false;
		Sensor_Read_State = 0;

        is_BMP390_Read_Complete = true;
		return 1;
	}

	default:
		return 0;
	}
}

/* :::::::::: BMP390 Flag Check Command :::::::: */
bool Is_BMP390_Init_Complete()
{
    if (is_BMP390_Init_Complete == true)
    {
        is_BMP390_Init_Complete = false;
        return 1;
    }
    
    return 0;
}

bool Is_BMP390_Read_Complete()
{
    if (is_BMP390_Read_Complete == true)
    {
        is_BMP390_Read_Complete = false;
        return 1;
    }
    
    return 0;
}

bool Is_BMP390_Calib_Complete()
{
    if (is_BMP390_Calib_Complete == true)
    {
        is_BMP390_Calib_Complete = false;
        return 1;
    }
    
    return 0;
}

/* :::::::::: Sensor_BMP390 Interface :::::::: */
Sensor_Interface Sensor_BMP390 =
{
    .init       = &BMP390_init,
    .read_value = &BMP390_read_value,

    .is_init_complete = &Is_BMP390_Init_Complete,
    .is_read_complete = &Is_BMP390_Read_Complete,
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
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
		return (*p_status_value & (1 << 4)) && (*p_status_value & (1 << 5))
				&& (*p_status_value & (1 << 6));

	default:
		break;
	}
	return 0;
}

static void BMP390_read_uncompensated_value(Sensor_Read_typedef read_type, uint8_t *p_BMP390_RX_buffer)
{
	if (read_type == SENSOR_READ_TEMP)
    {
		I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, BMP390_TEMP_REG_ADDR_BASE,
				p_BMP390_RX_buffer, BMP390_TEMP_REG_SIZE);
		return;
	}

	I2C_Mem_Read_IT(I2C_HANDLE, BMP390_ADDR, BMP390_READ_ALL_ADDR_BASE,
			p_BMP390_RX_buffer, BMP390_READ_ALL_REG_SIZE);
}

static void BMP390_compensate_value(Sensor_Read_typedef read_type, uint8_t *p_BMP390_RX_buffer)
{
	switch (read_type)
    {
	case SENSOR_READ_TEMP:
		BMP390_compensate_temp(
				(p_BMP390_RX_buffer[2] << 16) | (p_BMP390_RX_buffer[1] << 8)
						| p_BMP390_RX_buffer[0]);
		break;

	case SENSOR_READ_PRESSURE:
		BMP390_compensate_pressure(
				(p_BMP390_RX_buffer[2] << 16) | (p_BMP390_RX_buffer[1] << 8)
						| p_BMP390_RX_buffer[0]);
		BMP390_compensate_temp(
				(p_BMP390_RX_buffer[5] << 16) | (p_BMP390_RX_buffer[4] << 8)
						| p_BMP390_RX_buffer[3]);
		break;

	case SENSOR_READ_ALTITUDE:
	case SENSOR_READ_BMP390:
		BMP390_compensate_pressure(
				(p_BMP390_RX_buffer[2] << 16) | (p_BMP390_RX_buffer[1] << 8)
						| p_BMP390_RX_buffer[0]);
		BMP390_compensate_temp(
				(p_BMP390_RX_buffer[5] << 16) | (p_BMP390_RX_buffer[4] << 8)
						| p_BMP390_RX_buffer[3]);
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

	PAR_T1 = (uncomp_temp - BMP390_data.comp_data.PAR_T1);
	PAR_T2 = (PAR_T1 * BMP390_data.comp_data.PAR_T2);

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

	PAR_T1 = BMP390_data.comp_data.PAR_P6 * (Sensor_Temp);
	PAR_T2 = BMP390_data.comp_data.PAR_P7 * (Sensor_Temp * Sensor_Temp);
	PAR_T3 = BMP390_data.comp_data.PAR_P8
			* (Sensor_Temp * Sensor_Temp * Sensor_Temp);
	PAR_OUT1 = BMP390_data.comp_data.PAR_P5 + PAR_T1 + PAR_T2 + PAR_T3;

	PAR_T1 = BMP390_data.comp_data.PAR_P2 * (Sensor_Temp);
	PAR_T2 = BMP390_data.comp_data.PAR_P3 * (Sensor_Temp * Sensor_Temp);
	PAR_T3 = BMP390_data.comp_data.PAR_P4
			* (Sensor_Temp * Sensor_Temp * Sensor_Temp);
	PAR_OUT2 = uncomp_pressure
			* (BMP390_data.comp_data.PAR_P1 + PAR_T1 + PAR_T2 + PAR_T3);

	PAR_T1 = uncomp_pressure * uncomp_pressure;
	PAR_T2 = BMP390_data.comp_data.PAR_P9
			+ BMP390_data.comp_data.PAR_P10 * Sensor_Temp;
	PAR_T3 = PAR_T1 * PAR_T2;
	PAR_OUT3 = PAR_T3
			+ (uncomp_pressure * uncomp_pressure * uncomp_pressure)
					* BMP390_data.comp_data.PAR_P11;

	Sensor_Pressure = PAR_OUT1 + PAR_OUT2 + PAR_OUT3;
}

static void BMP390_compensate_altitude(void)
{
	float atmospheric = Sensor_Pressure / 100.0F;
 	Sensor_Altitude =  44330.0 * (1.0 - pow(atmospheric / SEALEVEL_PRESSURE_HPA, 0.1903));
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
