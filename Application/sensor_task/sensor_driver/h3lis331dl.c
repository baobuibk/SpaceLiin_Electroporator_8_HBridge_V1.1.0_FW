/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// #include <math.h>

// #include "app.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "board.h"

#include "sensor_interface.h"

#include "h3lis331dl.h"

#include "i2c_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define H3LIS331DL_ADDR                 0x32
#define H3LIS331DL_STATUS_REG_ADDR      0x27

#define H3LIS331DL_ACCEL_REG_ADDR_BASE	0x28
#define H3LIS331DL_ACCEL_REG_SIZE		6

#define H3LIS331DL_READ_ALL_ADDR_BASE	0x28
#define H3LIS331DL_READ_ALL_SIZE		6

#define H3LIS331DL_X_OFS_USR 			0x73
#define H3LIS331DL_Y_OFS_USR 			0x74
#define H3LIS331DL_Z_OFS_USR 			0x75

#define H3LIS331DL_SENSITIVITY_100g	    49
#define H3LIS331DL_SENSITIVITY_200g	    98
#define H3LIS331DL_SENSITIVITY_400g	    195

#define H3LIS331DL_CALIB_TIMEOUT		5000

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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint8_t Sensor_temp_buffer[30];
static uint8_t Sensor_Read_State = 0;
static uint8_t Sensor_Read_Value_State = 0;

static float H3LIS331DL_Sensivity = H3LIS331DL_SENSITIVITY_400g;

static bool is_H3LIS331DL_Init_Complete  = false;
static bool is_H3LIS331DL_Read_Complete  = false;
// static bool is_H3LIS331DL_Calib_Complete = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool H3LIS331DL_is_value_ready(Sensor_Read_typedef read_type, uint8_t *p_status_value);
static void H3LIS331DL_read_raw_value(Sensor_Read_typedef read_type, uint8_t *p_H3LIS331DL_RX_buffer);
static void H3LIS331DL_read_accel_value(Sensor_Read_typedef read_type, uint8_t *p_H3LIS331DL_RX_buffer);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
H3LIS331DL_data_typedef H3LIS_Accel;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H3LIS331DL Command :::::::: */
bool H3LIS331DL_init(void)
{
	switch (Sensor_Read_State)
    {
	case 0:
    {
		memset(Sensor_temp_buffer, 0, sizeof(Sensor_temp_buffer));

        // CTRL_REG4(23h)
        // BIT [4:5]: FULL SCALE (FS)[0:1]   : 11
        // BIT 7    : BLOCK DATA UPDATE (BDU): 1
		Sensor_temp_buffer[0] =  0x00;
        Sensor_temp_buffer[0] |= (1 << 4);
        Sensor_temp_buffer[0] |= (1 << 5);
        Sensor_temp_buffer[0] |= (1 << 7);
		I2C_Mem_Write_IT(I2C_HANDLE, H3LIS331DL_ADDR, 0x23, Sensor_temp_buffer, 1);
		Sensor_Read_State = 1;
		return 0;
	}

	case 1:
    {
        if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }
        
		// CTRL_REG1(20h)
        // BIT [3:4]: DR[1:0]
        // BIT [5:7]: PM[2:0]
		Sensor_temp_buffer[0] =  0x07;
        Sensor_temp_buffer[0] |= (1 << 3);
        Sensor_temp_buffer[0] |= (1 << 4);
        Sensor_temp_buffer[0] |= (1 << 5);
		I2C_Mem_Write_IT(I2C_HANDLE, H3LIS331DL_ADDR, 0x20, Sensor_temp_buffer, 1);
		Sensor_Read_State = 2;
		return 0;
	}

	case 2:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		Sensor_Read_State = 0;
        is_H3LIS331DL_Init_Complete = true;
		return 1;
	}

	default:
		return 0;
	}
}

bool H3LIS331DL_read_value(Sensor_Read_typedef read_type)
{
    switch (Sensor_Read_Value_State)
    {

	case SENSOR_READ_RESET_STATE:
    {
		memset(Sensor_temp_buffer, 0, sizeof(Sensor_temp_buffer));

		Sensor_Read_Value_State = SENSOR_CHECK_STATUS_STATE;
		return 0;
	}

	case SENSOR_CHECK_STATUS_STATE:
    {
		I2C_Mem_Read_IT
        (
            I2C_HANDLE, H3LIS331DL_ADDR,
            H3LIS331DL_STATUS_REG_ADDR,
			Sensor_temp_buffer,
            1
        );

		Sensor_Read_Value_State = SENSOR_READ_DATA_STATE;
		return 0;
	}

	case SENSOR_READ_DATA_STATE:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		if (H3LIS331DL_is_value_ready(read_type, Sensor_temp_buffer) == false)
        {
			Sensor_Read_State = SENSOR_CHECK_STATUS_STATE;
			return 0;
		}

		H3LIS331DL_read_raw_value(read_type, &Sensor_temp_buffer[1]);

		Sensor_Read_Value_State = SENSOR_PROCESS_DATA_STATE;
		return 0;
	}

	case SENSOR_PROCESS_DATA_STATE:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		H3LIS331DL_read_accel_value(read_type, &Sensor_temp_buffer[1]);

		Sensor_Read_Value_State = SENSOR_READ_RESET_STATE;

        is_H3LIS331DL_Read_Complete = true;
		return 1;
	}

	default:
		return 0;
	}
}

/* :::::::::: H3LIS331DL Flag Check Command :::::::: */
bool Is_H3LIS331DL_Init_Complete(void)
{
    if (is_H3LIS331DL_Init_Complete == true)
    {
        is_H3LIS331DL_Init_Complete = false;
        return 1;
    }
    
    return 0;
}

bool Is_H3LIS331DL_Read_Complete(void)
{
    if (is_H3LIS331DL_Read_Complete == true)
    {
        is_H3LIS331DL_Read_Complete = false;
        return 1;
    }
    
    return 0;
}

/* :::::::::: Sensor_LSM6DSOX Interface :::::::: */
Sensor_Interface Sensor_H3LIS331DL =
{
    .init       = &H3LIS331DL_init,
    .read_value = &H3LIS331DL_read_value,

    .is_init_complete = &Is_H3LIS331DL_Init_Complete,
    .is_read_complete = &Is_H3LIS331DL_Read_Complete,
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool H3LIS331DL_is_value_ready(Sensor_Read_typedef read_type, uint8_t *p_status_value)
{
	switch (read_type)
    {
	case SENSOR_READ_H3LIS331DL:
		return ((*p_status_value & (1 << 3)) == (1 << 3)) ;

	default:
		break;
	}
	return 0;
}

static void H3LIS331DL_read_raw_value(Sensor_Read_typedef read_type, uint8_t *p_H3LIS331DL_RX_buffer)
{
	switch (read_type)
    {
	case SENSOR_READ_H3LIS331DL:
		I2C_Mem_Read_IT
        (
            I2C_HANDLE,
            H3LIS331DL_ADDR,
            H3LIS331DL_ACCEL_REG_ADDR_BASE,
			p_H3LIS331DL_RX_buffer,
            H3LIS331DL_ACCEL_REG_SIZE
        );

		break;

	default:
		break;
	}
}

static void H3LIS331DL_read_accel_value(Sensor_Read_typedef read_type, uint8_t *p_H3LIS331DL_RX_buffer)
{
	switch (read_type)
    {

	case SENSOR_READ_H3LIS331DL:
    {
		H3LIS_Accel.x = ((p_H3LIS331DL_RX_buffer[1] << 8) | p_H3LIS331DL_RX_buffer[0]);
		H3LIS_Accel.x *= H3LIS331DL_Sensivity;

		H3LIS_Accel.y = ((p_H3LIS331DL_RX_buffer[3] << 8) | p_H3LIS331DL_RX_buffer[2]);
		H3LIS_Accel.y *= H3LIS331DL_Sensivity;

		H3LIS_Accel.z = ((p_H3LIS331DL_RX_buffer[5] << 8) | p_H3LIS331DL_RX_buffer[4]);
		H3LIS_Accel.z *= H3LIS331DL_Sensivity;
		break;
	}

	default:
		break;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
