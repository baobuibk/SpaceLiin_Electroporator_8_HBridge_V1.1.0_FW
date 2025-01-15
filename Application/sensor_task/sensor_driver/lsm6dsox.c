/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// #include <math.h>

// #include "app.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "board.h"

#include "sensor_interface.h"

#include "lsm6dsox.h"

#include "i2c_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define LSM6DSOX_ADDR                   0xD4
#define LSM6DSOX_STATUS_REG_ADDR		0x1E

#define LSM6DSOX_GYRO_REG_ADDR_BASE		0x22
#define LSM6DSOX_GYRO_REG_SIZE			6

#define LSM6DSOX_ACCEL_REG_ADDR_BASE	0x28
#define LSM6DSOX_ACCEL_REG_SIZE			6

#define LSM6DSOX_READ_ALL_ADDR_BASE		0x22
#define LSM6DSOX_READ_ALL_SIZE			12

#define LSM6DSOX_X_OFS_USR 				0x73
#define LSM6DSOX_Y_OFS_USR 				0x74
#define LSM6DSOX_Z_OFS_USR 				0x75

#define GYRO_SENSITIVITY_500DPS		    17.50f
#define LSM6DSOX_ACCEL_FS_2G			0.122f

#define LSM6DSOX_CALIB_TIMEOUT			5000

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
// static uint8_t Sensor_Init_State = 0;

// static uint8_t Sensor_Calib_State = 0;
// static uint8_t Sensor_Calib_Count = 0;
// static int16_t Sensor_Calib_Timeout = LSM6DSOX_CALIB_TIMEOUT;
// static LSM6DSOX_data_typedef Sensor_Calib_Data;
static float Accel_Calib_Factor = LSM6DSOX_ACCEL_FS_2G;

static bool is_LSM6DSOX_Init_Complete  = false;
static bool is_LSM6DSOX_Read_Complete  = false;
static bool is_LSM6DSOX_Calib_Complete = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool LSM6DSOX_is_value_ready
(
    Sensor_Read_typedef read_type,
	uint8_t *p_LSM6DSOX_RX_buffer
);

static void LSM6DSOX_read_uncompensated_value
(
    Sensor_Read_typedef read_type,
	uint8_t *p_LSM6DSOX_RX_buffer
);

static void LSM6DSOX_compensate_value
(
    Sensor_Read_typedef read_type,
	uint8_t *p_LSM6DSOX_RX_buffer
);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
LSM6DSOX_data_typedef Sensor_Gyro;
LSM6DSOX_data_typedef Sensor_Accel;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: LSM6DSOX Command :::::::: */
bool LSM6DSOX_init(void)
{
	switch (Sensor_Read_State)
    {
	case 0:
    {
		memset(Sensor_temp_buffer, 0, sizeof(Sensor_temp_buffer));

		Sensor_temp_buffer[0] = 0x4C;
		I2C_Mem_Write_IT(I2C_HANDLE, LSM6DSOX_ADDR, 0x11, Sensor_temp_buffer, 1);
		Sensor_Read_State = 1;
		return 0;
	}

	case 1:
    {
        if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }
        
		Sensor_temp_buffer[0] = 0x9A;
		I2C_Mem_Write_IT(I2C_HANDLE, LSM6DSOX_ADDR, 0x10, Sensor_temp_buffer, 1);
		Sensor_Read_State = 2;
		return 0;
	}

	case 2:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		Sensor_temp_buffer[0] = 0x00;		//0x02 for use user offset
		I2C_Mem_Write_IT(I2C_HANDLE, LSM6DSOX_ADDR, 0x16, Sensor_temp_buffer, 1);
		Sensor_Read_State = 3;
		return 0;
	}

	case 3:
    {
		if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		Sensor_temp_buffer[0] = 0x09;
		I2C_Mem_Write_IT(I2C_HANDLE, LSM6DSOX_ADDR, 0x17, Sensor_temp_buffer, 1);
		Sensor_Read_State = 4;
		return 0;
	}

	case 4:
    {
        if (Is_I2C_Write_Complete() == false)
        {
            return 0;
        }

		Sensor_Read_State = 0;
        is_LSM6DSOX_Init_Complete = true;
		return 1;
    }

	default:
		return 0;
	}
}

bool LSM6DSOX_read_value(Sensor_Read_typedef read_type)
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
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_STATUS_REG_ADDR,
				Sensor_temp_buffer, 1);

		Sensor_Read_Value_State = SENSOR_READ_DATA_STATE;
		return 0;
	}

	case SENSOR_READ_DATA_STATE:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		if (LSM6DSOX_is_value_ready(read_type, Sensor_temp_buffer) == false)
        {
			Sensor_Read_State = SENSOR_CHECK_STATUS_STATE;
			return 0;
		}

		LSM6DSOX_read_uncompensated_value(read_type, &Sensor_temp_buffer[1]);

		Sensor_Read_Value_State = SENSOR_PROCESS_DATA_STATE;
		return 0;
	}

	case SENSOR_PROCESS_DATA_STATE:
    {
		if (Is_I2C_Read_Complete() == false)
        {
            return 0;
        }

		LSM6DSOX_compensate_value(read_type, &Sensor_temp_buffer[1]);

		Sensor_Read_Value_State = SENSOR_READ_RESET_STATE;

        is_LSM6DSOX_Read_Complete = true;
		return 1;
	}

	default:
		return 0;
	}
}

// bool LSM6DSOX_Calib()
// {
// //	return 1;
// 	float Calib;

// 	switch (Sensor_Calib_State)
//     {
// 	case 0:
//     {
//         //begin calib
// 		Sensor_Calib_Count = 0;

// 		is_sensor_read_complete = false;
// 		Sensor_Read_Value(SENSOR_READ_ACCEL);

// 		Sensor_Calib_State = 1;
// 		Sensor_Calib_Timeout = LSM6DSOX_CALIB_TIMEOUT;
        
// 		Sensor_Calib_Data.x = 0;
// 		Sensor_Calib_Data.y = 0;
// 		Sensor_Calib_Data.z = 0;
// 		return 0;
//     }

// 	case 1:
//     {
//         if (is_sensor_read_complete == true)
//         {
// 			Sensor_Calib_Timeout = LSM6DSOX_CALIB_TIMEOUT;
// 			Sensor_Calib_Count++;

// 			if (Sensor_Calib_Count > 19)
//             {
// 				Sensor_Calib_State = 2;
// 				return 0;
// 			}

// 			Sensor_Calib_Data.x += Sensor_Accel.x;
// 			Sensor_Calib_Data.y += Sensor_Accel.y;
// 			Sensor_Calib_Data.z += Sensor_Accel.z;

// 			is_sensor_read_complete = false;
// 			Sensor_Read_Value(SENSOR_READ_ACCEL);
// 			return 0;
// 		} 
//         else
//         {
// 			if (Sensor_Calib_Timeout > 0)
// 				Sensor_Calib_Timeout--;
// 			else
// 				Sensor_Calib_State = 5;		//Timeout
// 			return 0;
// 		}

// 		return 0;
//     }
		
// 	case 2:
//     {
//         Sensor_Calib_Data.x /= 20;
// 		Sensor_Calib_Data.y /= 20;
// 		Sensor_Calib_Data.z /= 20;

// 		if((abs(Sensor_Calib_Data.x) > 100) || (abs(Sensor_Calib_Data.y) > 100))
//         {
// 			UART_Send_String(&RS232_UART, "\nLOG: Ensure the accelerometer is placed flat on the ground\n");
// 			Sensor_Calib_State = 0;
// 			return 0;
// 		}

// 		if(abs(Sensor_Calib_Data.z - 1000) < 100)
//         {
// 			Sensor_Calib_State = 0;
// 			UART_Send_String(&RS232_UART, "Calib done\n");
// 			UART_Printf(&RS232_UART, "Scale Factor changes to %f\n", Accel_Calib_Factor);

//             is_LSM6DSOX_Calib_Complete = true;
// 			return 1;
// 		}
// 		Calib = Sensor_Calib_Data.z;

// 		Calib /= Accel_Calib_Factor;
// 		Calib = 1000 / Calib;
// 		Accel_Calib_Factor = Calib;
// 		Sensor_Calib_State = 0;
// 		return 0;
//     }

// 	case 5:
//     {
//         UART_Send_String(&RS232_UART, "\nInit fail, being reset...\n");
// 		Sensor_Calib_State = 0;
// 		return 0;
//     }

//     default:
//         return 0;
		
// 	}
// }

/* :::::::::: LSM6DSOX Flag Check Command :::::::: */
bool Is_LSM6DSOX_Init_Complete()
{
    if (is_LSM6DSOX_Init_Complete == true)
    {
        is_LSM6DSOX_Init_Complete = false;
        return 1;
    }
    
    return 0;
}

bool Is_LSM6DSOX_Read_Complete()
{
    if (is_LSM6DSOX_Read_Complete == true)
    {
        is_LSM6DSOX_Read_Complete = false;
        return 1;
    }
    
    return 0;
}

bool Is_LSM6DSOX_Calib_Complete()
{
    if (is_LSM6DSOX_Calib_Complete == true)
    {
        is_LSM6DSOX_Calib_Complete = false;
        return 1;
    }
    
    return 0;
}

/* :::::::::: Sensor_LSM6DSOX Interface :::::::: */
Sensor_Interface Sensor_LSM6DSOX =
{
    .init       = &LSM6DSOX_init,
    .read_value = &LSM6DSOX_read_value,

    .is_init_complete = &Is_LSM6DSOX_Init_Complete,
    .is_read_complete = &Is_LSM6DSOX_Read_Complete,
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
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
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_GYRO_REG_ADDR_BASE,
				p_LSM6DSOX_RX_buffer, LSM6DSOX_GYRO_REG_SIZE);
		break;

	case SENSOR_READ_ACCEL:
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_ACCEL_REG_ADDR_BASE,
				p_LSM6DSOX_RX_buffer, LSM6DSOX_ACCEL_REG_SIZE);
		break;

	case SENSOR_READ_LSM6DSOX:
		I2C_Mem_Read_IT(I2C_HANDLE, LSM6DSOX_ADDR, LSM6DSOX_READ_ALL_ADDR_BASE,
				p_LSM6DSOX_RX_buffer, LSM6DSOX_READ_ALL_SIZE);
		break;

	default:
		break;
	}
}

static void LSM6DSOX_compensate_value(Sensor_Read_typedef read_type, uint8_t *p_LSM6DSOX_RX_buffer)
{
	uint8_t *p_LSM6DSOX_data;
	//float data;

	switch (read_type)
    {

	case SENSOR_READ_GYRO:
    {
		p_LSM6DSOX_data = (uint8_t*) &Sensor_Gyro;

		for (uint8_t i = 0; i < 6; i += 2)
        {
			p_LSM6DSOX_data[i] = (int) ((p_LSM6DSOX_RX_buffer[i + 1] << 8)
					| p_LSM6DSOX_RX_buffer[i]) * GYRO_SENSITIVITY_500DPS;
		}

		break;
	}

	case SENSOR_READ_ACCEL:
    {
		Sensor_Accel.x = ((p_LSM6DSOX_RX_buffer[1] << 8)
				| p_LSM6DSOX_RX_buffer[0]);
		Sensor_Accel.x *= Accel_Calib_Factor;
		Sensor_Accel.y = ((p_LSM6DSOX_RX_buffer[3] << 8)
				| p_LSM6DSOX_RX_buffer[2]);
		Sensor_Accel.y *= Accel_Calib_Factor;
		Sensor_Accel.z = ((p_LSM6DSOX_RX_buffer[5] << 8)
				| p_LSM6DSOX_RX_buffer[4]);
		Sensor_Accel.z *= Accel_Calib_Factor;
		break;
	}

	case SENSOR_READ_LSM6DSOX:
    {
		p_LSM6DSOX_data = (uint8_t*) &Sensor_Gyro;

		for (uint8_t i = 0; i < 6; i += 2)
        {
			p_LSM6DSOX_data[i] = (int) ((p_LSM6DSOX_RX_buffer[i + 1] << 8)
					| p_LSM6DSOX_RX_buffer[i]) * GYRO_SENSITIVITY_500DPS;
		}

		p_LSM6DSOX_data = (uint8_t*) &Sensor_Accel;

		for (uint8_t i = 0; i < 6; i += 2)
        {
			p_LSM6DSOX_data[i] = (int) ((p_LSM6DSOX_RX_buffer[i + 7] << 8)
					| p_LSM6DSOX_RX_buffer[i + 6]) * Accel_Calib_Factor;
		}

		break;
	}

	default:
		break;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
