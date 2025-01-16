/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <stdio.h>
#include "app.h"

#include "sensor_task.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef struct _sensor_task_typedef_
{
                uint16_t    buffer_size;
                uint8_t*    p_buffer;

    volatile    uint16_t    write_index;
    volatile    uint16_t    read_index;
    //volatile    char        RX_char;
} sensor_task_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool			is_sensor_init = false;

sensor_task_typedef Sensor_Task;
uint8_t				Sensor_Task_buffer[16];

uint8_t				Sensor_Init_State = 0;

static bool is_sensor_read_complete = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool Sensor_Init(void);
static bool Sensor_Read_Driver_Process(Sensor_Read_typedef read_type);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint8_t      is_buffer_full(volatile uint16_t *pui16Read,
                            volatile uint16_t *pui16Write, uint16_t ui16Size);

static uint8_t      is_buffer_empty(volatile uint16_t *pui16Read,
                            volatile uint16_t *pui16Write);

static uint16_t     get_buffer_count(volatile uint16_t *pui16Read,
                            volatile uint16_t *pui16Write, uint16_t ui16Size);

static uint16_t     advance_buffer_index(volatile uint16_t* pui16Index, uint16_t ui16Size);

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the receive buffer.
//
//*****************************************************************************
#define SENSOR_TASK_BUFFER_SIZE(p_sensor_task)          	((p_sensor_task)->buffer_size)

#define SENSOR_TASK_BUFFER_USED(p_sensor_task)          	(get_buffer_count(	&(p_sensor_task)->read_index,  \
                                                                  				&(p_sensor_task)->write_index, \
                                                                  			 	 (p_sensor_task)->buffer_size))

#define IS_SENSOR_TASK_BUFFER_FREE(p_sensor_task)           (CMD_BUFFER_SIZE - CMD_BUFFER_USED(p_sensor_task))

#define IS_SENSOR_TASK_BUFFER_EMPTY(p_sensor_task)          (is_buffer_empty(	&(p_sensor_task)->read_index,   \
                                                                  				&(p_sensor_task)->write_index))

#define IS_SENSOR_TASK_BUFFER_FULL(p_sensor_task)           (is_buffer_full(	&(p_sensor_task)->read_index,  \
                                                                 			    &(p_sensor_task)->write_index, \
                                                                 		 	     (p_sensor_task)->buffer_size))

#define ADVANCE_SENSOR_TASK_WRITE_INDEX(p_sensor_task) 	    (advance_buffer_index( &(p_sensor_task)->write_index, \
                                                                      			    (p_sensor_task)->buffer_size))

#define ADVANCE_SENSOR_TASK_READ_INDEX(p_sensor_task)   	(advance_buffer_index( &(p_sensor_task)->read_index, \
                                                                      				(p_sensor_task)->buffer_size))

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
bool is_accel_calib = true;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void Sensor_Task_Init(void)
{
	Sensor_Task.p_buffer       	= Sensor_Task_buffer;
    Sensor_Task.buffer_size    	= 16;
    Sensor_Task.write_index 	= 0;
    Sensor_Task.read_index		= 0;

    if(Sensor_Task.buffer_size != 0)
    {
        memset((void *)Sensor_Task.p_buffer, 0, sizeof(Sensor_Task.p_buffer));
    }
}

void Sensor_Read_Value(Sensor_Read_typedef read_type)
{
	Sensor_Task.p_buffer[Sensor_Task.write_index] = read_type;
	ADVANCE_SENSOR_TASK_WRITE_INDEX(&Sensor_Task);
}

void Sensor_Read_Task(void*)
{
	if (is_sensor_init == false)
	{
		is_sensor_init = Sensor_Init();

		if (is_sensor_init == true)
		{
			UART_Printf(&RS232_UART, "Sensor Init success\n");
			UART_Send_String(&RS232_UART, "> ");
			return;
		}

		return;
	}
	
//	if(is_accel_calib == false)
//    {
//        if(LSM6DSOX_Calib() == true)
//        {
//            is_accel_calib = true;
//        }
//    }

	if (IS_SENSOR_TASK_BUFFER_EMPTY(&Sensor_Task) == true)
	{
		return;
	}

    if (Sensor_Read_Driver_Process(Sensor_Task.p_buffer[Sensor_Task.read_index]) == false)
    {
        return;
    }

    is_sensor_read_complete = true;

    ADVANCE_SENSOR_TASK_READ_INDEX(&Sensor_Task);
}

bool Is_Sensor_Read_Complete(void)
{
	if (is_sensor_read_complete == true)
	{
		is_sensor_read_complete = false;
		return 1;
	}
	
	return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool Sensor_Init(void)
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
			Sensor_Init_State = 2;
		}
		return 0;
	}

	case 2:
    {
		if (H3LIS331DL_init() == 1)
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

static bool Sensor_Read_Driver_Process(Sensor_Read_typedef read_type)
{
    Sensor_Interface* sensor = NULL;

	switch (read_type)
    {

	case SENSOR_READ_GYRO:
	case SENSOR_READ_ACCEL:
	case SENSOR_READ_LSM6DSOX:
    {
		sensor = &Sensor_LSM6DSOX;
        break;
	}

	case SENSOR_READ_TEMP:
	case SENSOR_READ_PRESSURE:
	case SENSOR_READ_ALTITUDE:
	case SENSOR_READ_BMP390:
    {
		sensor = &Sensor_BMP390;
        break;
	}

	case SENSOR_READ_H3LIS331DL:
	{
		sensor = &Sensor_H3LIS331DL;
		break;
	}

	default:
		return 0;
	}

    return sensor->read_value(read_type);
}

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b 1 if the buffer is full or \b 0 otherwise.
//
//*****************************************************************************

static uint8_t is_buffer_full(volatile uint16_t *pui16Read,
             volatile uint16_t *pui16Write, uint16_t ui16Size)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((((ui16Write + 1) % ui16Size) == ui16Read) ? 1 : 0);
}


//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b 1 if the buffer is empty or \b 0 otherwise.
//
//*****************************************************************************

static uint8_t is_buffer_empty(volatile uint16_t *pui16Read,
              volatile uint16_t *pui16Write)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((ui16Read == ui16Write) ? 1 : 0);
}


//*****************************************************************************
//
//! Determines the number of bytes of data contained in a ring buffer.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************

static uint16_t get_buffer_count(volatile uint16_t *pui16Read,
               volatile uint16_t *pui16Write, uint16_t ui16Size)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((ui16Write >= ui16Read) ? (ui16Write - ui16Read) :
           (ui16Size - (ui16Read - ui16Write)));
}

//*****************************************************************************
//
//! Adding +1 to the index
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is use to advance the index by 1, if the index
//! already hit the uart size then it will reset back to 0.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************

static uint16_t advance_buffer_index(volatile uint16_t* pui16Index, uint16_t ui16Size)
{
    *pui16Index = (*pui16Index + 1) % ui16Size;

    return(*pui16Index);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
