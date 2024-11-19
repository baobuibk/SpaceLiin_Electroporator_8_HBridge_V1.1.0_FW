/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <stdio.h>
#include "app.h"

#include "stm32f4xx_ll_i2c.h"

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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
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
bool is_sensor_read_finished = false;
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

uint8_t sensor_driver_return = 0;
void Sensor_Read_Task(void*)
{
	if (is_sensor_init == false)
	{
		is_sensor_init = Sensor_Init();
		
		if (is_sensor_init == true)
		{
			UART_Printf(&RS232_UART, "Sensor Init success\n");
			UART_Send_String(&RS232_UART, "> ");
		}
		
		return;
	}
	
	if (IS_SENSOR_TASK_BUFFER_EMPTY(&Sensor_Task) == true)
	{
		return;
	}
	
	sensor_driver_return = Sensor_Read_Driver_Process(Sensor_Task.p_buffer[Sensor_Task.read_index]);

	if (sensor_driver_return == 0)
	{
        return;
	}

    is_sensor_read_finished = true;
    sensor_driver_return = 0;

    ADVANCE_SENSOR_TASK_READ_INDEX(&Sensor_Task);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
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
