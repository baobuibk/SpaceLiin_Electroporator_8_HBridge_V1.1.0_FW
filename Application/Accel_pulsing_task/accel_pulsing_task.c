/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "app.h"
#include "stdbool.h"

#include "accel_pulsing_task.h"

#include "crc.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define AUTO_PULSE_TIMEOUT 500

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum _auto_pulsing_state_typedef_
{
	DISABLE_AUTO_PULSING = 0,
	PREPARE_SEND_TO_SENSOR,
	SEND_RESQUEST_SENSOR,
	SENSOR_ERROR,
	SYSTERM_PULSING_RUN,
	SYSTERM_PULSING_DONE,

} auto_pulsing_state_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static auto_pulsing_state_typedef auto_pulsing_state = DISABLE_AUTO_PULSING;

static bool    is_accel_pulsing_disable = true;

static uint8_t print_tick = 10;
static uint8_t get_sensor_timeout = AUTO_PULSE_TIMEOUT;

static uint8_t frame_count = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void fsp_print(uint8_t packet_length);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
LSM6DSOX_data_typedef Threshold_Accel = { 0, 0, 2000 };
LSM6DSOX_data_typedef Auto_Accel_Data;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void Accel_Pulsing_Task(void*)
{

	if (is_accel_pulsing_disable)
	{
		SchedulerTaskDisable(6);
		auto_pulsing_state = DISABLE_AUTO_PULSING;
		return;
	}

	switch (auto_pulsing_state)
	{
	case PREPARE_SEND_TO_SENSOR:
	{
		// move up 1 line, turn off console
		// UART_Send_String(&RS232_UART, "\033[A\033[?25l");
		// UART_Send_String(&RS232_UART, "\rAccel is: X:       Y:       Z:");

		Is_Sensor_Read_Complete();
		Sensor_Read_Value(SENSOR_READ_ACCEL);
		
		auto_pulsing_state = SEND_RESQUEST_SENSOR;
		get_sensor_timeout = AUTO_PULSE_TIMEOUT;
		frame_count = 0;
		break;
	}

	case SEND_RESQUEST_SENSOR:
	{
		if (Is_Sensor_Read_Complete() == true)
		{
			Auto_Accel_Data = Sensor_Accel;

			ps_FSP_TX->CMD  = FSP_CMD_STREAM_ACCEL;
			// Ghi byte thấp
			ps_FSP_TX->Payload.stream_accel.XL = Auto_Accel_Data.x & 0xFF;       // Byte thấp của x
			ps_FSP_TX->Payload.stream_accel.YL = Auto_Accel_Data.y & 0xFF;       // Byte thấp của y
			ps_FSP_TX->Payload.stream_accel.ZL = Auto_Accel_Data.z & 0xFF;       // Byte thấp của z

			// Ghi byte cao
			ps_FSP_TX->Payload.stream_accel.XH = (Auto_Accel_Data.x >> 8) & 0xFF; // Byte cao của x
			ps_FSP_TX->Payload.stream_accel.YH = (Auto_Accel_Data.y >> 8) & 0xFF; // Byte cao của y
			ps_FSP_TX->Payload.stream_accel.ZH = (Auto_Accel_Data.z >> 8) & 0xFF; // Byte cao của z

			ps_FSP_TX->Payload.stream_accel.count = frame_count++;

			if(frame_count == 251)
			{
				frame_count = 1;
			}	
				
			fsp_print(8);

			if (Auto_Accel_Data.z >= Threshold_Accel.z)
			{
				//start pulsing
				auto_pulsing_state = SYSTERM_PULSING_RUN;

				//move console to begin of line then print
			// 	UART_Printf
			// 	(
			// 		&RS232_UART,
			// 		"\rAccel threshold at: X:%d Y:%d Z:%d",
			// 		Auto_Accel_Data.x,
			// 		Auto_Accel_Data.y,
			// 		Auto_Accel_Data.z
			// 	);
			}
			else
			{
			// 	if (print_tick > 0)
			// 	{
			// 		print_tick--;
			// 	}
			// 	else
			// 	{
			// 		print_tick = 10;

			// 		//move console to x:11 then print the value
			// 		UART_Printf
			// 		(
			// 			&RS232_UART,
			// 			"\033[13G%5d\033[22G%5d\033[31G%5d",
			// 			Auto_Accel_Data.x,
			// 			Auto_Accel_Data.y,
			// 			Auto_Accel_Data.z
			// 		);
			// 	}

				Sensor_Read_Value(SENSOR_READ_ACCEL);
				auto_pulsing_state = SEND_RESQUEST_SENSOR;
				get_sensor_timeout = AUTO_PULSE_TIMEOUT;
			}
		}
		else
		{
			if (get_sensor_timeout > 0)
			{
				get_sensor_timeout--;
			}
			else
			{
				auto_pulsing_state = SENSOR_ERROR;
			}
		}

		break;
	}

	case SYSTERM_PULSING_RUN:
	{
		if (is_h_bridge_enable == true)
		{
			return;
		}
			
		is_h_bridge_enable = true;
		auto_pulsing_state = SYSTERM_PULSING_DONE;
		SchedulerTaskEnable(0, 1);

		break;
	}
		
	case SYSTERM_PULSING_DONE:
	{
		if (is_h_bridge_enable == true)
		{
			return;
		}

		Sensor_Read_Value(SENSOR_READ_ACCEL);

		auto_pulsing_state = SEND_RESQUEST_SENSOR;

		//clear present line
		// UART_Send_String(&RS232_UART, "\033[2K");

		//refresh for next callback
		// UART_Send_String(&RS232_UART, "\rAccel is: X:       Y:       Z:");

		get_sensor_timeout = AUTO_PULSE_TIMEOUT;
		break;
	}

	case SENSOR_ERROR:
	{
		break;
	}

	case DISABLE_AUTO_PULSING:
	default:
		is_accel_pulsing_disable = true;
	}
}

void Enable_Auto_Pulsing()
{
	if (is_accel_pulsing_disable == true)
	{
		auto_pulsing_state = PREPARE_SEND_TO_SENSOR;
		is_accel_pulsing_disable = false;
		SchedulerTaskEnable(6, 0);
		return;
	}
}

void Disable_Auto_Pulsing()
{
	is_accel_pulsing_disable = true;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void fsp_print(uint8_t packet_length)
{
	s_FSP_TX_Packet.sod 	= FSP_PKT_SOD;
	s_FSP_TX_Packet.src_adr = fsp_my_adr;
	s_FSP_TX_Packet.dst_adr = FSP_ADR_GPC;
	s_FSP_TX_Packet.length 	= packet_length;
	s_FSP_TX_Packet.type 	= FSP_PKT_TYPE_CMD_W_DATA;
	s_FSP_TX_Packet.eof 	= FSP_PKT_EOF;
	s_FSP_TX_Packet.crc16 	= crc16_CCITT(FSP_CRC16_INITIAL_VALUE, &s_FSP_TX_Packet.src_adr, s_FSP_TX_Packet.length + 4);

	uint8_t encoded_frame[100] = { 0 };
	uint8_t frame_len;
	fsp_encode(&s_FSP_TX_Packet, encoded_frame, &frame_len);

	UART_FSP(&GPC_UART, (char*) encoded_frame, frame_len);
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */