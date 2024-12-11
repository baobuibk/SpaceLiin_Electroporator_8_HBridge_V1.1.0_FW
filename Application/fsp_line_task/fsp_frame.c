/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <stdlib.h>

#include "app.h"

#include "fsp_frame.h"
#include "crc.h"
//#include "pwm.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
//extern Accel_Gyro_DataTypedef _gyro, _accel;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void fsp_print(uint8_t packet_length);
//static void convertTemperature(float temp, uint8_t buf[]);
//static void convertIntegerToBytes(int number, uint8_t arr[]);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern uint8_t CMD_sequence_index;
extern uint8_t CMD_total_sequence_index;

/*
extern double   compensated_pressure;
extern double   compensated_temperature;

float           temp;
uint32_t        press;
*/

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uint8_t hs_relay_pole, ls_relay_pole, relay_state;
void FSP_Line_Process()
{
switch (ps_FSP_RX->CMD)
{

/* :::::::::: Pulse Control Command :::::::: */
case FSP_CMD_SET_SEQUENCE_INDEX:
{
	CMD_sequence_index 	= ps_FSP_RX->Payload.set_sequence_index.index - 1;

	if (CMD_total_sequence_index < CMD_sequence_index)
	{
		CMD_total_sequence_index = CMD_sequence_index;
	}

	UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_SEQUENCE_INDEX\r\n> ");
	break;
}

case FSP_CMD_SET_SEQUENCE_DELETE:
{
	HB_sequence_array[(CMD_sequence_index)].is_setted &= ~(1 << 7);
	CMD_total_sequence_index -= 1;
	CMD_sequence_index = CMD_total_sequence_index;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_SEQUENCE_DELETE\r\n> ");
	break;
}

case FSP_CMD_SET_SEQUENCE_CONFIRM:
{
	HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 7);

	UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_SEQUENCE_CONFIRM\r\n> ");
	break;
}

case FSP_CMD_SET_SEQUENCE_DELAY:
{
	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 0)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 0);
	}

	HB_sequence_array[CMD_sequence_index].sequence_delay_ms = ps_FSP_RX->Payload.set_sequence_delay.Delay_high;
	HB_sequence_array[CMD_sequence_index].sequence_delay_ms <<= 8;
	HB_sequence_array[CMD_sequence_index].sequence_delay_ms |= ps_FSP_RX->Payload.set_sequence_delay.Delay_low;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_SEQUENCE_DELAY\r\n> ");
	break;
}

case FSP_CMD_SET_PULSE_POLE:
{
	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 1)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 1);
	}

	HB_sequence_array[CMD_sequence_index].pos_pole_index 	= ps_FSP_RX->Payload.set_pulse_pole.pos_pole;

	HB_sequence_array[CMD_sequence_index].neg_pole_index 	= ps_FSP_RX->Payload.set_pulse_pole.neg_pole;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_PULSE_POLE\r\n> ");
	break;
}

case FSP_CMD_SET_PULSE_COUNT:
{
	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 2)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 2);
	}

	HB_sequence_array[CMD_sequence_index].hv_pos_count 	= ps_FSP_RX->Payload.set_pulse_count.HV_pos_count;
	HB_sequence_array[CMD_sequence_index].hv_neg_count 	= ps_FSP_RX->Payload.set_pulse_count.HV_neg_count;

	HB_sequence_array[CMD_sequence_index].lv_pos_count 	= ps_FSP_RX->Payload.set_pulse_count.LV_pos_count;
	HB_sequence_array[CMD_sequence_index].lv_neg_count 	= ps_FSP_RX->Payload.set_pulse_count.LV_neg_count;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_COUNT\r\n> ");
	break;
}
	

case FSP_CMD_SET_PULSE_DELAY:
{
	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 3)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 3);
	}

	HB_sequence_array[CMD_sequence_index].hv_delay_ms = ps_FSP_RX->Payload.set_pulse_delay.HV_delay;
	HB_sequence_array[CMD_sequence_index].lv_delay_ms	= ps_FSP_RX->Payload.set_pulse_delay.LV_delay;

	HB_sequence_array[CMD_sequence_index].pulse_delay_ms = ps_FSP_RX->Payload.set_pulse_delay.Delay_high;
	HB_sequence_array[CMD_sequence_index].pulse_delay_ms <<= 8;
	HB_sequence_array[CMD_sequence_index].pulse_delay_ms |= ps_FSP_RX->Payload.set_pulse_delay.Delay_low;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_DELAY\r\n> ");
	break;
}
	

case FSP_CMD_SET_PULSE_HV:
{
	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 4)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 4);
	}

	HB_sequence_array[CMD_sequence_index].hv_on_ms 	= ps_FSP_RX->Payload.set_pulse_HV.OnTime;
	HB_sequence_array[CMD_sequence_index].hv_off_ms 	= ps_FSP_RX->Payload.set_pulse_HV.OffTime;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_HV\r\n> ");
	break;
}
	
case FSP_CMD_SET_PULSE_LV:
{
	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 5)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 5);
	}

	HB_sequence_array[CMD_sequence_index].lv_on_ms 	= ps_FSP_RX->Payload.set_pulse_LV.OnTime_high;
	HB_sequence_array[CMD_sequence_index].lv_on_ms   <<= 8;
	HB_sequence_array[CMD_sequence_index].lv_on_ms	|= ps_FSP_RX->Payload.set_pulse_LV.OnTime_low;

	HB_sequence_array[CMD_sequence_index].lv_off_ms	= ps_FSP_RX->Payload.set_pulse_LV.OffTime_high;
	HB_sequence_array[CMD_sequence_index].lv_off_ms	<<= 8;
	HB_sequence_array[CMD_sequence_index].lv_off_ms	|= ps_FSP_RX->Payload.set_pulse_LV.OffTime_low;

	UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_LV\r\n> ");
	break;
}
	
case FSP_CMD_SET_PULSE_CONTROL:
{
	//H_Bridge_Set_Pole(3, 8);
	is_h_bridge_enable = ps_FSP_RX->Payload.set_pulse_control.State;
	SchedulerTaskEnable(0, 1);

	UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_CONTROL\r\n> ");
	break;
}


/* :::::::::: VOM Command :::::::: */
case FSP_CMD_MEASURE_IMPEDANCE:
{
	is_Measure_Impedance 	= true;

    Current_Sense_Period	= ps_FSP_RX->Payload.measure_impedance.Period_high;
	Current_Sense_Period	= Current_Sense_Period << 8;
	Current_Sense_Period	|= ps_FSP_RX->Payload.measure_impedance.Period_low;
	
    is_h_bridge_enable 		= false;

	H_Bridge_Set_Pole(3, 8);
    V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);
    H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
    H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_HS_ON);

    LL_ADC_REG_StartConversionSWStart(ADC_I_SENSE_HANDLE);
    SchedulerTaskEnable(3, 1);

    UART_Send_String(&RS232_UART, "Received FSP_CMD_GET_IMPEDANCE\r\n> ");
    break;
}


/* :::::::::: Ultility Command :::::::: */
case FSP_CMD_HANDSHAKE:
{
	ps_FSP_TX->CMD = FSP_CMD_HANDSHAKE;
	ps_FSP_TX->Payload.handshake.Check = 0xAB;
	
	fsp_print(2);
	
	UART_Send_String(&RS232_UART, "Received FSP_CMD_HANDSHAKE\r\n> ");
	break;
}
	
default:
	break;
}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void fsp_print(uint8_t packet_length)
{
	s_FSP_TX_Packet.sod 		= FSP_PKT_SOD;
	s_FSP_TX_Packet.src_adr 	= fsp_my_adr;
	s_FSP_TX_Packet.dst_adr 	= FSP_ADR_GPC;
	s_FSP_TX_Packet.length 		= packet_length;
	s_FSP_TX_Packet.type 		= FSP_PKT_TYPE_CMD_W_DATA;
	s_FSP_TX_Packet.eof 		= FSP_PKT_EOF;
	s_FSP_TX_Packet.crc16 		= crc16_CCITT(FSP_CRC16_INITIAL_VALUE, &s_FSP_TX_Packet.src_adr, s_FSP_TX_Packet.length + 4);

	uint8_t encoded_frame[100] = { 0 };
	uint8_t frame_len;
	fsp_encode(&s_FSP_TX_Packet, encoded_frame, &frame_len);

	UART_FSP(&GPC_UART, (char*)encoded_frame, frame_len);
}

/*
static void convertTemperature(float temp, uint8_t buf[]) {
	// temperature is xxx.x format
	//float to byte

	gcvt(temp, 5, buf);
}

static void convertIntegerToBytes(int number, uint8_t arr[]) {
	arr[0]= number & 0xff;
	arr[1]= (number >>8 ) & 0xff;
	arr[2] = (number >>16) & 0xff;
	arr[3] = (number >>24) & 0xff;
}
*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
