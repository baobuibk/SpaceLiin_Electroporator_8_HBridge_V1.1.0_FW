/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdlib.h>
#include <stdio.h>

#include "stm32f4xx_ll_gpio.h"

#include "app.h"
#include "command.h"
#include "sensor_driver.h"
#include "cmd_line.h"
#include "sensor_task.h"
//#include "pwm.h"
//#include "fsp.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static const char * Error_Sequence_Code[5] =
{
    "> PULSE_POLE_IS_NOT_SETTED\n",
    "> PULSE_COUNT_IS_NOT_SETTED\n",
    "> PULSE_DELAY_IS_NOT_SETTED\n",
    "> PULSE_HV_IS_NOT_SETTED\n",
    "> PULSE_LV_IS_NOT_SETTED\n",
};

static uint8_t CMD_process_state = 0;

static uint8_t ChannelMapping[8] = {2, 4, 7, 6, 5, 3, 0, 1};
static uint8_t User_Channel_Mapping[8] = {7, 8, 1, 6, 2, 5, 4, 3};



/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void 	double_to_string(double value, char *buffer, uint8_t precision);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
tCmdLineEntry g_psCmdTable[] =
{
    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Pulse Control Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	{ "SET_SEQUENCE_INDEX",		CMD_SET_SEQUENCE_INDEX, 	" : Set current HB sequence index" },
	{ "SET_SEQUENCE_DELETE",	CMD_SET_SEQUENCE_DELETE, 	" : Delete a HB sequence index" },
	{ "SET_SEQUENCE_CONFIRM",	CMD_SET_SEQUENCE_CONFIRM, 	" : Confirm the setting of the current sequence index" },
	{ "SET_SEQUENCE_DELAY",		CMD_SET_SEQUENCE_DELAY, 	" : Set current HB index delay" },

	{ "SET_PULSE_POLE",			CMD_SET_PULSE_POLE, 		" : Set pole for H Bridge Pole" },
    { "SET_PULSE_COUNT",		CMD_SET_PULSE_COUNT, 		" : Set number of pulse" },
    { "SET_PULSE_DELAY",		CMD_SET_PULSE_DELAY, 		" : Set delay between pulse hv and lv" },
    { "SET_PULSE_HV", 			CMD_SET_PULSE_HV, 			" : Set hs pulse on time and off time" },
    { "SET_PULSE_LV", 			CMD_SET_PULSE_LV, 			" : Set ls pulse on time and off time" },
    { "SET_PULSE_CONTROL", 		CMD_SET_PULSE_CONTROL, 		" : Start pulsing" },

	{ "GET_SEQUENCE_INDEX",		CMD_GET_SEQUENCE_INDEX, 	" : Get current sequence index" },
	{ "GET_SEQUENCE_DELAY",		CMD_GET_SEQUENCE_DELAY, 	" : Get current sequence delay" },
	{ "GET_SEQUENCE_ALL",		CMD_GET_SEQUENCE_ALL, 		" : Get all info about sequence" },

	{ "GET_PULSE_POLE",			CMD_GET_PULSE_POLE, 		" : Set pole for H Bridge Pole" },
	{ "GET_PULSE_COUNT",		CMD_GET_PULSE_COUNT, 		" : Get number of pulse" },
	{ "GET_PULSE_DELAY",		CMD_GET_PULSE_DELAY, 		" : Get delay between pulse hv and lv" },
	{ "GET_PULSE_HV", 			CMD_GET_PULSE_HV, 			" : Get hs pulse on time and off time" },
	{ "GET_PULSE_LV", 			CMD_GET_PULSE_LV, 			" : Get ls pulse on time and off time" },
	{ "GET_PULSE_CONTROL", 		CMD_GET_PULSE_CONTROL, 		" : Get info whether pulse starting pulsing" },
	{ "GET_PULSE_ALL", 			CMD_GET_PULSE_ALL, 			" : Get all info about pulse" },

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ VOM Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	{ "MEASURE_IMPEDANCE", 		CMD_MEASURE_IMPEDANCE,		" : Measure cuvette impedance"},

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ I2C Sensor Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	{ "GET_SENSOR_GYRO", 		CMD_GET_SENSOR_GYRO, 		" : Get gyro" },
	{ "GET_SENSOR_ACCEL", 		CMD_GET_SENSOR_ACCEL, 		" : Get accel" },
	{ "GET_SENSOR_LSM6DSOX", 	CMD_GET_SENSOR_LSM6DSOX, 	" : Get accel and gyro" },

	{ "GET_SENSOR_TEMP", 		CMD_GET_SENSOR_TEMP, 		" : Get temp" },
	{ "GET_SENSOR_PRESSURE", 	CMD_GET_SENSOR_PRESSURE, 	" : Get pressure" },
	{ "GET_SENSOR_ALTITUDE", 	CMD_GET_SENSOR_ALTITUDE, 	" : Get altitude" },
	{ "GET_SENSOR_BMP390", 		CMD_GET_SENSOR_BMP390, 		" : Get temp, pressure and altitude" },


	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Auto Pulsing Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	{ "SET_THRESHOLD_ACCEL",	CMD_SET_THRESHOLD_ACCEL,	" : Set high threshold accel for auto pulsing" },
	{ "GET_THRESHOLD_ACCEL",	CMD_GET_THRESHOLD_ACCEL, 	" : Get high threshold accel for auto pulsing" },
	{ "SET_AUTO_ACCEL",			CMD_SET_AUTO_ACCEL,			" : Enable auto pulsing" },
	{ "CALIB_ACCEL",			CMD_CALIB_ACCEL,			" : Enable auto accel calib" },
    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Ultility Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    { "CLC",           			CMD_CLEAR_SCREEN,              " " },


	{0,0,0}
};

uint8_t CMD_sequence_index = 0;
uint8_t CMD_total_sequence_index = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: Pulse Control Command :::::::: */
int CMD_SET_SEQUENCE_INDEX(int argc, char *argv[])
{
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm;

	receive_argm = atoi(argv[1]);

	if (receive_argm > 10)
		return CMDLINE_INVALID_ARG;
	if (receive_argm < 1)
		return CMDLINE_INVALID_ARG;

	if ((receive_argm - 1) > CMD_total_sequence_index + 1)
	{
		UART_Printf(&RS232_UART, "> ERROR YOUR NEXT SEQUENCE INDEX IS: %d, NOT %d\n", CMD_total_sequence_index + 2, receive_argm);

		UART_Printf(&RS232_UART, "> CURRENT SEQUENCE INDEX: %d\n", CMD_sequence_index + 1);
		return CMDLINE_OK;
	}

	if (((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 7)) == 0) && ((receive_argm - 1) != CMD_sequence_index))
	{
		UART_Printf(&RS232_UART, "> ERROR CURRENT SEQUENCE INDEX: %d IS NOT CONFIRMED\n", CMD_sequence_index + 1);

		UART_Send_String(&RS232_UART, "> EITHER CONFIRM IT OR DELETE IT\n");
		
		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> DELAY BETWEEN SEQUENCE: %dms\n", HB_sequence_array[CMD_sequence_index].sequence_delay_ms);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> POS HV PULSE COUNT: %d; NEG HV PULSE COUNT: %d\n",
		HB_sequence_array[CMD_sequence_index].hv_pos_count, HB_sequence_array[CMD_sequence_index].hv_neg_count);
		UART_Printf(&RS232_UART, "> POS LV PULSE COUNT: %d; NEG LV PULSE COUNT: %d\n",
		HB_sequence_array[CMD_sequence_index].lv_pos_count, HB_sequence_array[CMD_sequence_index].lv_neg_count);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> DELAY BETWEEN HV POS AND NEG PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].hv_delay_ms);
		UART_Printf(&RS232_UART, "> DELAY BETWEEN LV POS AND NEG PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].lv_delay_ms);
		UART_Printf(&RS232_UART, "> DELAY BETWEEN HV PULSE AND LV PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].pulse_delay_ms);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> HV PULSE ON TIME: %dms; HV PULSE OFF TIME: %dms\n",
		HB_sequence_array[CMD_sequence_index].hv_on_ms, HB_sequence_array[CMD_sequence_index].hv_off_ms);
		UART_Printf(&RS232_UART, "> LV PULSE ON TIME: %dms; LV PULSE OFF TIME: %dms\n",
		HB_sequence_array[CMD_sequence_index].lv_on_ms, HB_sequence_array[CMD_sequence_index].lv_off_ms);

		UART_Send_String(&RS232_UART, "> \n");
		return CMDLINE_OK;
	}

	UART_Send_String(&RS232_UART, "> \n");

	UART_Printf(&RS232_UART, "> CURRENT SEQUENCE INDEX: %d\n", receive_argm);
	
	UART_Send_String(&RS232_UART, "> \n");

	CMD_sequence_index = receive_argm - 1;

	if (CMD_total_sequence_index < CMD_sequence_index)
	{
		CMD_total_sequence_index = CMD_sequence_index;
	}

	return CMDLINE_OK;
}

int CMD_SET_SEQUENCE_DELETE(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	/* int receive_argm;

	receive_argm = atoi(argv[1]);

	if (receive_argm > 10)
		return CMDLINE_INVALID_ARG;
	if (receive_argm < 1)
		return CMDLINE_INVALID_ARG;
	
	if ((receive_argm - 1) > CMD_total_sequence_index + 1)
	{
		UART_Printf(&RS232_UART, "> ERROR SEQUENCE INDEX: %d IS OUT-OF-BOUND\n", receive_argm);

		UART_Printf(&RS232_UART, "> TOTAL SEQUENCE INDEX: %d\n", CMD_total_sequence_index + 1);
		return CMDLINE_OK;
	} */

	if (CMD_sequence_index < CMD_total_sequence_index)
	{
		UART_Send_String(&RS232_UART, "> ERROR CANNOT DELETE IN-BETWEEN SEQUENCE\n");
		return CMDLINE_OK;
	}
	
	if (CMD_sequence_index == 0)
	{
		UART_Send_String(&RS232_UART, "> ERROR CANNOT DELETE ANYMORE\n");

		UART_Printf(&RS232_UART, "> TOTAL SEQUENCE: %d\n", CMD_total_sequence_index + 1);
		return CMDLINE_OK;
	}

	HB_sequence_array[(CMD_sequence_index)].is_setted &= ~(1 << 7);
	CMD_total_sequence_index -= 1;
	CMD_sequence_index = CMD_total_sequence_index;
	UART_Printf(&RS232_UART, "> CURRENT SEQUENCE INDEX: %d\n", CMD_sequence_index + 1);

	return CMDLINE_OK;
}

int CMD_SET_SEQUENCE_CONFIRM(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

//	int receive_argm;
//
//	receive_argm = atoi(argv[1]);
//
//	if (receive_argm > 1)
//		return CMDLINE_INVALID_ARG;
//	if (receive_argm < 1)
//		return CMDLINE_INVALID_ARG;

	HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 7);

	return CMDLINE_OK;
}

int CMD_SET_SEQUENCE_DELAY(int argc, char *argv[])
{
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm;

	receive_argm = atoi(argv[1]);

	if (receive_argm > 100)
		return CMDLINE_INVALID_ARG;
	if (receive_argm < 1)
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 0)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 0);
		//CMD_total_sequence_index = CMD_sequence_index;
	}
	
	HB_sequence_array[CMD_sequence_index].sequence_delay_ms = receive_argm;
	
	return CMDLINE_OK;
}

int CMD_SET_PULSE_POLE(int argc, char *argv[])
{
	if (argc < 3)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 3)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm[2];

	receive_argm[0] = atoi(argv[1]);
	receive_argm[1] = atoi(argv[2]);

	if (receive_argm[0] == receive_argm[1])
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[0] > 8) || (receive_argm[0] < 1) || (receive_argm[0] == 9))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[1] > 8) || (receive_argm[1] < 1) || (receive_argm[1] == 9))
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 1)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 1);
	}
	
	HB_sequence_array[CMD_sequence_index].pos_pole_index = ChannelMapping[receive_argm[0] - 1];
	HB_sequence_array[CMD_sequence_index].neg_pole_index = ChannelMapping[receive_argm[1] - 1];

	return CMDLINE_OK;
}

int CMD_SET_PULSE_COUNT(int argc, char *argv[])
{
	if (argc < 5)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 5)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm[4];

	receive_argm[0] = atoi(argv[1]);
	receive_argm[1] = atoi(argv[2]);
	receive_argm[2] = atoi(argv[3]);
	receive_argm[3] = atoi(argv[4]);

	if ((receive_argm[0] > 20) || (receive_argm[1] > 20) || (receive_argm[2] > 20) || (receive_argm[3] > 20))
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 2)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 2);
	}

	HB_sequence_array[CMD_sequence_index].hv_pos_count 	= receive_argm[0];
    HB_sequence_array[CMD_sequence_index].hv_neg_count 	= receive_argm[1];

    HB_sequence_array[CMD_sequence_index].lv_pos_count 	= receive_argm[2];
    HB_sequence_array[CMD_sequence_index].lv_neg_count 	= receive_argm[3];

	return CMDLINE_OK;
}

int CMD_SET_PULSE_DELAY(int argc, char *argv[])
{
	if (argc < 4)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 4)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm[3];

	receive_argm[0] = atoi(argv[1]);
	receive_argm[1] = atoi(argv[2]);
	receive_argm[2] = atoi(argv[3]);

	if ((receive_argm[0] > 100) || (receive_argm[0] < 1))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[1] > 100) || (receive_argm[1] < 1))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[2] > 1000) || (receive_argm[2] < 1))
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 3)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 3);
	}

	HB_sequence_array[CMD_sequence_index].hv_delay_ms = receive_argm[0];
	HB_sequence_array[CMD_sequence_index].lv_delay_ms	= receive_argm[1];

    HB_sequence_array[CMD_sequence_index].pulse_delay_ms = receive_argm[2];

	return CMDLINE_OK;
}

int CMD_SET_PULSE_HV(int argc, char *argv[])
{
	if (argc < 3)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 3)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm[2];

	receive_argm[0] = atoi(argv[1]);
	receive_argm[1] = atoi(argv[2]);

	if ((receive_argm[0] > 20) || (receive_argm[0] < 1))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[1] > 20) || (receive_argm[1] < 1))
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 4)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 4);
	}

	HB_sequence_array[CMD_sequence_index].hv_on_ms   = receive_argm[0];
	HB_sequence_array[CMD_sequence_index].hv_off_ms  = receive_argm[1];

	return CMDLINE_OK;
}

int CMD_SET_PULSE_LV(int argc, char *argv[])
{
	if (argc < 3)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 3)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm[2];

	receive_argm[0] = atoi(argv[1]);
	receive_argm[1] = atoi(argv[2]);

	if ((receive_argm[0] > 1000) || (receive_argm[0] < 1))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[1] > 1000) || (receive_argm[1] < 1))
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 5)) == false)
	{
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 5);
	}

	HB_sequence_array[CMD_sequence_index].lv_on_ms 	= receive_argm[0];
    HB_sequence_array[CMD_sequence_index].lv_off_ms	= receive_argm[1];

	return CMDLINE_OK;
}

int CMD_SET_PULSE_CONTROL(int argc, char *argv[])
{
	int8_t receive_argm = atoi(argv[1]);

	if ((receive_argm > 1) || (receive_argm < 0))
		return CMDLINE_INVALID_ARG;

	if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 7)) == 0)
	{
		UART_Printf(&RS232_UART, "> ERROR CURRENT SEQUENCE INDEX: %d IS NOT CONFIRMED\n", CMD_sequence_index + 1);

		UART_Send_String(&RS232_UART, "> EITHER CONFIRM IT OR DELETE IT\n");
		
		UART_Send_String(&RS232_UART, "> PULSE CONTROL IS DISABLED\n");
		return CMDLINE_OK;
	}

	is_h_bridge_enable = receive_argm;
	SchedulerTaskEnable(0, 1);

	return CMDLINE_OK;
}

int CMD_GET_SEQUENCE_INDEX(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> CURRENT SEQUENCE INDEX: %d\n", CMD_sequence_index + 1);

	return CMDLINE_OK;
}

int CMD_GET_SEQUENCE_DELAY(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> CURRENT SEQUENCE DELAY: %dms\n", HB_sequence_array[CMD_sequence_index].sequence_delay_ms);

	return CMDLINE_OK;
}

int CMD_GET_SEQUENCE_ALL(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	for (uint8_t i = 0; i <= CMD_total_sequence_index; i++)
	{
		UART_Printf(&RS232_UART, "> CURRENT SEQUENCE INDEX: %d\n", i + 1);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> PULSE POS POLE: %d; PULSE NEG POLE: %d\n",
		User_Channel_Mapping[HB_sequence_array[i].pos_pole_index], User_Channel_Mapping[HB_sequence_array[i].neg_pole_index]);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> DELAY BETWEEN SEQUENCE: %dms\n", HB_sequence_array[i].sequence_delay_ms);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> POS HV PULSE COUNT: %d; NEG HV PULSE COUNT: %d\n",
		HB_sequence_array[i].hv_pos_count, HB_sequence_array[i].hv_neg_count);
		UART_Printf(&RS232_UART, "> POS LV PULSE COUNT: %d; NEG LV PULSE COUNT: %d\n",
		HB_sequence_array[i].lv_pos_count, HB_sequence_array[i].lv_neg_count);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> DELAY BETWEEN HV POS AND NEG PULSE: %dms\n", HB_sequence_array[i].hv_delay_ms);
		UART_Printf(&RS232_UART, "> DELAY BETWEEN LV POS AND NEG PULSE: %dms\n", HB_sequence_array[i].lv_delay_ms);
		UART_Printf(&RS232_UART, "> DELAY BETWEEN HV PULSE AND LV PULSE: %dms\n", HB_sequence_array[i].pulse_delay_ms);

		UART_Send_String(&RS232_UART, "> \n");

		UART_Printf(&RS232_UART, "> HV PULSE ON TIME: %dms; HV PULSE OFF TIME: %dms\n",
		HB_sequence_array[i].hv_on_ms, HB_sequence_array[i].hv_off_ms);
		UART_Printf(&RS232_UART, "> LV PULSE ON TIME: %dms; LV PULSE OFF TIME: %dms\n",
		HB_sequence_array[i].lv_on_ms, HB_sequence_array[i].lv_off_ms);

		UART_Send_String(&RS232_UART, "> \n");
	}

	return CMDLINE_OK;
}

int CMD_GET_PULSE_POLE(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> PULSE POS POLE: %d; PULSE NEG POLE: %d\n", 
	HB_sequence_array[CMD_sequence_index].pos_pole_index + 1, HB_sequence_array[CMD_sequence_index].neg_pole_index + 1);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_COUNT(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> POS HV PULSE COUNT: %d; NEG HV PULSE COUNT: %d\n", 
	HB_sequence_array[CMD_sequence_index].hv_pos_count, HB_sequence_array[CMD_sequence_index].hv_neg_count);

	UART_Printf(&RS232_UART, "> POS LV PULSE COUNT: %d; NEG LV PULSE COUNT: %d\n", 
	HB_sequence_array[CMD_sequence_index].lv_pos_count, HB_sequence_array[CMD_sequence_index].lv_neg_count);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_DELAY(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV POS AND NEG PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].hv_delay_ms);

	UART_Printf(&RS232_UART, "> DELAY BETWEEN LV POS AND NEG PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].lv_delay_ms);

	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV PULSE AND LV PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].pulse_delay_ms);

	return CMDLINE_OK;		
}

int CMD_GET_PULSE_HV(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "HV PULSE ON TIME: %dms; HV PULSE OFF TIME: %dms\n", 
	HB_sequence_array[CMD_sequence_index].hv_on_ms, HB_sequence_array[CMD_sequence_index].hv_off_ms);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_LV(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> LV PULSE ON TIME: %dms; LV PULSE OFF TIME: %dms\n",
	HB_sequence_array[CMD_sequence_index].lv_on_ms, HB_sequence_array[CMD_sequence_index].lv_off_ms);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_CONTROL(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;
	/*
	if (is_h_bridge_enable == 1)
	{
		UART_Send_String(&RS232_UART, "> H BRIDGE IS PULSING\n");
	}
	else
	{
		UART_Send_String(&RS232_UART, "> H BRIDGE IS NOT PULSING\n");
	}
	*/

	UART_Printf(&RS232_UART, "> %s\n", 
	is_h_bridge_enable ? "H BRIDGE IS PULSING" : "H BRIDGE IS NOT PULSING");

	return CMDLINE_OK;
}

int CMD_GET_PULSE_ALL(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Send_String(&RS232_UART, "> \n");

	UART_Printf(&RS232_UART, "> PULSE POS POLE: %d; PULSE NEG POLE: %d\n",
	User_Channel_Mapping[HB_sequence_array[CMD_sequence_index].pos_pole_index], User_Channel_Mapping[HB_sequence_array[CMD_sequence_index].neg_pole_index]);

	UART_Send_String(&RS232_UART, "> \n");

	UART_Printf(&RS232_UART, "> DELAY BETWEEN SEQUENCE: %dms\n", HB_sequence_array[CMD_sequence_index].sequence_delay_ms);

	UART_Send_String(&RS232_UART, "> \n");

	UART_Printf(&RS232_UART, "> POS HV PULSE COUNT: %d; NEG HV PULSE COUNT: %d\n",
	HB_sequence_array[CMD_sequence_index].hv_pos_count, HB_sequence_array[CMD_sequence_index].hv_neg_count);
	UART_Printf(&RS232_UART, "> POS LV PULSE COUNT: %d; NEG LV PULSE COUNT: %d\n",
	HB_sequence_array[CMD_sequence_index].lv_pos_count, HB_sequence_array[CMD_sequence_index].lv_neg_count);

	UART_Send_String(&RS232_UART, "> \n");

	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV POS AND NEG PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].hv_delay_ms);
	UART_Printf(&RS232_UART, "> DELAY BETWEEN LV POS AND NEG PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].lv_delay_ms);
	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV PULSE AND LV PULSE: %dms\n", HB_sequence_array[CMD_sequence_index].pulse_delay_ms);

	UART_Send_String(&RS232_UART, "> \n");

	UART_Printf(&RS232_UART, "> HV PULSE ON TIME: %dms; HV PULSE OFF TIME: %dms\n",
	HB_sequence_array[CMD_sequence_index].hv_on_ms, HB_sequence_array[CMD_sequence_index].hv_off_ms);
	UART_Printf(&RS232_UART, "> LV PULSE ON TIME: %dms; LV PULSE OFF TIME: %dms\n",
	HB_sequence_array[CMD_sequence_index].lv_on_ms, HB_sequence_array[CMD_sequence_index].lv_off_ms);

	UART_Send_String(&RS232_UART, "> \n");

	return CMDLINE_OK;
}

/* :::::::::: VOM Command :::::::: */
int CMD_MEASURE_IMPEDANCE(int argc, char *argv[])
{
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;
	
	is_Measure_Impedance 	= true;

    Current_Sense_Period	= atoi(argv[1]);
	
    is_h_bridge_enable 		= false;

	H_Bridge_Set_Pole(3, 8);
    V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);
    H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
    H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_HS_ON);

    LL_ADC_REG_StartConversionSWStart(ADC_I_SENSE_HANDLE);
    SchedulerTaskEnable(3, 1);

	return CMDLINE_OK;
}

/* :::::::::: I2C Sensor Command :::::::: */
int CMD_GET_SENSOR_GYRO(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_GYRO);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;
	
	UART_Printf(&RS232_UART, "> GYRO x: %dmpds; GYRO y: %dmpds; GYRO z: %dmpds\n", Sensor_Gyro.x, Sensor_Gyro.y, Sensor_Gyro.z);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

int CMD_GET_SENSOR_ACCEL(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_ACCEL);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;
	
	UART_Printf(&RS232_UART, "> ACCEL x: %dmm/s2; ACCEL y: %dmm/s2; ACCEL z: %dmm/s2\n", Sensor_Accel.x, Sensor_Accel.y, Sensor_Accel.z);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

int CMD_GET_SENSOR_LSM6DSOX(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_LSM6DSOX);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;
	
	UART_Printf(&RS232_UART, "> GYRO x: %dmpds; GYRO y: %dmpds; GYRO z: %dmpds\n", Sensor_Gyro.x, Sensor_Gyro.y, Sensor_Gyro.z);
	UART_Printf(&RS232_UART, "> ACCEL x: %dmm/s2; ACCEL y: %dmm/s2; ACCEL z: %dmm/s2\n", Sensor_Accel.x, Sensor_Accel.y, Sensor_Accel.z);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

int CMD_GET_SENSOR_TEMP(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_TEMP);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;
	
	char fractional_string[16] = {0};
	double_to_string(Sensor_Temp, fractional_string, 3);

	UART_Printf(&RS232_UART, "> TEMPERATURE: %s Celsius\n", fractional_string);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

int CMD_GET_SENSOR_PRESSURE(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_PRESSURE);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;

	char fractional_string[16] = {0};
	double_to_string(Sensor_Pressure, fractional_string, 3);
	
	UART_Printf(&RS232_UART, "> PRESSURE: %s Pa\n", fractional_string);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

int CMD_GET_SENSOR_ALTITUDE(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_ACCEL);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;
	
	char fractional_string[16] = {0};
	double_to_string(Sensor_Altitude, fractional_string, 3);

	UART_Printf(&RS232_UART, "> ALTITUDE: %s m\n", fractional_string);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

int CMD_GET_SENSOR_BMP390(int argc, char *argv[])
{
switch (CMD_process_state)
{
case 0:
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	Sensor_Read_Value(SENSOR_READ_BMP390);
	CMD_process_state = 1;
	return CMDLINE_IS_PROCESSING;
}


case 1:
{
	if (is_sensor_read_finished == false)
	{
		return CMDLINE_IS_PROCESSING;
	}

	is_sensor_read_finished = false;

	char fractional_string[16] = {0};

	double_to_string(Sensor_Temp, fractional_string, 3);
	UART_Printf(&RS232_UART, "> TEMPERATURE: %s Celsius\n", fractional_string);

	double_to_string(Sensor_Pressure, fractional_string, 3);
	UART_Printf(&RS232_UART, "> PRESSURE: %s Pa\n", fractional_string);

	double_to_string(Sensor_Altitude, fractional_string, 3);
	UART_Printf(&RS232_UART, "> ALTITUDE: %s m\n", fractional_string);
	CMD_process_state = 0;
    return CMDLINE_OK;
}

default:
	break;
}
return CMDLINE_BAD_CMD;
}

/* :::::::::: Ultility Command :::::::: */
int CMD_CLEAR_SCREEN(int argc, char *argv[])
{
    UART_Send_String(&RS232_UART, "\033[2J");
    return CMDLINE_NO_RESPONSE;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void double_to_string(double value, char *buffer, uint8_t precision)
{
    // Handle negative numbers
    if (value < 0)
	{
        *buffer++ = '-';
        value = -value;
    }

    // Extract the integer part
    uint32_t integer_part  = (uint32_t)value;
    double fractional_part = value - integer_part;

    // Convert integer part to string
    sprintf(buffer, "%ld", integer_part);
    while (*buffer) buffer++; // Move pointer to the end of the integer part

    // Add decimal point
    if (precision > 0)
	{
        *buffer++ = '.';

        // Extract and convert the fractional part
        for (uint8_t i = 0; i < precision; i++)
		{
            fractional_part *= 10;
            uint8_t digit = (uint8_t)fractional_part;
            *buffer++ = '0' + digit;
            fractional_part -= digit;
        }
    }

    // Null-terminate the string
    *buffer = '\0';
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Auto Pulsing Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
int CMD_SET_THRESHOLD_ACCEL(int argc, char *argv[]) {
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;

	Threshold_Accel.z = atoi(argv[1]);
	return CMDLINE_OK;
}

int CMD_GET_THRESHOLD_ACCEL(int argc, char *argv[]) {
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;
	UART_Printf(&RS232_UART, "\rAccel threshold is: %d\r\n",Threshold_Accel.z);
	return CMDLINE_OK;
}

int CMD_SET_AUTO_ACCEL(int argc, char *argv[]) {
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;
	int8_t receive_argm = atoi(argv[1]);

	if ((receive_argm > 1) || (receive_argm < 0))
		return CMDLINE_INVALID_ARG;
	if(receive_argm)
		Enable_Auto_Pulsing();
	else
		Disable_Auto_Pulsing();
	return CMDLINE_OK;
}

int CMD_CALIB_ACCEL(int argc, char *argv[]) {
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;
	is_accel_calib = false;
	return CMDLINE_OK;
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
