/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdlib.h>
#include <stdio.h>

#include "stm32f4xx_ll_gpio.h"

#include "app.h"
#include "command.h"

#include "cmd_line.h"
//#include "pwm.h"
//#include "fsp.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint8_t CMD_process_state = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void 	double_to_string(double value, char *buffer, uint8_t precision);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
//extern uart_stdio_typedef  RS232_UART;
//extern uart_stdio_typedef  GPP_UART;

//extern Accel_Gyro_DataTypedef _gyro, _accel;
//extern PWM_TypeDef H_Bridge_1_PWM;
//extern PWM_TypeDef H_Bridge_2_PWM;

tCmdLineEntry g_psCmdTable[] =
{
	{ "SET_CB_CONTROL",			CMD_SET_CB_CONTROL, 		" : Set pole for H Bridge Pole" },
	{ "SET_G_CONTROL",			CMD_SET_G_CONTROL, 			" : Set pole for H Bridge Pole" },

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Pulse Control Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	{ "SET_PULSE_POLE",			CMD_SET_PULSE_POLE, 		" : Set pole for H Bridge Pole" },
    { "SET_PULSE_COUNT",		CMD_SET_PULSE_COUNT, 		" : Set number of pulse" },
    { "SET_PULSE_DELAY",		CMD_SET_PULSE_DELAY, 		" : Set delay between pulse hv and lv" },
    { "SET_PULSE_HV", 			CMD_SET_PULSE_HV, 			" : Set hs pulse on time and off time" },
    { "SET_PULSE_LV", 			CMD_SET_PULSE_LV, 			" : Set ls pulse on time and off time" },
    { "SET_PULSE_CONTROL", 		CMD_SET_PULSE_CONTROL, 		" : Start pulsing" },

	{ "GET_PULSE_POLE",			CMD_GET_PULSE_POLE, 		" : Set pole for H Bridge Pole" },
	{ "GET_PULSE_COUNT",		CMD_GET_PULSE_COUNT, 		" : Get number of pulse" },
	{ "GET_PULSE_DELAY",		CMD_GET_PULSE_DELAY, 		" : Get delay between pulse hv and lv" },
	{ "GET_PULSE_HV", 			CMD_GET_PULSE_HV, 			" : Get hs pulse on time and off time" },
	{ "GET_PULSE_LV", 			CMD_GET_PULSE_LV, 			" : Get ls pulse on time and off time" },
	{ "GET_PULSE_CONTROL", 		CMD_GET_PULSE_CONTROL, 		" : Get info whether pulse starting pulsing" },
	{ "GET_PULSE_ALL", 			CMD_GET_PULSE_ALL, 			" : Get all info about pulse" },

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ I2C Sensor Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
	{ "GET_SENSOR_GYRO", 		CMD_GET_SENSOR_GYRO, 		" : Get gyro" },
	{ "GET_SENSOR_ACCEL", 		CMD_GET_SENSOR_ACCEL, 		" : Get accel" },
	{ "GET_SENSOR_LSM6DSOX", 	CMD_GET_SENSOR_LSM6DSOX, 	" : Get accel and gyro" },

	{ "GET_SENSOR_TEMP", 		CMD_GET_SENSOR_TEMP, 		" : Get temp" },
	{ "GET_SENSOR_PRESSURE", 	CMD_GET_SENSOR_PRESSURE, 	" : Get pressure" },
	{ "GET_SENSOR_ALTITUDE", 	CMD_GET_SENSOR_ALTITUDE, 	" : Get altitude" },
	{ "GET_SENSOR_BMP390", 		CMD_GET_SENSOR_BMP390, 		" : Get temp, pressure and altitude" },

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Ultility Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    { "MARCO",                  CMD_LINE_TEST,              "TEST" },
	{0,0,0}
};
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
int CMD_SET_CB_CONTROL(int argc, char *argv[])
{
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm = 0;
	receive_argm = atoi(argv[1]);

	if (receive_argm == 1)
	{
		V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);
	}
	else
	{
		V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);
	}
	
	return CMDLINE_OK;
}

int CMD_SET_G_CONTROL(int argc, char *argv[])
{
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;

	int receive_argm = 0;
	receive_argm = atoi(argv[1]);

	if (receive_argm == 1)
	{
		LL_GPIO_SetOutputPin(V_Switch_HV.Port, V_Switch_HV.Pin);
	}
	else
	{
		LL_GPIO_ResetOutputPin(V_Switch_HV.Port, V_Switch_HV.Pin);
	}
	
	return CMDLINE_OK;
}

/* :::::::::: Pulse Control Command :::::::: */
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

	HB_pos_pole_index = receive_argm[0] - 1;
	HB_neg_pole_index = receive_argm[1] - 1;

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

	hv_pulse_pos_count 	= receive_argm[0];
    hv_pulse_neg_count 	= receive_argm[1];

    lv_pulse_pos_count 	= receive_argm[2];
    lv_pulse_neg_count 	= receive_argm[3];

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

	if ((receive_argm[0] > 100) || (receive_argm[0] < 0))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[1] > 100) || (receive_argm[1] < 0))
		return CMDLINE_INVALID_ARG;
	else if ((receive_argm[2] > 1000) || (receive_argm[2] < 0))
		return CMDLINE_INVALID_ARG;

	hv_delay_ms = receive_argm[0];
	lv_delay_ms	= receive_argm[1];

    pulse_delay_ms = receive_argm[2];

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

	hv_on_time_ms   = receive_argm[0];
	hv_off_time_ms  = receive_argm[1];

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

	lv_on_time_ms 	= receive_argm[0];
    lv_off_time_ms	= receive_argm[1];

	return CMDLINE_OK;
}

int CMD_SET_PULSE_CONTROL(int argc, char *argv[])
{
	if (argc < 2)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 2)
		return CMDLINE_TOO_MANY_ARGS;

	int8_t receive_argm = atoi(argv[1]);

	if ((receive_argm > 1) || (receive_argm < 0))
		return CMDLINE_INVALID_ARG;

	H_Bridge_Set_Pole();
	is_h_bridge_enable = receive_argm;
	SchedulerTaskEnable(0, 1);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_POLE(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> PULSE POS POLE: %d; PULSE NEG POLE: %d\n", 
	HB_pos_pole_index + 1, HB_neg_pole_index + 1);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_COUNT(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> POS HV PULSE COUNT: %d; NEG HV PULSE COUNT: %d\n", 
	hv_pulse_pos_count, hv_pulse_neg_count);

	UART_Printf(&RS232_UART, "> POS LV PULSE COUNT: %d; NEG LV PULSE COUNT: %d\n", 
	lv_pulse_pos_count, lv_pulse_neg_count);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_DELAY(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV POS AND NEG PULSE: %dms\n", hv_delay_ms);

	UART_Printf(&RS232_UART, "> DELAY BETWEEN LV POS AND NEG PULSE: %dms\n", lv_delay_ms);

	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV PULSE AND LV PULSE: %dms\n", pulse_delay_ms);

	return CMDLINE_OK;		
}

int CMD_GET_PULSE_HV(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "HV PULSE ON TIME: %dms; HV PULSE OFF TIME: %dms\n", 
	hv_on_time_ms, hv_off_time_ms);

	return CMDLINE_OK;
}

int CMD_GET_PULSE_LV(int argc, char *argv[])
{
	if (argc < 1)
		return CMDLINE_TOO_FEW_ARGS;
	else if (argc > 1)
		return CMDLINE_TOO_MANY_ARGS;

	UART_Printf(&RS232_UART, "> LV PULSE ON TIME: %dms; LV PULSE OFF TIME: %dms\n",
	lv_on_time_ms, lv_off_time_ms);

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

	UART_Printf(&RS232_UART, "> POS HV PULSE COUNT: %d; NEG HV PULSE COUNT: %d\n", 
	hv_pulse_pos_count, hv_pulse_neg_count);
	UART_Printf(&RS232_UART, "> POS LV PULSE COUNT: %d; NEG LV PULSE COUNT: %d\n", 
	lv_pulse_pos_count, lv_pulse_neg_count);

	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV POS AND NEG PULSE: %dms\n", hv_delay_ms);
	UART_Printf(&RS232_UART, "> DELAY BETWEEN LV POS AND NEG PULSE: %dms\n", lv_delay_ms);
	UART_Printf(&RS232_UART, "> DELAY BETWEEN HV PULSE AND LV PULSE: %dms\n", pulse_delay_ms);

	UART_Printf(&RS232_UART, "HV PULSE ON TIME: %dms; HV PULSE OFF TIME: %dms\n", 
	hv_on_time_ms, hv_off_time_ms);
	UART_Printf(&RS232_UART, "> LV PULSE ON TIME: %dms; LV PULSE OFF TIME: %dms\n",
	lv_on_time_ms, lv_off_time_ms);

	UART_Printf(&RS232_UART, "> %s\n", 
	is_h_bridge_enable ? "H BRIDGE IS PULSING" : "H BRIDGE IS NOT PULSING");

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
int CMD_LINE_TEST(int argc, char *argv[])
{
    UART_Send_String(&RS232_UART, "> POLO\n");
    return CMDLINE_OK;
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */