/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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
static uint8_t FSP_process_state = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static void fsp_print(uint8_t packet_length);
static void convert_Integer_To_Bytes(uint32_t number, uint8_t arr[], uint8_t size);
static void double_to_string(double value, char *buffer, uint8_t precision);

//static void convertTemperature(float temp, uint8_t buf[]);
//static void convertIntegerToBytes(int number, uint8_t arr[]);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern uint8_t  CMD_sequence_index;
extern uint8_t  CMD_total_sequence_index;

extern bool 	is_HV_volt_available;
extern uint32_t Current_HV_volt;

/*
 extern double   compensated_pressure;
 extern double   compensated_temperature;

 float           temp;
 uint32_t        press;
 */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uint8_t hs_relay_pole, ls_relay_pole, relay_state;
uint8_t FSP_Line_Process() {
	switch (ps_FSP_RX->CMD) {

	case FSP_CMD_MEASURE_VOLT: {
		Current_HV_volt =  ps_FSP_RX->Payload.measure_volt.HV_volt[3];
		Current_HV_volt <<= 8;
		Current_HV_volt |= ps_FSP_RX->Payload.measure_volt.HV_volt[2];
		Current_HV_volt <<= 8;
		Current_HV_volt |= ps_FSP_RX->Payload.measure_volt.HV_volt[1];
		Current_HV_volt <<= 8;
		Current_HV_volt |= ps_FSP_RX->Payload.measure_volt.HV_volt[0];

		is_HV_volt_available = true;

		return 1;
	}

	/* :::::::::: Pulse Control Command :::::::: */
	case FSP_CMD_SET_SEQUENCE_INDEX: {
		CMD_sequence_index = ps_FSP_RX->Payload.set_sequence_index.index - 1;

		if (CMD_total_sequence_index < CMD_sequence_index) {
			CMD_total_sequence_index = CMD_sequence_index;
		}

		UART_Send_String(&RS232_UART,
				"Received FSP_CMD_SET_SEQUENCE_INDEX\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_SEQUENCE_DELETE: {
		HB_sequence_array[(CMD_sequence_index)].is_setted &= ~(1 << 7);

		if (CMD_total_sequence_index > 0)
		{
			CMD_total_sequence_index -= 1;
		}
		
		CMD_sequence_index = CMD_total_sequence_index;

		UART_Send_String(&RS232_UART,
				"Received FSP_CMD_SET_SEQUENCE_DELETE\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_SEQUENCE_CONFIRM: {
		HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 7);

		UART_Send_String(&RS232_UART,
				"Received FSP_CMD_SET_SEQUENCE_CONFIRM\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_SEQUENCE_DELAY: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 0))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 0);
		}

		HB_sequence_array[CMD_sequence_index].sequence_delay_ms =
				ps_FSP_RX->Payload.set_sequence_delay.Delay_high;
		HB_sequence_array[CMD_sequence_index].sequence_delay_ms <<= 8;
		HB_sequence_array[CMD_sequence_index].sequence_delay_ms |=
				ps_FSP_RX->Payload.set_sequence_delay.Delay_low;

		UART_Send_String(&RS232_UART,
				"Received FSP_CMD_SET_SEQUENCE_DELAY\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_POLE: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 1))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 1);
		}

		HB_sequence_array[CMD_sequence_index].pos_pole_index =
				ps_FSP_RX->Payload.set_pulse_pole.pos_pole;

		HB_sequence_array[CMD_sequence_index].neg_pole_index =
				ps_FSP_RX->Payload.set_pulse_pole.neg_pole;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_PULSE_POLE\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_COUNT: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 2))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 2);
		}

		HB_sequence_array[CMD_sequence_index].hv_pos_count =
				ps_FSP_RX->Payload.set_pulse_count.HV_pos_count;
		HB_sequence_array[CMD_sequence_index].hv_neg_count =
				ps_FSP_RX->Payload.set_pulse_count.HV_neg_count;

		HB_sequence_array[CMD_sequence_index].lv_pos_count =
				ps_FSP_RX->Payload.set_pulse_count.LV_pos_count;
		HB_sequence_array[CMD_sequence_index].lv_neg_count =
				ps_FSP_RX->Payload.set_pulse_count.LV_neg_count;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_COUNT\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_DELAY: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 3))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 3);
		}

		HB_sequence_array[CMD_sequence_index].hv_delay_ms =
				ps_FSP_RX->Payload.set_pulse_delay.HV_delay;
		HB_sequence_array[CMD_sequence_index].lv_delay_ms =
				ps_FSP_RX->Payload.set_pulse_delay.LV_delay;

		HB_sequence_array[CMD_sequence_index].pulse_delay_ms =
				ps_FSP_RX->Payload.set_pulse_delay.Delay_high;
		HB_sequence_array[CMD_sequence_index].pulse_delay_ms <<= 8;
		HB_sequence_array[CMD_sequence_index].pulse_delay_ms |=
				ps_FSP_RX->Payload.set_pulse_delay.Delay_low;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_DELAY\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_HV_POS: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 4))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 4);
		}

		HB_sequence_array[CMD_sequence_index].hv_pos_on_ms =
				ps_FSP_RX->Payload.set_pulse_HV_pos.OnTime;
		HB_sequence_array[CMD_sequence_index].hv_pos_off_ms =
				ps_FSP_RX->Payload.set_pulse_HV_pos.OffTime;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_PULSE_HV_POS\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_HV_NEG: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 4))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 4);
		}

		HB_sequence_array[CMD_sequence_index].hv_neg_on_ms =
				ps_FSP_RX->Payload.set_pulse_HV_neg.OnTime;
		HB_sequence_array[CMD_sequence_index].hv_neg_off_ms =
				ps_FSP_RX->Payload.set_pulse_HV_neg.OffTime;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_PULSE_HV_NEG\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_LV_POS: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 5))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 5);
		}

		HB_sequence_array[CMD_sequence_index].lv_pos_on_ms =
				ps_FSP_RX->Payload.set_pulse_LV_pos.OnTime_high;
		HB_sequence_array[CMD_sequence_index].lv_pos_on_ms <<= 8;
		HB_sequence_array[CMD_sequence_index].lv_pos_on_ms |=
				ps_FSP_RX->Payload.set_pulse_LV_pos.OnTime_low;

		HB_sequence_array[CMD_sequence_index].lv_pos_off_ms =
				ps_FSP_RX->Payload.set_pulse_LV_pos.OffTime_high;
		HB_sequence_array[CMD_sequence_index].lv_pos_off_ms <<= 8;
		HB_sequence_array[CMD_sequence_index].lv_pos_off_ms |=
				ps_FSP_RX->Payload.set_pulse_LV_pos.OffTime_low;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_PULSE_LV_POS\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_LV_NEG: {
		if ((HB_sequence_array[CMD_sequence_index].is_setted & (1 << 5))
				== false) {
			HB_sequence_array[CMD_sequence_index].is_setted |= (1 << 5);
		}

		HB_sequence_array[CMD_sequence_index].lv_neg_on_ms =
				ps_FSP_RX->Payload.set_pulse_LV_neg.OnTime_high;
		HB_sequence_array[CMD_sequence_index].lv_neg_on_ms <<= 8;
		HB_sequence_array[CMD_sequence_index].lv_neg_on_ms |=
				ps_FSP_RX->Payload.set_pulse_LV_neg.OnTime_low;

		HB_sequence_array[CMD_sequence_index].lv_neg_off_ms =
				ps_FSP_RX->Payload.set_pulse_LV_neg.OffTime_high;
		HB_sequence_array[CMD_sequence_index].lv_neg_off_ms <<= 8;
		HB_sequence_array[CMD_sequence_index].lv_neg_off_ms |=
				ps_FSP_RX->Payload.set_pulse_LV_neg.OffTime_low;

		UART_Send_String(&RS232_UART, "Received FSP_CMD_SET_PULSE_LV_NEG\r\n> ");
		return 1;
	}

	case FSP_CMD_SET_PULSE_CONTROL: {
		H_Bridge_Process_Sequence_Array();
		
		is_h_bridge_enable = ps_FSP_RX->Payload.set_pulse_control.State;
		SchedulerTaskEnable(0, 1);

		UART_Send_String(&RS232_UART, "Received FSP_CMD_PULSE_CONTROL\r\n> ");
		return 1;
	}

		/* :::::::::: Auto Accel Command :::::::: */
	case FSP_CMD_SET_AUTO_ACCEL:
		if(ps_FSP_RX->Payload.set_auto_accel.State)
			Enable_Auto_Pulsing();
		else
			Disable_Auto_Pulsing();
		break;

	case FSP_CMD_SET_THRESHOLD_ACCEL:
		Threshold_Accel.x = (int16_t)((ps_FSP_RX->Payload.set_threshold_accel.XH << 8) |
		                              ps_FSP_RX->Payload.set_threshold_accel.XL);
		Threshold_Accel.y = (int16_t)((ps_FSP_RX->Payload.set_threshold_accel.YH << 8) |
		                              ps_FSP_RX->Payload.set_threshold_accel.YL);
		Threshold_Accel.z = (int16_t)((ps_FSP_RX->Payload.set_threshold_accel.ZH << 8) |
		                              ps_FSP_RX->Payload.set_threshold_accel.ZL);

		break;

	case FSP_CMD_GET_THRESHOLD_ACCEL:
		ps_FSP_TX->CMD = FSP_CMD_GET_THRESHOLD_ACCEL;
		ps_FSP_TX->Payload.get_threshold_accel.XL = (uint8_t)(Threshold_Accel.x & 0xFF);
		ps_FSP_TX->Payload.get_threshold_accel.XH = (uint8_t)((Threshold_Accel.x >> 8) & 0xFF);
		ps_FSP_TX->Payload.get_threshold_accel.YL = (uint8_t)(Threshold_Accel.y & 0xFF);
		ps_FSP_TX->Payload.get_threshold_accel.YH = (uint8_t)((Threshold_Accel.y >> 8) & 0xFF);
		ps_FSP_TX->Payload.get_threshold_accel.ZL = (uint8_t)(Threshold_Accel.z & 0xFF);
		ps_FSP_TX->Payload.get_threshold_accel.ZH = (uint8_t)((Threshold_Accel.z >> 8) & 0xFF);

		fsp_print(7);
		break;

		/* :::::::::: Set Pulse Manual Command :::::::: */
	case FSP_CMD_SET_MANUAL_POLE:
	{
		if (is_manual_mode_enable == true)
		{
			H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
			H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
			
			manual_mode_on_count = 0;
		}

		H_Bridge_Set_Pole(&HB_pos_pole, &HB_neg_pole, ps_FSP_RX->Payload.set_manual_pole.pos_pole, ps_FSP_RX->Payload.set_manual_pole.neg_pole);

		if (is_manual_mode_enable == true)
		{
			H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_HS_ON);
			H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
		}
		
		break;
	}

	case FSP_CMD_SET_MANUAL_CAP:
	{
		manual_mode_which_cap = ps_FSP_RX->Payload.set_manual_cap.Which_Cap;

		if (manual_mode_which_cap == 1)
		{
			V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);
		}
		else if (manual_mode_which_cap == 2)
		{
			V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
		}
		else
		{
			V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);
		}
		
		break;
	}

	case FSP_CMD_SET_MANUAL_PULSE:
	{
		is_manual_mode_enable = ps_FSP_RX->Payload.set_manual_pulse.State;

		if (is_manual_mode_enable == true)
		{
			if (manual_mode_which_cap == 1)
			{
				V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);
			}
			else if (manual_mode_which_cap == 2)
			{
				V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
			}
			else
			{
				V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);
			}

			H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_HS_ON);
			H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
			LL_GPIO_SetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

			manual_mode_on_count = 0;

			SchedulerTaskEnable(7, 1);

			break;
		}
	
		V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);
		H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
		H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
		LL_GPIO_ResetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

		manual_mode_on_count = 0;

        ps_FSP_TX->CMD = FSP_CMD_SET_MANUAL_PULSE;
        ps_FSP_TX->Payload.set_manual_pulse.State = 0;
        fsp_print(2);

        SchedulerTaskDisable(7);

		break;
	}

		/* :::::::::: VOM Command :::::::: */
	case FSP_CMD_MEASURE_IMPEDANCE: {
		is_Measure_Impedance = true;

		Current_Sense_Period = ps_FSP_RX->Payload.measure_impedance.Period_high;
		Current_Sense_Period = Current_Sense_Period << 8;
		Current_Sense_Period |= ps_FSP_RX->Payload.measure_impedance.Period_low;

		is_h_bridge_enable = false;

		H_Bridge_Set_Pole(&HB_pos_pole, &HB_neg_pole, ps_FSP_RX->Payload.measure_impedance.Pos_pole_index, ps_FSP_RX->Payload.measure_impedance.Neg_pole_index);
		V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);
		H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
		H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_HS_ON);

		LL_ADC_REG_StartConversionSWStart(ADC_I_SENSE_HANDLE);
		SchedulerTaskEnable(3, 1);

		UART_Send_String(&RS232_UART, "Received FSP_CMD_GET_IMPEDANCE\r\n> ");
		return 1;
	}

		/* :::::::::: I2C Sensor Command :::::::: */
	case FSP_CMD_GET_SENSOR_GYRO: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_GYRO);
			FSP_process_state = 1;
			return 0;
		}

		case 1: {
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_GYRO;

			convert_Integer_To_Bytes(Sensor_Gyro.x,
					ps_FSP_TX->Payload.get_sensor_gyro.gyro_x, 2);
			convert_Integer_To_Bytes(Sensor_Gyro.y,
					ps_FSP_TX->Payload.get_sensor_gyro.gyro_y, 2);
			convert_Integer_To_Bytes(Sensor_Gyro.z,
					ps_FSP_TX->Payload.get_sensor_gyro.gyro_z, 2);

			fsp_print(7);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_ACCEL: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_ACCEL);
			FSP_process_state = 1;
			return 0;
		}

		case 1: {
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_ACCEL;

			convert_Integer_To_Bytes(Sensor_Accel.x,
					ps_FSP_TX->Payload.get_sensor_accel.accel_x, 2);
			convert_Integer_To_Bytes(Sensor_Accel.y,
					ps_FSP_TX->Payload.get_sensor_accel.accel_y, 2);
			convert_Integer_To_Bytes(Sensor_Accel.z,
					ps_FSP_TX->Payload.get_sensor_accel.accel_z, 2);

			fsp_print(7);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_LSM6DSOX: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_LSM6DSOX);
			FSP_process_state = 1;
			return 0;
		}

		case 1: {
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_LSM6DSOX;

			convert_Integer_To_Bytes(Sensor_Gyro.x,
					ps_FSP_TX->Payload.get_sensor_LSM6DSOX.gyro_x, 2);
			convert_Integer_To_Bytes(Sensor_Gyro.y,
					ps_FSP_TX->Payload.get_sensor_LSM6DSOX.gyro_y, 2);
			convert_Integer_To_Bytes(Sensor_Gyro.z,
					ps_FSP_TX->Payload.get_sensor_LSM6DSOX.gyro_z, 2);

			convert_Integer_To_Bytes(Sensor_Accel.x,
					ps_FSP_TX->Payload.get_sensor_LSM6DSOX.accel_x, 2);
			convert_Integer_To_Bytes(Sensor_Accel.y,
					ps_FSP_TX->Payload.get_sensor_LSM6DSOX.accel_y, 2);
			convert_Integer_To_Bytes(Sensor_Accel.z,
					ps_FSP_TX->Payload.get_sensor_LSM6DSOX.accel_z, 2);

			fsp_print(13);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_TEMP: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_TEMP);
			FSP_process_state = 1;
			return 0;
		}

		case 1:
		{
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_TEMP;

			double_to_string(Sensor_Temp,
					(char*) ps_FSP_TX->Payload.get_sensor_temp.temp, 2);

			fsp_print(7);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_PRESSURE: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_PRESSURE);
			FSP_process_state = 1;
			return 0;
		}

		case 1:
		{
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_PRESSURE;

			double_to_string(Sensor_Pressure,
					(char*) ps_FSP_TX->Payload.get_sensor_pressure.pressure, 0);

			fsp_print(8);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_ALTITUDE: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_ALTITUDE);
			FSP_process_state = 1;
			return 0;
		}

		case 1:
		{
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_ALTITUDE;

			double_to_string(Sensor_Altitude,
					(char*) ps_FSP_TX->Payload.get_sensor_altitude.altitude, 2);

			fsp_print(5);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_BMP390: {
		switch (FSP_process_state) {
		case 0: {
			Sensor_Read_Value(SENSOR_READ_BMP390);
			FSP_process_state = 1;
			return 0;
		}

		case 1:
		{
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_BMP390;

			double_to_string(Sensor_Temp,
					(char*) ps_FSP_TX->Payload.get_sensor_BMP390.temp, 2);
			double_to_string(Sensor_Pressure,
					(char*) ps_FSP_TX->Payload.get_sensor_BMP390.pressure, 0);
			double_to_string(Sensor_Altitude,
					(char*) ps_FSP_TX->Payload.get_sensor_BMP390.altitude, 2);

			fsp_print(18);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

	case FSP_CMD_GET_SENSOR_H3LIS331DL:
	{
		switch (FSP_process_state)
		{
		case 0:
		{
			Sensor_Read_Value(SENSOR_READ_H3LIS331DL);
			FSP_process_state = 1;
			return 0;
		}

		case 1:
		{
			if (Is_Sensor_Read_Complete() == false)
			{
				return 0;
			}

			ps_FSP_TX->CMD = FSP_CMD_GET_SENSOR_H3LIS331DL;

			convert_Integer_To_Bytes(H3LIS_Accel.x,
					ps_FSP_TX->Payload.get_sensor_H3LIS331DL.accel_x, 3);
			convert_Integer_To_Bytes(H3LIS_Accel.y,
					ps_FSP_TX->Payload.get_sensor_H3LIS331DL.accel_y, 3);
			convert_Integer_To_Bytes(H3LIS_Accel.z,
					ps_FSP_TX->Payload.get_sensor_H3LIS331DL.accel_z, 3);

			fsp_print(10);
			FSP_process_state = 0;
			return 1;
		}

		default:
			break;
		}
		return 0;
	}

		/* :::::::::: Ultility Command :::::::: */
	case FSP_CMD_HANDSHAKE: {
		ps_FSP_TX->CMD = FSP_CMD_HANDSHAKE;
		ps_FSP_TX->Payload.handshake.Check = 0xAB;

		fsp_print(2);

		UART_Send_String(&RS232_UART, "Received FSP_CMD_HANDSHAKE\r\n> ");
		return 1;
	}


	default:
		break;
	}

	return 1;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static void fsp_print(uint8_t packet_length) {
	s_FSP_TX_Packet.sod = FSP_PKT_SOD;
	s_FSP_TX_Packet.src_adr = fsp_my_adr;
	s_FSP_TX_Packet.dst_adr = FSP_ADR_GPC;
	s_FSP_TX_Packet.length = packet_length;
	s_FSP_TX_Packet.type = FSP_PKT_TYPE_CMD_W_DATA;
	s_FSP_TX_Packet.eof = FSP_PKT_EOF;
	s_FSP_TX_Packet.crc16 = crc16_CCITT(FSP_CRC16_INITIAL_VALUE,
			&s_FSP_TX_Packet.src_adr, s_FSP_TX_Packet.length + 4);

	uint8_t encoded_frame[100] = { 0 };
	uint8_t frame_len;
	fsp_encode(&s_FSP_TX_Packet, encoded_frame, &frame_len);

	UART_FSP(&GPC_UART, (char*) encoded_frame, frame_len);
}

static void convert_Integer_To_Bytes(uint32_t number, uint8_t arr[], uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	{
		arr[i] = (number >> (8 * i)) & 0xff;
	}
}

static void double_to_string(double value, char *buffer, uint8_t precision) {
	// Handle negative numbers
	if (value < 0) {
		*buffer++ = '-';
		value = -value;
	}

	// Extract the integer part
	uint32_t integer_part = (uint32_t) value;
	double fractional_part = value - integer_part;

	// Convert integer part to string
	sprintf(buffer, "%ld", integer_part);
	while (*buffer)
		buffer++; // Move pointer to the end of the integer part

	// Add decimal point
	if (precision > 0) {
		*buffer++ = '.';

		// Extract and convert the fractional part
		for (uint8_t i = 0; i < precision; i++) {
			fractional_part *= 10;
			uint8_t digit = (uint8_t) fractional_part;
			*buffer++ = '0' + digit;
			fractional_part -= digit;
		}
	}

	// Null-terminate the string
	*buffer = '\0';
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
