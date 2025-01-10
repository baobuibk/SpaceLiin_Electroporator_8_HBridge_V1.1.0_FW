#ifndef _FSP_FRAME_H

#define _FSP_FRAME_H

// Commands
typedef enum _FSP_CMD_typedef_
{
	/* :::::::::: Pulse Control Command :::::::: */
	FSP_CMD_SET_SEQUENCE_INDEX,
	FSP_CMD_SET_SEQUENCE_CONFIRM,
	FSP_CMD_SET_SEQUENCE_DELETE,
	FSP_CMD_SET_SEQUENCE_DELAY,

	FSP_CMD_SET_PULSE_POLE,
	FSP_CMD_SET_PULSE_COUNT,
	FSP_CMD_SET_PULSE_DELAY,
	FSP_CMD_SET_PULSE_HV_POS,
	FSP_CMD_SET_PULSE_HV_NEG,
	FSP_CMD_SET_PULSE_LV_POS,
	FSP_CMD_SET_PULSE_LV_NEG,
	FSP_CMD_SET_PULSE_CONTROL,

	/* :::::::::: VOM Command :::::::: */
	FSP_CMD_MEASURE_VOLT,
	FSP_CMD_MEASURE_CURRENT,
	FSP_CMD_MEASURE_IMPEDANCE,

	/* :::::::::: I2C Sensor Command :::::::: */
	FSP_CMD_GET_SENSOR_GYRO,
	FSP_CMD_GET_SENSOR_ACCEL,
	FSP_CMD_GET_SENSOR_LSM6DSOX,

	FSP_CMD_GET_SENSOR_TEMP,
	FSP_CMD_GET_SENSOR_PRESSURE,
	FSP_CMD_GET_SENSOR_BMP390,

	/* :::::::::: Ultility Command :::::::: */
	FSP_CMD_HANDSHAKE,

	/* :::::::::: Auto Accel Command :::::::: */
	FSP_CMD_SET_AUTO_ACCEL,
	FSP_CMD_SET_THRESHOLD_ACCEL,
	FSP_CMD_GET_THRESHOLD_ACCEL,
	FSP_CMD_STREAM_ACCEL,
	
} FSP_CMD_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Pulse Control Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef struct _FSP_SET_SEQUENCE_INDEX_
{
	uint8_t 	index;			/* hv pulse count */
} FSP_SET_SEQUENCE_INDEX;

typedef struct _FSP_SET_SEQUENCE_DELAY_
{
	uint8_t 	Delay_high;
	uint8_t		Delay_low;
} FSP_SET_SEQUENCE_DELAY;

typedef struct _FSP_SET_PULSE_POLE_FRAME_
{
	uint8_t 	pos_pole;		/* hv pulse count */
	uint8_t 	neg_pole;		/* hv pulse count */

} FSP_SET_PULSE_POLE_FRAME;

typedef struct _FSP_SET_PULSE_COUNT_FRAME_
{
	uint8_t 	HV_pos_count;		/* hv pulse count */
	uint8_t 	HV_neg_count;		/* hv pulse count */
	uint8_t 	LV_pos_count;		/* lv pulse count */
	uint8_t 	LV_neg_count;		/* lv pulse count */

} FSP_SET_PULSE_COUNT_FRAME;

typedef struct _FSP_SET_PULSE_DELAY_FRAME_
{
	uint8_t 	HV_delay;
	uint8_t 	LV_delay;
	uint8_t 	Delay_high;		  	/* Delay time */
	uint8_t 	Delay_low;		  	/* Delay time */

} FSP_SET_PULSE_DELAY_FRAME;

typedef struct _FSP_SET_PULSE_HV_FRAME_
{
	uint8_t 	OnTime;      	/* HV On time */
	uint8_t 	OffTime;      	/* HV Off time */

} FSP_SET_PULSE_HV_FRAME;

typedef struct _FSP_SET_PULSE_LV_FRAME_
{
	uint8_t 	OnTime_high;      	/* LV On time */
	uint8_t 	OnTime_low;      	/* LV On time */
	uint8_t 	OffTime_high;      	/* LV Off time */
	uint8_t 	OffTime_low;      	/* LV Off time */

} FSP_SET_PULSE_LV_FRAME;

typedef struct _FSP_SET_PULSE_CONTROL_FRAME_
{
	uint8_t 	State;      	/* 0: OFF, 1: ON */

} FSP_SET_PULSE_CONTROL_FRAME;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ VOM Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef struct _MEASURE_VOLT_FRAME_
{
	uint8_t		HV_volt[4];
} MEASURE_VOLT_FRAME;

typedef struct _MEASURE_CURRENT_FRAME_
{
	uint8_t 	Value_high;
	uint8_t 	Value_low;

} MEASURE_CURRENT_FRAME;

typedef struct _MEASURE_IMPEDANCE_FRAME_
{
	uint8_t		Pos_pole_index;
	uint8_t		Neg_pole_index;
	uint8_t		Period_high;
	uint8_t		Period_low;
	uint8_t 	Value_high;
	uint8_t 	Value_low;

} MEASURE_IMPEDANCE_FRAME;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ I2C Sensor Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef struct _GET_SENSOR_GYRO_
{
	uint8_t 	gyro_x[2];
	uint8_t 	gyro_y[2];
	uint8_t 	gyro_z[2];

} GET_SENSOR_GYRO;

typedef struct _GET_SENSOR_ACCEL_
{
	uint8_t 	accel_x[2];
	uint8_t 	accel_y[2];
	uint8_t 	accel_z[2];

} GET_SENSOR_ACCEL;

typedef struct _GET_SENSOR_LSM6DSOX_
{
	uint8_t 	gyro_x[2];
	uint8_t 	gyro_y[2];
	uint8_t 	gyro_z[2];
	uint8_t 	accel_x[2];
	uint8_t 	accel_y[2];
	uint8_t 	accel_z[2];

} GET_SENSOR_LSM6DSOX;

typedef struct _GET_SENSOR_TEMP_
{
	uint8_t 	temp[6];

} GET_SENSOR_TEMP;

typedef struct _GET_SENSOR_PRESSURE_
{
	uint8_t 	pressure[7];

} GET_SENSOR_PRESSURE;

typedef struct _GET_SENSOR_BMP390_
{
	uint8_t 	temp[6];
	uint8_t 	pressure[7];

} GET_SENSOR_BMP390;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Ultility Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef struct _FSP_HANDSAKE_
{
	uint8_t 	Check;

} FSP_HANDSAKE;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Auto Accel Command ~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef struct _FSP_SET_AUTO_ACCEL_FRAME_
{
	uint8_t 	State;      	/* 0: OFF, 1: ON */

} FSP_SET_AUTO_ACCEL_FRAME;

typedef struct _FSP_SET_THRESHOLD_ACCEL_FRAME_
{
	uint8_t 	XL;
	uint8_t 	XH;
	uint8_t		YL;
	uint8_t		YH;
	uint8_t		ZL;
	uint8_t		ZH;

} FSP_SET_THRESHOLD_ACCEL_FRAME;

typedef struct _FSP_GET_THRESHOLD_ACCEL_FRAME_
{
	uint8_t 	XL;
	uint8_t 	XH;
	uint8_t		YL;
	uint8_t		YH;
	uint8_t		ZL;
	uint8_t		ZH;

} FSP_GET_THRESHOLD_ACCEL_FRAME;

typedef struct _FSP_CMD_STREAM_ACCEL_FRAME_
{
	uint8_t		count;
	uint8_t 	XL;
	uint8_t 	XH;
	uint8_t		YL;
	uint8_t		YH;
	uint8_t		ZL;
	uint8_t		ZH;
} FSP_CMD_STREAM_ACCEL_FRAME;

// Union to encapsulate all frame types
typedef union _FSP_Payload_Frame_typedef_
{
	/* :::::::::: Pulse Control Command :::::::: */
	FSP_SET_SEQUENCE_INDEX					set_sequence_index;
	FSP_SET_SEQUENCE_DELAY					set_sequence_delay;

	FSP_SET_PULSE_POLE_FRAME				set_pulse_pole;
	FSP_SET_PULSE_COUNT_FRAME				set_pulse_count;
	FSP_SET_PULSE_DELAY_FRAME				set_pulse_delay;
	FSP_SET_PULSE_HV_FRAME					set_pulse_HV_pos;
	FSP_SET_PULSE_HV_FRAME					set_pulse_HV_neg;
	FSP_SET_PULSE_LV_FRAME					set_pulse_LV_pos;
	FSP_SET_PULSE_LV_FRAME					set_pulse_LV_neg;
	FSP_SET_PULSE_CONTROL_FRAME				set_pulse_control;

	/* :::::::::: VOM Command :::::::: */
	MEASURE_VOLT_FRAME						measure_volt;
	MEASURE_CURRENT_FRAME					measure_current;
	MEASURE_IMPEDANCE_FRAME					measure_impedance;

	/* :::::::::: I2C Sensor Command :::::::: */
	GET_SENSOR_GYRO							get_sensor_gyro;
	GET_SENSOR_ACCEL						get_sensor_accel;
	GET_SENSOR_LSM6DSOX						get_sensor_LSM6DSOX;
	
	GET_SENSOR_TEMP							get_sensor_temp;
	GET_SENSOR_PRESSURE						get_sensor_pressure;
	GET_SENSOR_BMP390						get_sensor_BMP390;

	/* :::::::::: Ultility Command :::::::: */
	FSP_HANDSAKE							handshake;

	/* :::::::::: Auto Accel Command :::::::: */
	FSP_SET_AUTO_ACCEL_FRAME				set_auto_accel;
	FSP_SET_THRESHOLD_ACCEL_FRAME			set_threshold_accel;
	FSP_GET_THRESHOLD_ACCEL_FRAME			get_threshold_accel;
	FSP_CMD_STREAM_ACCEL_FRAME				stream_accel;
} FSP_Payload_Frame_typedef;

typedef struct _FSP_Payload_typedef_
{
	uint8_t						CMD;
	FSP_Payload_Frame_typedef 	Payload;
	
} FSP_Payload;

/* :::::::::: FSP Line Process ::::::::::::: */
uint8_t FSP_Line_Process();

#endif
