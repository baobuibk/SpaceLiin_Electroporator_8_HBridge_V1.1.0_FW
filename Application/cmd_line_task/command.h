#ifndef COMMAND_H_
#define COMMAND_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern uint8_t CMD_sequence_index;
extern uint8_t CMD_total_sequence_index;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: Pulse Control Command :::::::: */
int CMD_SET_SEQUENCE_INDEX(int argc, char *argv[]);
int CMD_SET_SEQUENCE_DELETE(int argc, char *argv[]);
int CMD_SET_SEQUENCE_CONFIRM(int argc, char *argv[]);
int CMD_SET_SEQUENCE_DELAY(int argc, char *argv[]);

int CMD_SET_PULSE_POLE(int argc, char *argv[]);
int CMD_SET_PULSE_COUNT(int argc, char *argv[]);
int CMD_SET_PULSE_DELAY(int argc, char *argv[]);
int CMD_SET_PULSE_HV(int argc, char *argv[]);
int CMD_SET_PULSE_LV(int argc, char *argv[]);
int CMD_SET_PULSE_CONTROL(int argc, char *argv[]);

int CMD_GET_SEQUENCE_INDEX(int argc, char *argv[]);
int CMD_GET_SEQUENCE_DELAY(int argc, char *argv[]);
int CMD_GET_SEQUENCE_ALL(int argc, char *argv[]);

int CMD_GET_PULSE_POLE(int argc, char *argv[]);
int CMD_GET_PULSE_COUNT(int argc, char *argv[]);
int CMD_GET_PULSE_DELAY(int argc, char *argv[]);
int CMD_GET_PULSE_HV(int argc, char *argv[]);
int CMD_GET_PULSE_LV(int argc, char *argv[]);
int CMD_GET_PULSE_CONTROL(int argc, char *argv[]);
int CMD_GET_PULSE_ALL(int argc, char *argv[]);

/* :::::::::: VOM Command :::::::: */
int CMD_MEASURE_IMPEDANCE(int argc, char *argv[]);

/* :::::::::: I2C Sensor Command :::::::: */
int CMD_GET_SENSOR_GYRO(int argc, char *argv[]);
int CMD_GET_SENSOR_ACCEL(int argc, char *argv[]);
int CMD_GET_SENSOR_LSM6DSOX(int argc, char *argv[]);

int CMD_GET_SENSOR_TEMP(int argc, char *argv[]);
int CMD_GET_SENSOR_PRESSURE(int argc, char *argv[]);
int CMD_GET_SENSOR_ALTITUDE(int argc, char *argv[]);
int CMD_GET_SENSOR_BMP390(int argc, char *argv[]);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Auto Pulsing Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
int CMD_SET_THRESHOLD_ACCEL(int argc, char *argv[]);
int CMD_GET_THRESHOLD_ACCEL(int argc, char *argv[]);
int CMD_SET_AUTO_ACCEL(int argc, char *argv[]);
int CMD_CALIB_ACCEL(int argc, char *argv[]);
/* :::::::::: Ultility Command :::::::: */
int CMD_CLEAR_SCREEN(int argc, char *argv[]);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* COMMAND_H_ */
