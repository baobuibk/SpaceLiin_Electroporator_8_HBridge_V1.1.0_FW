#ifndef _SENSOR_DRIVER_H_

#define _SENSOR_DRIVER_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define BMP390_ADDR                     0xEE
#define BMP390_STATUS_REG_ADDR			0x03

#define BMP390_PRESSURE_REG_ADDR_BASE	0x04
#define BMP390_PRESSURE_REG_SIZE		3

#define BMP390_TEMP_REG_ADDR_BASE		0x07
#define BMP390_TEMP_REG_SIZE			3

#define BMP390_ALTITUDE_REG_ADDR_BASE	0x07
#define BMP390_ALTITUDE_REG_SIZE		3

#define BMP390_READ_ALL_ADDR_BASE		0x04
#define BMP390_READ_ALL_REG_SIZE		6

#define ATM_PRESSURE_PA                 101325.0
#define BAROMETRIC_CONSTANT             44330.0

#define LSM6DSOX_ADDR                   0xD4
#define LSM6DSOX_STATUS_REG_ADDR		0x1E

#define LSM6DSOX_GYRO_REG_ADDR_BASE		0x22
#define LSM6DSOX_GYRO_REG_SIZE			6

#define LSM6DSOX_ACCEL_REG_ADDR_BASE	0x28
#define LSM6DSOX_ACCEL_REG_SIZE			6

#define LSM6DSOX_READ_ALL_ADDR_BASE		0x22
#define LSM6DSOX_READ_ALL_SIZE			12

#define GYRO_SENSITIVITY_500DPS		    17.50f
#define LSM6DSOX_ACCEL_FS_8G			0.244f

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef enum _Sensor_Read_Status_typedef_
{
	SENSOR_READ_OK_STATUS,
	SENSOR_READ_BUSY_STATUS,
	SENSOR_READ_ERROR_STATUS,
} Sensor_Read_Status_typedef;

typedef enum _Sensor_Read_typedef_
{
	/* :::::::::: LSM6DSOX Read Data Type :::::::: */
	SENSOR_READ_GYRO,
	SENSOR_READ_ACCEL,
	SENSOR_READ_LSM6DSOX,

	/* :::::::::: BMP390 Read Data Type :::::::: */
	SENSOR_READ_TEMP,
	SENSOR_READ_PRESSURE,
	SENSOR_READ_ALTITUDE,
	SENSOR_READ_BMP390,

} Sensor_Read_typedef;

/* typedef enum _BMP390_read_typedef_
{
	BMP390_READ_TEMP,
	BMP390_READ_PRESSURE,
	BMP390_READ_ALTITUDE,
	BMP390_READ_ALL,
} BMP390_read_typedef;

typedef enum _LSM6DSOX_read_typedef_
{
	LSM6DSOX_READ_GYRO,
	LSM6DSOX_READ_ACCEL,
	LSM6DSOX_READ_ALL,
} LSM6DSOX_read_typedef; */

typedef struct _LSM6DSOX_data_typedef_
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
} LSM6DSOX_data_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern bool	  is_BMP390_read_enable;
extern double Sensor_Temp;
extern double Sensor_Pressure;
extern double Sensor_Altitude;

extern bool					 is_LSM6DSOX_read_enable;
extern LSM6DSOX_data_typedef Sensor_Gyro;
extern LSM6DSOX_data_typedef Sensor_Accel;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
bool Sensor_Init(void);

/* :::::::::: LSM6DSOX Command :::::::: */
bool Sensor_Read_Driver_Process(Sensor_Read_typedef read_type);

/* :::::::::: IRQ Handler ::::::::::::: */
void I2C_EV_IRQHandler(void);

#endif //_SENSOR_DRIVER_H_
