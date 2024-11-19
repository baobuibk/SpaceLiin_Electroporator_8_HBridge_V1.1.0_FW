/*
 * BMP390.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Administrator
 */

#ifndef _SENSOR_TASK_H_
#define _SENSOR_TASK_H_

#include "sensor_driver.h"

extern bool is_sensor_read_finished;

void Sensor_Task_Init(void);
void Sensor_Read_Value(Sensor_Read_typedef read_type);
void Sensor_Read_Task(void*);

#endif /* _SENSOR_TASK_H_ */
