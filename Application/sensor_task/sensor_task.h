/*
 * BMP390.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Administrator
 */

#ifndef _SENSOR_TASK_H_
#define _SENSOR_TASK_H_

#include "lsm6dsox.h"
#include "bmp390.h"
#include "h3lis331dl.h"

extern bool is_accel_calib;

void Sensor_Task_Init(void);
void Sensor_Read_Value(Sensor_Read_typedef read_type);
void Sensor_Read_Task(void*);
bool Is_Sensor_Read_Complete(void);

#endif /* _SENSOR_TASK_H_ */
