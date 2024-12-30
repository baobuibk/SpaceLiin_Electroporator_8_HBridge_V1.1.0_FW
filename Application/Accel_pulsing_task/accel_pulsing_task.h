/*
 * accel_pulsing_task.h
 *
 *  Created on: Dec 17, 2024
 *      Author: thanh
 */

#ifndef ACCEL_PULSING_TASK_ACCEL_PULSING_TASK_H_
#define ACCEL_PULSING_TASK_ACCEL_PULSING_TASK_H_

extern LSM6DSOX_data_typedef Threshold_Accel;
/* :::::::::: Task Init :::::::: */
void Accel_Pulsing_Task_Init();

/* :::::::::: Task ::::::::::::: */
void Accel_Pulsing_Task(void*);
void Enable_Auto_Pulsing();
void Disable_Auto_Pulsing();
#endif /* ACCEL_PULSING_TASK_ACCEL_PULSING_TASK_H_ */
