#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f405xx.h"
#include "stm32f4xx_ll_gpio.h"

// SYSTEM DRIVER //
#include "board.h"

// USER DRIVER //
#include "scheduler.h"
#include "uart.h"

// INCLUDE DRIVER //
#include "h_bridge_driver.h"
#include "v_switch_driver.h"

// INCLUDE TASK //
#include "cmd_line_task.h"
#include "h_bridge_task.h"
#include "v_switch_task.h"
#include "fsp_line_task.h"
#include "sensor_task.h"
#include "current_sense_task.h"
/*
#include "impedance_task.h"
#include "BMP390.h"
*/


void App_Main(void);

#endif /* APP_H_ */
