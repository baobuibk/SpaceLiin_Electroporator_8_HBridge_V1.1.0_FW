#ifndef H_TASK_H_
#define H_TASK_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <stdbool.h>

// #include "h_bridge_driver.h"
// #include "v_switch_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// typedef struct _H_Bridge_Task_Data_typedef_
// {
//     V_Switch_mode    VS_mode;

//     H_Bridge_typdef  HB_pole_ls_on;
//     H_Bridge_typdef  HB_pole_pulse;

//     bool             is_setted;
// } H_Bridge_Task_data_typedef;

// typedef struct _H_Bridge_Task_typedef_
// {
//     bool                       is_setted;
//     H_Bridge_Task_data_typedef task_data[4];

// } H_Bridge_Task_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// extern H_Bridge_Task_typedef HB_Task_data[10];

extern bool is_h_bridge_enable;

extern bool is_manual_mode_enable;
extern uint16_t manual_mode_on_count;
extern uint8_t manual_mode_which_cap;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Task Init :::::::: */
void H_Bridge_Task_Init(void);

/* :::::::::: H Bridge Task ::::::::::::: */
void H_Bridge_Task(void*);
void H_Bridge_Manual_Task(void*);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

//extern uint16_t                 pulse_delay_ms;
//
//extern uint8_t                  hv_pulse_pos_count;
//extern uint8_t                  hv_pulse_neg_count;
//extern uint8_t                  hv_delay_ms;
//extern uint8_t                  hv_on_time_ms;
//extern uint8_t                  hv_off_time_ms;
//
//extern uint8_t                  lv_pulse_pos_count;
//extern uint8_t                  lv_pulse_neg_count;
//extern uint8_t                  lv_delay_ms;
//extern uint16_t                 lv_on_time_ms;
//extern uint16_t                 lv_off_time_ms;

#endif /* H_TASK_H_ */
