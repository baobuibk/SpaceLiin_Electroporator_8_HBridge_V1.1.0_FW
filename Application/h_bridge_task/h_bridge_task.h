#ifndef H_TASK_H_
#define H_TASK_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <stdbool.h>

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef struct _H_Bridge_task_typedef_
{
    uint8_t     is_setted;

    uint16_t    sequence_delay_ms;

    uint8_t     pos_pole_index;
    uint8_t     neg_pole_index;

    uint16_t    pulse_delay_ms;

    uint8_t     hv_pos_count;
    uint8_t     hv_neg_count;
    uint8_t     hv_delay_ms;
    uint8_t     hv_on_ms;
    uint8_t     hv_off_ms;

    uint8_t     lv_pos_count;
    uint8_t     lv_neg_count;
    uint8_t     lv_delay_ms;
    uint16_t    lv_on_ms;
    uint16_t    lv_off_ms;
} H_Bridge_task_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern H_Bridge_task_typedef    HB_sequence_array[10];
extern H_Bridge_task_typedef*   ps_HB_current;

extern bool                     is_h_bridge_enable;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Task Init :::::::: */
void H_Bridge_Task_Init(void);

/* :::::::::: H Bridge Task ::::::::::::: */
void H_Bridge_Task(void*);

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
