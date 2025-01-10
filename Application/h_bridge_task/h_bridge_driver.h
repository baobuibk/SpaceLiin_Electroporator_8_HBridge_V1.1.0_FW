#ifndef H_BRIDGE_DRIVER_H_
#define H_BRIDGE_DRIVER_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_ll_gpio.h"
#include "v_switch_driver.h"
#include "pwm.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
typedef enum _H_Bridge_mode_typedef_
{
    H_BRIDGE_MODE_HS_ON,
    H_BRIDGE_MODE_LS_ON,
    H_BRIDGE_MODE_FLOAT,
    H_BRIDGE_MODE_PULSE,
}H_Bridge_mode;

typedef struct _H_Bridge_typdef_
{
    PWM_TypeDef     PWM;
    GPIO_TypeDef    *Port;
    uint32_t        *Pin;
    bool            *Pin_State;
    H_Bridge_mode   Mode;

    uint16_t        delay_time_ms;
    uint16_t        on_time_ms;
    uint16_t        off_time_ms;
    uint16_t        pulse_count;
    uint16_t        set_pulse_count;
    bool            is_setted;
}H_Bridge_typdef;

typedef struct _H_Bridge_Task_Data_typedef_
{
    V_Switch_mode    VS_mode;

    H_Bridge_typdef  HB_pole_ls_on;
    H_Bridge_typdef  HB_pole_pulse;

    bool             is_setted;
} H_Bridge_Task_data_typedef;

typedef struct _H_Bridge_Task_typedef_
{
    bool                       is_setted;
    H_Bridge_Task_data_typedef task_data[4];

} H_Bridge_Task_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
extern H_Bridge_typdef* p_HB_SD_0_3_IRQn;
extern H_Bridge_typdef* p_HB_SD_4_7_IRQn;

extern H_Bridge_typdef HB_pos_pole;
extern H_Bridge_typdef HB_neg_pole;

extern H_Bridge_Task_typedef HB_Task_data[10];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void H_Bridge_Driver_Init(void);

void H_Bridge_Set_Pole(H_Bridge_typdef* p_HB_pos_pole, H_Bridge_typdef* p_HB_neg_pole, uint8_t pos_pole_index, uint8_t neg_pole_index);

void H_Bridge_Set_Mode(H_Bridge_typdef* H_Bridge_x, H_Bridge_mode SetMode);

void H_Bridge_Calculate_Timing(
                               H_Bridge_Task_data_typedef* p_HB_task_data,
                               V_Switch_mode               _VS_mode_,

                               uint16_t Set_delay_time_ms, 
                               uint16_t Set_on_time_ms, 
                               uint16_t Set_off_time_ms, 
                               uint16_t Set_pulse_count
                              );

void H_Bridge_Kill(void);

void H_Bridge_SD_Interupt_Handle(H_Bridge_typdef* p_HB_SD_IRQn);
void H_Bridge_SD0_3_Interupt_Handle();
void H_Bridge_SD4_7_Interupt_Handle();

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif /* H_BRIDGE_DRIVER_H_ */

// void H_Bridge_Set_Pulse_Timing(H_Bridge_typdef* H_Bridge_x, uint16_t Set_delay_time_ms, uint16_t Set_on_time_ms, uint16_t Set_off_time_ms, uint16_t Set_pulse_count);