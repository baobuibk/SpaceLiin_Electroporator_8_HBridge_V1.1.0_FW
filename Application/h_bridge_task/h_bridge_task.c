/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "app.h"
#include "stm32f4xx_ll_gpio.h"

#include "h_bridge_driver.h"
#include "v_switch_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum
{
    H_BRIDGE_STOP_STATE,
    H_BRIDGE_HV_1_STATE,
    H_BRDIGE_HV_2_STATE,
    H_BRIDGE_LV_1_STATE,
    H_BRIDGE_LV_2_STATE,
} H_Bridge_State_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
H_Bridge_State_typedef H_Bridge_State = H_BRIDGE_STOP_STATE;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
bool        is_h_bridge_enable          = false;

uint16_t    hv_w1_count                 = 0;
uint16_t    hv_w2_count                 = 0;

uint16_t    pulse_delay_ms              = 15;

uint8_t     hv_pulse_pos_count          = 5;
uint8_t     hv_pulse_neg_count          = 6;
uint8_t     hv_delay_ms                 = 5;
uint8_t     hv_on_time_ms               = 5;
uint8_t     hv_off_time_ms              = 15;

uint8_t     lv_pulse_pos_count          = 7;
uint8_t     lv_pulse_neg_count          = 8;
uint8_t     lv_delay_ms                 = 10;
uint16_t    lv_on_time_ms               = 50;
uint16_t    lv_off_time_ms              = 90;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Task Init :::::::: */
void H_Bridge_Task_Init(void)
{
    ;
}

void H_Bridge_Pulse_Control(bool pulse_state)
{
    is_h_bridge_enable = true;
    SchedulerTaskEnable(0, 1);
}

void H_Bridge_Set_HV_Timing(uint8_t on_time_ms, uint8_t off_time_ms)
{
    ;
}

/* :::::::::: H Bridge Task ::::::::::::: */
void H_Bridge_Task(void*)
{
    switch (H_Bridge_State)
    {
    case H_BRIDGE_STOP_STATE:
        if(is_h_bridge_enable == false)
        {
            V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);
            //V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

            H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
            H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);

            SchedulerTaskDisable(0);
        }
        else if(is_h_bridge_enable == true)
        {
            if (hv_pulse_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, 1, hv_on_time_ms, hv_off_time_ms, hv_pulse_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_HV_1_STATE;
            }
            else if (hv_pulse_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, 1, hv_on_time_ms, hv_off_time_ms, hv_pulse_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRDIGE_HV_2_STATE;
            }
            else if (lv_pulse_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, 1, lv_on_time_ms, lv_off_time_ms, lv_pulse_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_1_STATE;
            }
            else if (lv_pulse_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, 1, lv_on_time_ms, lv_off_time_ms, lv_pulse_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_2_STATE;
            }
            else
            {
                is_h_bridge_enable = false;
                H_Bridge_State = H_BRIDGE_STOP_STATE;
            }
        }
        break;
    case H_BRIDGE_HV_1_STATE:
        if(is_h_bridge_enable == false)
        {
            H_Bridge_State  = H_BRIDGE_STOP_STATE;
        }
        else if(HB_pos_pole.pulse_count >= ((HB_pos_pole.set_pulse_count * 2) + 1))
        {
            if (hv_pulse_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, hv_delay_ms, hv_on_time_ms, hv_off_time_ms, hv_pulse_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRDIGE_HV_2_STATE;
            }
            else if (lv_pulse_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, pulse_delay_ms, lv_on_time_ms, lv_off_time_ms, lv_pulse_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_1_STATE;
            }
            else if (lv_pulse_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, pulse_delay_ms, lv_on_time_ms, lv_off_time_ms, lv_pulse_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_2_STATE;
            }
            else
            {
                is_h_bridge_enable = false;
                H_Bridge_State = H_BRIDGE_STOP_STATE;
            }
        }
        break;
    case H_BRDIGE_HV_2_STATE:
        if(is_h_bridge_enable == false)
        {
            H_Bridge_State = H_BRIDGE_STOP_STATE;
        }
        else if(HB_neg_pole.pulse_count >= ((HB_neg_pole.set_pulse_count * 2) + 1))
        {
            if (lv_pulse_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, pulse_delay_ms, lv_on_time_ms, lv_off_time_ms, lv_pulse_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_1_STATE;
            }
            else if (lv_pulse_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, pulse_delay_ms, lv_on_time_ms, lv_off_time_ms, lv_pulse_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_2_STATE;
            }
            else
            {
                is_h_bridge_enable = false;
                H_Bridge_State = H_BRIDGE_STOP_STATE;
            }
        }
        break;
    case H_BRIDGE_LV_1_STATE:
        if(is_h_bridge_enable == false)
        {
            H_Bridge_State = H_BRIDGE_STOP_STATE;
        }
        else if(HB_pos_pole.pulse_count >= ((HB_pos_pole.set_pulse_count * 2) + 1))
        {
            if (lv_pulse_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, lv_delay_ms, lv_on_time_ms, lv_off_time_ms, lv_pulse_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_2_STATE;
            }
            else
            {
                is_h_bridge_enable = false;
                H_Bridge_State = H_BRIDGE_STOP_STATE;
            }
        }
        break;
    case H_BRIDGE_LV_2_STATE:
        if(is_h_bridge_enable == false)
        {
            H_Bridge_State = H_BRIDGE_STOP_STATE;
        }
        else if(HB_neg_pole.pulse_count >= ((HB_neg_pole.set_pulse_count * 2) + 1))
        {
            /*
            V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

            H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
            H_Bridge_Set_Pulse_Timing(&HB_pos_pole, pulse_delay_ms, hv_on_time_ms, hv_off_time_ms, hv_pulse_count);
            H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

            H_Bridge_State = H_BRIDGE_HV_1_STATE;
            */
        
            is_h_bridge_enable = false;
            H_Bridge_State = H_BRIDGE_STOP_STATE;
        }
        break;

    default:
        break;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
