/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "app.h"
#include "stm32f4xx_ll_gpio.h"

#include "h_bridge_driver.h"
#include "v_switch_driver.h"

#include "crc.h"

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
static H_Bridge_State_typedef H_Bridge_State = H_BRIDGE_STOP_STATE;

static uint8_t current_sequence_index = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool H_Bridge_Set_Next_Sequence(void);
static void fsp_print(uint8_t packet_length);
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
H_Bridge_task_typedef  HB_sequence_array[10] = {0};

H_Bridge_task_typedef* ps_HB_current = &HB_sequence_array[0];

bool is_h_bridge_enable = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Task Init :::::::: */
void H_Bridge_Task_Init(void)
{
    current_sequence_index = 0;
    
    for (uint8_t i = 0; i < 10; i++)
    {
        HB_sequence_array[i].is_setted = false;
    }
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
            H_Bridge_Set_Pole(ps_HB_current->pos_pole_index, ps_HB_current->neg_pole_index);

            if (ps_HB_current->hv_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, ps_HB_current->sequence_delay_ms, ps_HB_current->hv_on_ms, ps_HB_current->hv_off_ms, ps_HB_current->hv_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_HV_1_STATE;
            }
            else if (ps_HB_current->hv_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, ps_HB_current->sequence_delay_ms, ps_HB_current->hv_on_ms, ps_HB_current->hv_off_ms, ps_HB_current->hv_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRDIGE_HV_2_STATE;
            }
            else if (ps_HB_current->lv_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, ps_HB_current->sequence_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_1_STATE;
            }
            else if (ps_HB_current->lv_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, ps_HB_current->sequence_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_neg_count);
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
            if (ps_HB_current->hv_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, ps_HB_current->hv_delay_ms, ps_HB_current->hv_on_ms, ps_HB_current->hv_off_ms, ps_HB_current->hv_neg_count);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRDIGE_HV_2_STATE;
            }
            else if (ps_HB_current->lv_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, ps_HB_current->pulse_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_1_STATE;
            }
            else if (ps_HB_current->lv_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, ps_HB_current->pulse_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_neg_count);
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
            if (ps_HB_current->lv_pos_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);

                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_pos_pole, ps_HB_current->pulse_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_pos_count);
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_PULSE);

                H_Bridge_State = H_BRIDGE_LV_1_STATE;
            }
            else if (ps_HB_current->lv_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, ps_HB_current->pulse_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_neg_count);
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
            if (ps_HB_current->lv_neg_count != 0)
            {
                V_Switch_Set_Mode(V_SWITCH_MODE_LV_ON);
            
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_LS_ON);
                H_Bridge_Set_Pulse_Timing(&HB_neg_pole, ps_HB_current->lv_delay_ms, ps_HB_current->lv_on_ms, ps_HB_current->lv_off_ms, ps_HB_current->lv_neg_count);
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
            if (H_Bridge_Set_Next_Sequence() == false)
            {
                is_h_bridge_enable = false;

                ps_FSP_TX->CMD = FSP_CMD_SET_PULSE_CONTROL;
                ps_FSP_TX->Payload.set_pulse_control.State = is_h_bridge_enable;

                fsp_print(2);
            }
            else
            {
                H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
                H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
            }
            
            H_Bridge_State = H_BRIDGE_STOP_STATE;
        }
        break;

    default:
        break;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool H_Bridge_Set_Next_Sequence(void)
{
    uint8_t next_sequence_index = current_sequence_index + 1;
    
    if (next_sequence_index == 10)
    {
        current_sequence_index = 0;
        ps_HB_current = &HB_sequence_array[0];
        return 0;
    }
    
    if ((HB_sequence_array[next_sequence_index].is_setted & (1 << 7)) == false)
    {
        current_sequence_index = 0;
        ps_HB_current = &HB_sequence_array[0];
        return 0;
    }
    
    ps_HB_current = &HB_sequence_array[next_sequence_index];
    current_sequence_index = next_sequence_index;
    return 1;
}

static void fsp_print(uint8_t packet_length)
{
    s_FSP_TX_Packet.sod 		= FSP_PKT_SOD;
    s_FSP_TX_Packet.src_adr 	= fsp_my_adr;
    s_FSP_TX_Packet.dst_adr 	= FSP_ADR_GPC;
    s_FSP_TX_Packet.length 	    = packet_length;
    s_FSP_TX_Packet.type 		= FSP_PKT_TYPE_CMD_W_DATA;
    s_FSP_TX_Packet.eof 		= FSP_PKT_EOF;
    s_FSP_TX_Packet.crc16 		= crc16_CCITT(FSP_CRC16_INITIAL_VALUE, &s_FSP_TX_Packet.src_adr, s_FSP_TX_Packet.length + 4);

    uint8_t encoded_frame[20] = { 0 };
    uint8_t frame_len;
    fsp_encode(&s_FSP_TX_Packet, encoded_frame, &frame_len);

    UART_FSP(&GPC_UART, (char*)encoded_frame, frame_len);
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

//uint16_t    pulse_delay_ms              = 15;
//
//uint8_t     hv_pulse_pos_count          = 5;
//uint8_t     hv_pulse_neg_count          = 6;
//uint8_t     hv_delay_ms                 = 5;
//uint8_t     hv_on_time_ms               = 5;
//uint8_t     hv_off_time_ms              = 15;
//
//uint8_t     lv_pulse_pos_count          = 7;
//uint8_t     lv_pulse_neg_count          = 8;
//uint8_t     lv_delay_ms                 = 10;
//uint16_t    lv_on_time_ms               = 50;
//uint16_t    lv_off_time_ms              = 90;
