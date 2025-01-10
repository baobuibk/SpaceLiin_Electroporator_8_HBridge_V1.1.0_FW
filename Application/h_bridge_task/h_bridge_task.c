/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "app.h"
#include "stm32f4xx_ll_gpio.h"

#include "h_bridge_task.h"
#include "h_bridge_driver.h"
#include "v_switch_driver.h"

#include "fsp_frame.h"
#include "crc.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum
{
    H_BRIDGE_INITIAL_SET_STATE,
    H_BRIDGE_CHECK_PULSE_STATE,
} H_Bridge_State_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static H_Bridge_State_typedef H_Bridge_State = H_BRIDGE_INITIAL_SET_STATE;

static H_Bridge_Task_data_typedef* ps_current_HB_timing_data;
static H_Bridge_Task_typedef*      ps_current_HB_Task_data;

static uint8_t current_HB_timing_data_index = 0;
static uint8_t current_HB_Task_data_index  = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// static void H_Bridge_Update_Pulse_Timing(H_Bridge_Task_data_typedef* p_HB_Timing_Data);
static bool H_Bridge_Set_Next_HB_Task_Data(void);
static void fsp_print(uint8_t packet_length);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// H_Bridge_Task_typedef HB_Task_data[10];

bool is_h_bridge_enable = false;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Task Init :::::::: */
void H_Bridge_Task_Init(void)
{
    current_HB_Task_data_index  = 0;
    current_HB_timing_data_index = 0;
}

/* :::::::::: H Bridge Task ::::::::::::: */
void H_Bridge_Task(void*)
{

if(is_h_bridge_enable == false)
{
    V_Switch_Set_Mode(V_SWITCH_MODE_ALL_OFF);
    //V_Switch_Set_Mode(V_SWITCH_MODE_HV_ON);

    H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
    H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
    LL_GPIO_ResetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

    current_HB_Task_data_index  = 0;
    current_HB_timing_data_index = 0;
    H_Bridge_State = H_BRIDGE_INITIAL_SET_STATE;

    ps_FSP_TX->CMD = FSP_CMD_SET_PULSE_CONTROL;
    ps_FSP_TX->Payload.set_pulse_control.State = 0;
    fsp_print(2);

    SchedulerTaskDisable(0);
}

switch (H_Bridge_State)
{
case H_BRIDGE_INITIAL_SET_STATE:
{
    current_HB_Task_data_index   = 0;
    current_HB_timing_data_index = 0;
 
    if (H_Bridge_Set_Next_HB_Task_Data() == false)
    {
        is_h_bridge_enable = false;
        break;
    }
    
    LL_GPIO_SetOutputPin(PULSE_LED_PORT,PULSE_LED_PIN);

    V_Switch_Set_Mode(ps_current_HB_timing_data->VS_mode);

    H_Bridge_Set_Mode(&ps_current_HB_timing_data->HB_pole_ls_on, H_BRIDGE_MODE_LS_ON);
    H_Bridge_Set_Mode(&ps_current_HB_timing_data->HB_pole_pulse, H_BRIDGE_MODE_PULSE);

    H_Bridge_State = H_BRIDGE_CHECK_PULSE_STATE;

    break;
}

case H_BRIDGE_CHECK_PULSE_STATE:
{
    if(ps_current_HB_timing_data->HB_pole_pulse.pulse_count < ((ps_current_HB_timing_data->HB_pole_pulse.set_pulse_count * 2) + 1))
    {
        break;
    }

    if (H_Bridge_Set_Next_HB_Task_Data() == false)
    {
        is_h_bridge_enable = false;
        break;
    }

    V_Switch_Set_Mode(ps_current_HB_timing_data->VS_mode);

    H_Bridge_Set_Mode(&ps_current_HB_timing_data->HB_pole_ls_on, H_BRIDGE_MODE_LS_ON);
    H_Bridge_Set_Mode(&ps_current_HB_timing_data->HB_pole_pulse, H_BRIDGE_MODE_PULSE);

    break;
}
default:
    break;
}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool H_Bridge_Set_Next_HB_Task_Data(void)
{
    if (HB_Task_data[current_HB_Task_data_index].is_setted == false)
    {
        return 0;
    }
    
    for (; (ps_current_HB_Task_data->task_data[current_HB_timing_data_index].is_setted == false) && (current_HB_timing_data_index < 4); current_HB_timing_data_index++)
    {
        ;
    }

    if (current_HB_timing_data_index < 4)
    {
        ps_current_HB_timing_data = &ps_current_HB_Task_data->task_data[current_HB_timing_data_index];

        return 1;
    }
    
    // if (current_HB_timing_data_index >= 4)
    uint8_t next_HB_Task_data_index = current_HB_Task_data_index + 1;

    if (HB_Task_data[next_HB_Task_data_index].is_setted == false)
    {
        return 0;
    }

    current_HB_Task_data_index   = next_HB_Task_data_index;
    current_HB_timing_data_index = 0;

    for (; (ps_current_HB_Task_data->task_data[current_HB_timing_data_index].is_setted == false) && (current_HB_timing_data_index < 4); current_HB_timing_data_index++)
    {
        ;
    }

    ps_current_HB_Task_data    = &HB_Task_data[current_HB_Task_data_index];
    ps_current_HB_timing_data  = &ps_current_HB_Task_data->task_data[current_HB_timing_data_index];
    
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
// static void H_Bridge_Update_Pulse_Timing(H_Bridge_Task_data_typedef* p_HB_Timing_Data)
// {
//     LL_TIM_DisableIT_UPDATE(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);
//     LL_TIM_DisableCounter(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);
//     LL_TIM_ClearFlag_UPDATE(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);
     
//     p_HB_Timing_Data->p_HB_pole_pulse->PWM.Prescaler = p_HB_Timing_Data->pulse_prescaler;

//     p_HB_Timing_Data->p_HB_pole_pulse->PWM.Duty      = p_HB_Timing_Data->pulse_duty;

//     p_HB_Timing_Data->p_HB_pole_pulse->delay_time_ms = p_HB_Timing_Data->delay_time_ms;

//     p_HB_Timing_Data->p_HB_pole_pulse->on_time_ms      = p_HB_Timing_Data->on_time_ms;
//     p_HB_Timing_Data->p_HB_pole_pulse->off_time_ms     = p_HB_Timing_Data->off_time_ms;

//     p_HB_Timing_Data->p_HB_pole_pulse->set_pulse_count = p_HB_Timing_Data->set_pulse_count;
//     p_HB_Timing_Data->p_HB_pole_pulse->pulse_count     = 0;

//     LL_TIM_SetPrescaler(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx, p_HB_Timing_Data->p_HB_pole_pulse->PWM.Prescaler);
//     LL_TIM_GenerateEvent_UPDATE(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);
//     LL_TIM_ClearFlag_UPDATE(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);

//     LL_TIM_EnableIT_UPDATE(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);
//     LL_TIM_EnableCounter(p_HB_Timing_Data->p_HB_pole_pulse->PWM.TIMx);
// }

// static bool H_Bridge_Set_Next_Timing_Data(void)
// {
//     for (; (ps_current_HB_Task_data->task_data[current_HB_timing_data_index].is_setted == false) && (current_HB_timing_data_index < 4); current_HB_timing_data_index++)
//     {
//         ;
//     }
    
//     if (current_HB_timing_data_index >= 4)
//     {
//         current_HB_timing_data_index = 0;

//         ps_current_HB_timing_data = &ps_current_HB_Task_data->task_data[0];
//         return 0;
//     }

//     ps_current_HB_timing_data = &ps_current_HB_Task_data->task_data[current_HB_timing_data_index];

//     return 1;
// }

// static bool H_Bridge_Set_Next_Pulse_Data(void)
// {
//     current_HB_Task_data_index++;

//     bool out_of_pulse_data_condition = (current_HB_Task_data_index >= 10) || (HB_Task_data[current_HB_Task_data_index].is_setted == false);
    
//     if (out_of_pulse_data_condition == true)
//     {
//         current_HB_Task_data_index = 0;
        
//         ps_current_HB_Task_data   = &HB_Task_data[0];
//         ps_current_HB_timing_data = &ps_current_HB_Task_data->task_data[0];
//         return 0;
//     }

//     ps_current_HB_Task_data  = &HB_Task_data[current_HB_Task_data_index];
//     ps_current_HB_timing_data = &ps_current_HB_Task_data->task_data[0];
    
//     return 1;
// }