/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "h_bridge_sequence.h"
#include "h_bridge_task.h"
#include "h_bridge_driver.h"
#include "v_switch_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define SEQUENCE_MAX_INDEX 10

#define SD_DUTY_MIN \
((APB1_TIMER_CLK / 1000000) * 100) / (p_HB_task_data->HB_pole_pulse.PWM.Prescaler)

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// static void H_Bridge_Calculate_Timing(
//                                       H_Bridge_Task_data_typedef* p_HB_task_data,
//                                       V_Switch_mode               _VS_mode_,

//                                       uint16_t Set_delay_time_ms, 
//                                       uint16_t Set_on_time_ms, 
//                                       uint16_t Set_off_time_ms, 
//                                       uint16_t Set_pulse_count
//                                     );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef enum
{
    H_BRIDGE_HV_1_STATE,
    H_BRIDGE_HV_2_STATE,
    H_BRIDGE_LV_1_STATE,
    H_BRIDGE_LV_2_STATE,
} H_Bridge_Sequence_Process_State_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static H_Bridge_Sequence_Process_State_typedef H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
H_Bridge_task_typedef  HB_sequence_default =
{
	.is_setted = 0,

	.sequence_delay_ms = 1,

	.pos_pole_index = 0,
	.neg_pole_index = 5,

	.pulse_delay_ms = 15,

	.hv_pos_count = 5,
	.hv_neg_count = 6,

	.hv_delay_ms = 5,

	.hv_pos_on_ms = 5,
	.hv_pos_off_ms = 15,
    .hv_neg_on_ms = 5,
	.hv_neg_off_ms = 15,

	.lv_pos_count = 7,
	.lv_neg_count = 8,

	.lv_delay_ms = 10,
    
	.lv_pos_on_ms = 50,
	.lv_pos_off_ms = 90,
    .lv_neg_on_ms = 50,
	.lv_neg_off_ms = 90,
};

H_Bridge_task_typedef  HB_sequence_array[SEQUENCE_MAX_INDEX] = {0};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void H_Bridge_Sequence_Init(void)
{
    for (uint8_t i = 0; i < SEQUENCE_MAX_INDEX; i++)
    {
        HB_sequence_array[i] = HB_sequence_default;
    }
}

void H_Bridge_Process_Sequence_Array(void)
{

uint8_t sequence_index;

for (sequence_index = 0; ((HB_sequence_array[sequence_index].is_setted & (1 << 7)) == (1 << 7)) && (sequence_index < SEQUENCE_MAX_INDEX) ; sequence_index++)
{

HB_Task_data[sequence_index].is_setted = true;

if (H_Bridge_Sequence_Process_State == H_BRIDGE_HV_1_STATE)
{
    if (HB_sequence_array[sequence_index].hv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[0].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[0].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[0],
                                   V_SWITCH_MODE_HV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].hv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].hv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].hv_pos_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_2_STATE;

    }
    else if (HB_sequence_array[sequence_index].hv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[1],
                                   V_SWITCH_MODE_HV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_1_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[2],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_2_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].sequence_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}

if (H_Bridge_Sequence_Process_State == H_BRIDGE_HV_2_STATE )
{
    if (HB_sequence_array[sequence_index].hv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[1].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[1],
                                   V_SWITCH_MODE_HV_ON,

                                   HB_sequence_array[sequence_index].hv_delay_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].hv_neg_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_1_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[2],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_2_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}

if (H_Bridge_Sequence_Process_State == H_BRIDGE_LV_1_STATE )
{
    if (HB_sequence_array[sequence_index].lv_pos_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[2].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].pos_pole_index,
                          HB_sequence_array[sequence_index].neg_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[2],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_on_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_off_ms, 
                                   HB_sequence_array[sequence_index].lv_pos_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_LV_2_STATE;
    }
    else if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].pulse_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}

if (H_Bridge_Sequence_Process_State == H_BRIDGE_LV_2_STATE )
{
    if (HB_sequence_array[sequence_index].lv_neg_count != 0)
    {
        H_Bridge_Set_Pole(
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_pulse,
                          &HB_Task_data[sequence_index].task_data[3].HB_pole_ls_on,
                          HB_sequence_array[sequence_index].neg_pole_index,
                          HB_sequence_array[sequence_index].pos_pole_index
                         );

        H_Bridge_Calculate_Timing(
                                   &HB_Task_data[sequence_index].task_data[3],
                                   V_SWITCH_MODE_LV_ON,

                                   HB_sequence_array[sequence_index].lv_delay_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_on_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_off_ms, 
                                   HB_sequence_array[sequence_index].lv_neg_count
                                 );

        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
    else
    {   
        H_Bridge_Sequence_Process_State = H_BRIDGE_HV_1_STATE;
        continue;
    }
}
}

for (; sequence_index < SEQUENCE_MAX_INDEX; sequence_index++)
{
    HB_sequence_array[sequence_index].is_setted = 0;
}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// static void H_Bridge_Calculate_Timing(
//                                       H_Bridge_Task_data_typedef* p_HB_task_data,
//                                       V_Switch_mode               _VS_mode_,

//                                       uint16_t Set_delay_time_ms, 
//                                       uint16_t Set_on_time_ms, 
//                                       uint16_t Set_off_time_ms, 
//                                       uint16_t Set_pulse_count
//                                     )
// {
//     p_HB_task_data->VS_mode = _VS_mode_;

//     p_HB_task_data->HB_pole_ls_on.PWM.Mode     = LL_TIM_OCMODE_FORCED_INACTIVE;
//     p_HB_task_data->HB_pole_ls_on.PWM.Polarity = LL_TIM_OCPOLARITY_HIGH;

//     p_HB_task_data->HB_pole_pulse.PWM.Mode     = LL_TIM_OCMODE_FORCED_INACTIVE;
//     p_HB_task_data->HB_pole_pulse.PWM.Polarity = LL_TIM_OCPOLARITY_HIGH;

//     /* 
//     * @brief Calculate the maximum of Set_delay_time_ms, Set_on_time_ms, and Set_off_time_ms.
//     * @details 
//     *   - If 'Set_delay_time_ms' is greater than 'Set_on_time_ms', check if 'Set_delay_time_ms' is also greater than 'Set_off_time_ms'.
//     *     If true, 'Set_delay_time_ms' is the maximum; otherwise, 'Set_off_time_ms' is the maximum.
//     *   - If 'Set_delay_time_ms' is not greater than 'Set_on_time_ms', then check if 'Set_on_time_ms' is greater than 'Set_off_time_ms'.
//     *     If true, 'Set_on_time_ms' is the maximum; otherwise, 'Set_off_time_ms' is the maximum.
//     */
//     uint16_t max_time_ms =  (Set_delay_time_ms > Set_on_time_ms) ? 
//                             ((Set_delay_time_ms > Set_off_time_ms) ? Set_delay_time_ms : Set_off_time_ms) : 
//                             ((Set_on_time_ms > Set_off_time_ms) ? Set_on_time_ms : Set_off_time_ms);

//     uint16_t min_time_ms =  (Set_on_time_ms < Set_off_time_ms) ? Set_on_time_ms : Set_off_time_ms;
     
//     p_HB_task_data->HB_pole_pulse.PWM.Prescaler = (((APB1_TIMER_CLK / 1000) * max_time_ms) / (UINT16_MAX)) + 1;
//     p_HB_task_data->HB_pole_pulse.PWM.Prescaler = (p_HB_task_data->HB_pole_pulse.PWM.Prescaler > UINT16_MAX) ? UINT16_MAX : p_HB_task_data->HB_pole_pulse.PWM.Prescaler;

//     p_HB_task_data->HB_pole_pulse.PWM.Duty      = ((APB1_TIMER_CLK / 1000) * min_time_ms) / (p_HB_task_data->HB_pole_pulse.PWM.Prescaler * 100);
//     p_HB_task_data->HB_pole_pulse.PWM.Duty      = (p_HB_task_data->HB_pole_pulse.PWM.Duty > SD_DUTY_MIN) ? p_HB_task_data->HB_pole_pulse.PWM.Duty : SD_DUTY_MIN;

//     p_HB_task_data->HB_pole_ls_on.PWM.Prescaler = 1;

//     p_HB_task_data->HB_pole_pulse.delay_time_ms   = Set_delay_time_ms;
//     p_HB_task_data->HB_pole_pulse.on_time_ms      = Set_on_time_ms;
//     p_HB_task_data->HB_pole_pulse.off_time_ms     = Set_off_time_ms;
//     p_HB_task_data->HB_pole_pulse.set_pulse_count = Set_pulse_count;
//     p_HB_task_data->HB_pole_pulse.pulse_count     = 0;

//     p_HB_task_data->is_setted                      = true;
// }

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */