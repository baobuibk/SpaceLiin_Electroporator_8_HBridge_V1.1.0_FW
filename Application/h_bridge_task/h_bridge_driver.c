/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
//#include "app.h"
#include "board.h"

#include "h_bridge_driver.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#define SD_DUTY_MIN \
((APB1_TIMER_CLK / 1000000) * 100) / (p_HB_task_data->HB_pole_pulse.PWM.Prescaler)
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void HB_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now);
__STATIC_INLINE void HB_Set_OC(PWM_TypeDef *PWMx, uint32_t _OC, bool apply_now);
__STATIC_INLINE void HB_Set_Freq(PWM_TypeDef *PWMx, float _Freq, bool apply_now);
__STATIC_INLINE void HB_Set_ARR(PWM_TypeDef *PWMx, uint32_t _ARR, bool apply_now);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uint32_t HB_PWM_channel_array[8] =
{
    H_BRIDGE_SD0_CHANNEL,
    H_BRIDGE_SD1_CHANNEL,
    H_BRIDGE_SD2_CHANNEL,
    H_BRIDGE_SD3_CHANNEL,
    H_BRIDGE_SD4_CHANNEL,
    H_BRIDGE_SD5_CHANNEL,
    H_BRIDGE_SD6_CHANNEL,
    H_BRIDGE_SD7_CHANNEL,
};

uint32_t HB_pin_array[8] =
{
    H_BRIDGE_HIN0_PIN,
    H_BRIDGE_HIN1_PIN,
    H_BRIDGE_HIN2_PIN,
    H_BRIDGE_HIN3_PIN,
    H_BRIDGE_HIN4_PIN,
    H_BRIDGE_HIN5_PIN,
    H_BRIDGE_HIN6_PIN,
    H_BRIDGE_HIN7_PIN,
};

bool HB_pin_state_array[8] =
{
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

//uint8_t HB_pos_pole_index = 1;
//uint8_t HB_neg_pole_index = 6;

H_Bridge_typdef HB_pos_pole =
{
    .PWM =
    {
        .Mode           = LL_TIM_OCMODE_FORCED_INACTIVE,
        .Polarity       = LL_TIM_OCPOLARITY_HIGH,
    },
    .Port               = H_BRIDGE_HIN0_7_PORT,
    .Mode               = H_BRIDGE_MODE_LS_ON,
    .delay_time_ms      = 0,
    .on_time_ms         = 0,
    .off_time_ms        = 0,
    .set_pulse_count    = 0,
    .pulse_count        = 0,
    .is_setted          = false,
};

H_Bridge_typdef HB_neg_pole =
{
    .PWM =
    {
        .Mode           = LL_TIM_OCMODE_FORCED_INACTIVE,
        .Polarity       = LL_TIM_OCPOLARITY_HIGH,
    },
    .Port               = H_BRIDGE_HIN0_7_PORT,
    .Mode               = H_BRIDGE_MODE_LS_ON,
    .delay_time_ms      = 0,
    .on_time_ms         = 0,
    .off_time_ms        = 0,
    .set_pulse_count    = 0,
    .pulse_count        = 0,
    .is_setted          = false,
};

H_Bridge_typdef* p_HB_SD_0_3_IRQn = &HB_pos_pole;
H_Bridge_typdef* p_HB_SD_4_7_IRQn = &HB_neg_pole;

H_Bridge_Task_typedef HB_Task_data[10];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: H Bridge Driver Init :::::::: */
void H_Bridge_Driver_Init(void)
{   
    for (uint8_t i = 0; i < 4; i++)
    {
        LL_TIM_OC_SetMode(H_BRIDGE_SD0_3_HANDLE, HB_PWM_channel_array[i],  LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetPolarity(H_BRIDGE_SD0_3_HANDLE, HB_PWM_channel_array[i], LL_TIM_OCPOLARITY_HIGH);
        LL_TIM_OC_EnablePreload(H_BRIDGE_SD0_3_HANDLE, HB_PWM_channel_array[i]);
        LL_TIM_CC_EnableChannel(H_BRIDGE_SD0_3_HANDLE, HB_PWM_channel_array[i]);
        LL_GPIO_ResetOutputPin(H_BRIDGE_HIN0_7_PORT, HB_pin_array[i]);
    }

    for (uint8_t i = 4; i < 8; i++)
    {
        LL_TIM_OC_SetMode(H_BRIDGE_SD4_7_HANDLE, HB_PWM_channel_array[i],  LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_TIM_OC_SetPolarity(H_BRIDGE_SD4_7_HANDLE, HB_PWM_channel_array[i], LL_TIM_OCPOLARITY_HIGH);
        LL_TIM_OC_EnablePreload(H_BRIDGE_SD4_7_HANDLE, HB_PWM_channel_array[i]);
        LL_TIM_CC_EnableChannel(H_BRIDGE_SD4_7_HANDLE, HB_PWM_channel_array[i]);
        LL_GPIO_ResetOutputPin(H_BRIDGE_HIN0_7_PORT, HB_pin_array[i]);
    }

    LL_TIM_SetOffStates(H_BRIDGE_SD0_3_HANDLE, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_SetOffStates(H_BRIDGE_SD4_7_HANDLE, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);

    LL_TIM_DisableAutomaticOutput(H_BRIDGE_SD0_3_HANDLE);
    LL_TIM_DisableAutomaticOutput(H_BRIDGE_SD4_7_HANDLE);

    LL_TIM_EnableAllOutputs(H_BRIDGE_SD0_3_HANDLE);
    LL_TIM_EnableAllOutputs(H_BRIDGE_SD4_7_HANDLE);

    LL_TIM_SetPrescaler(H_BRIDGE_SD0_3_HANDLE, 1199);
    LL_TIM_SetPrescaler(H_BRIDGE_SD4_7_HANDLE, 1199);

    LL_TIM_EnableARRPreload(H_BRIDGE_SD0_3_HANDLE);
    LL_TIM_EnableARRPreload(H_BRIDGE_SD4_7_HANDLE);

    LL_TIM_DisableIT_UPDATE(H_BRIDGE_SD0_3_HANDLE);
    LL_TIM_SetUpdateSource(H_BRIDGE_SD0_3_HANDLE, LL_TIM_UPDATESOURCE_REGULAR);
    LL_TIM_GenerateEvent_UPDATE(H_BRIDGE_SD0_3_HANDLE);

    LL_TIM_DisableIT_UPDATE(H_BRIDGE_SD4_7_HANDLE);
    LL_TIM_SetUpdateSource(H_BRIDGE_SD4_7_HANDLE, LL_TIM_UPDATESOURCE_REGULAR);
    LL_TIM_GenerateEvent_UPDATE(H_BRIDGE_SD4_7_HANDLE);

    LL_TIM_EnableCounter(H_BRIDGE_SD0_3_HANDLE);
    LL_TIM_EnableCounter(H_BRIDGE_SD4_7_HANDLE);
}

void H_Bridge_Set_Pole(H_Bridge_typdef* p_HB_pos_pole, H_Bridge_typdef* p_HB_neg_pole, uint8_t pos_pole_index, uint8_t neg_pole_index)
{
    //Set pole for positive pole
    p_HB_pos_pole->Port        = H_BRIDGE_HIN0_7_PORT;
    p_HB_pos_pole->Pin         = &HB_pin_array[pos_pole_index];
    p_HB_pos_pole->Pin_State   = &HB_pin_state_array[pos_pole_index];

    (pos_pole_index < 4) ? (p_HB_pos_pole->PWM.TIMx = H_BRIDGE_SD0_3_HANDLE) : (p_HB_pos_pole->PWM.TIMx = H_BRIDGE_SD4_7_HANDLE);
    p_HB_pos_pole->PWM.Channel = HB_PWM_channel_array[pos_pole_index];

    //Set pole for negative pole
    p_HB_neg_pole->Port        = H_BRIDGE_HIN0_7_PORT;
    p_HB_neg_pole->Pin         = &HB_pin_array[neg_pole_index];
    p_HB_neg_pole->Pin_State   = &HB_pin_state_array[neg_pole_index];
    
    (neg_pole_index < 4) ? (p_HB_neg_pole->PWM.TIMx = H_BRIDGE_SD0_3_HANDLE) : (p_HB_neg_pole->PWM.TIMx = H_BRIDGE_SD4_7_HANDLE);
    p_HB_neg_pole->PWM.Channel = HB_PWM_channel_array[neg_pole_index];
}

void H_Bridge_Set_Mode(H_Bridge_typdef* H_Bridge_x, H_Bridge_mode SetMode)
{
    LL_TIM_DisableIT_UPDATE(H_Bridge_x->PWM.TIMx);
    LL_TIM_DisableCounter(H_Bridge_x->PWM.TIMx);
    LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);
    (H_Bridge_x->PWM.TIMx ==  H_BRIDGE_SD0_3_HANDLE) ? (p_HB_SD_0_3_IRQn = H_Bridge_x) : (p_HB_SD_4_7_IRQn = H_Bridge_x);
    H_Bridge_x->Mode = SetMode;

    switch (SetMode)
    {
    case H_BRIDGE_MODE_PULSE:
        LL_TIM_SetPrescaler(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Prescaler);
        LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->PWM.TIMx);

        HB_Set_Freq(&H_Bridge_x->PWM, (1000.0 / (float)H_Bridge_x->delay_time_ms), 1);
        HB_Set_OC(&H_Bridge_x->PWM, 0, 1);

        //SD timing
        LL_TIM_OC_SetMode(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Channel, LL_TIM_OCMODE_PWM2);
        HB_Set_Freq(&H_Bridge_x->PWM, (1000.0 / (float)H_Bridge_x->on_time_ms), 0); //Period = 1ms
        HB_Set_OC(&H_Bridge_x->PWM, H_Bridge_x->PWM.Duty, 0); //Duty = 50us
        
        *H_Bridge_x->Pin_State = 1;
        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

        break;
    case H_BRIDGE_MODE_HS_ON:
    case H_BRIDGE_MODE_LS_ON:
        LL_TIM_SetPrescaler(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Prescaler);
        LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->PWM.TIMx);
        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

        LL_TIM_OC_SetMode(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Channel, LL_TIM_OCMODE_PWM2);
        HB_Set_Freq(&H_Bridge_x->PWM, 1000.0, 1); //Period = 1ms
        HB_Set_Duty(&H_Bridge_x->PWM, 10, 1); //Duty = 100us

        *H_Bridge_x->Pin_State = 1;
        H_Bridge_x->pulse_count = 0;
        H_Bridge_x->delay_time_ms = 0;

        break;
    case H_BRIDGE_MODE_FLOAT:
        LL_TIM_OC_SetMode(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        HB_Set_ARR(&H_Bridge_x->PWM, 0, 1); //Period = 1ms
        HB_Set_OC(&H_Bridge_x->PWM, 0, 1); //Duty = 50us
        H_Bridge_x->pulse_count = 0;
        H_Bridge_x->delay_time_ms = 0;

        LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

        break;
    
    default:
        break;
    }

    LL_TIM_EnableCounter(H_Bridge_x->PWM.TIMx);
    LL_TIM_EnableIT_UPDATE(H_Bridge_x->PWM.TIMx);
}

void H_Bridge_Calculate_Timing(
                               H_Bridge_Task_data_typedef* p_HB_task_data,
                               V_Switch_mode               _VS_mode_,

                               uint16_t Set_delay_time_ms, 
                               uint16_t Set_on_time_ms, 
                               uint16_t Set_off_time_ms, 
                               uint16_t Set_pulse_count
                              )
{
    p_HB_task_data->VS_mode = _VS_mode_;

    /* 
    * @brief Calculate the maximum of Set_delay_time_ms, Set_on_time_ms, and Set_off_time_ms.
    * @details 
    *   - If 'Set_delay_time_ms' is greater than 'Set_on_time_ms', check if 'Set_delay_time_ms' is also greater than 'Set_off_time_ms'.
    *     If true, 'Set_delay_time_ms' is the maximum; otherwise, 'Set_off_time_ms' is the maximum.
    *   - If 'Set_delay_time_ms' is not greater than 'Set_on_time_ms', then check if 'Set_on_time_ms' is greater than 'Set_off_time_ms'.
    *     If true, 'Set_on_time_ms' is the maximum; otherwise, 'Set_off_time_ms' is the maximum.
    */
    uint16_t max_time_ms =  (Set_delay_time_ms > Set_on_time_ms) ? 
                            ((Set_delay_time_ms > Set_off_time_ms) ? Set_delay_time_ms : Set_off_time_ms) : 
                            ((Set_on_time_ms > Set_off_time_ms) ? Set_on_time_ms : Set_off_time_ms);

    uint16_t min_time_ms =  (Set_on_time_ms < Set_off_time_ms) ? Set_on_time_ms : Set_off_time_ms;
     
    p_HB_task_data->HB_pole_pulse.PWM.Prescaler = (((APB1_TIMER_CLK / 1000) * max_time_ms) / (UINT16_MAX)) + 1;
    p_HB_task_data->HB_pole_pulse.PWM.Prescaler = (p_HB_task_data->HB_pole_pulse.PWM.Prescaler > UINT16_MAX) ? UINT16_MAX : p_HB_task_data->HB_pole_pulse.PWM.Prescaler;

    p_HB_task_data->HB_pole_pulse.PWM.Duty      = ((APB1_TIMER_CLK / 1000) * min_time_ms) / (p_HB_task_data->HB_pole_pulse.PWM.Prescaler * 100);
    p_HB_task_data->HB_pole_pulse.PWM.Duty      = (p_HB_task_data->HB_pole_pulse.PWM.Duty > SD_DUTY_MIN) ? p_HB_task_data->HB_pole_pulse.PWM.Duty : SD_DUTY_MIN;

    p_HB_task_data->HB_pole_ls_on.PWM.Prescaler = 1;

    p_HB_task_data->HB_pole_pulse.delay_time_ms   = Set_delay_time_ms;
    p_HB_task_data->HB_pole_pulse.on_time_ms      = Set_on_time_ms;
    p_HB_task_data->HB_pole_pulse.off_time_ms     = Set_off_time_ms;
    p_HB_task_data->HB_pole_pulse.set_pulse_count = Set_pulse_count;
    p_HB_task_data->HB_pole_pulse.pulse_count     = 0;

    p_HB_task_data->is_setted                     = true;
}

void H_Bridge_Kill(void)
{
    H_Bridge_Set_Mode(&HB_pos_pole, H_BRIDGE_MODE_FLOAT);
    H_Bridge_Set_Mode(&HB_neg_pole, H_BRIDGE_MODE_FLOAT);
}

/* ::::H_Bridge SD Interupt Handle:::: */
void H_Bridge_SD_Interupt_Handle(H_Bridge_typdef* p_HB_SD_IRQn)
{
    if(LL_TIM_IsActiveFlag_UPDATE(p_HB_SD_IRQn->PWM.TIMx) == true)
    {
        LL_TIM_ClearFlag_UPDATE(p_HB_SD_IRQn->PWM.TIMx);

        switch (p_HB_SD_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_IRQn->pulse_count++;

            if (p_HB_SD_IRQn->pulse_count >= ((p_HB_SD_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(p_HB_SD_IRQn->PWM.TIMx);
                return;
            }
            
            if (*p_HB_SD_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_IRQn->Port, *p_HB_SD_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_IRQn->PWM, 1000.0 / (float)p_HB_SD_IRQn->off_time_ms, 0);
                *p_HB_SD_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_IRQn->Port, *p_HB_SD_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_IRQn->PWM, 1000.0 / (float)p_HB_SD_IRQn->on_time_ms, 0);
                *p_HB_SD_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_IRQn->Port, *p_HB_SD_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_IRQn->PWM.TIMx, p_HB_SD_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_IRQn->Port, *p_HB_SD_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_IRQn->PWM.TIMx, p_HB_SD_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_IRQn->PWM.TIMx);
            break;
        
        default:
            break;
        }

        p_HB_SD_IRQn->is_setted = true;
    }
}

/* ::::H_Bridge SD0_3 Interupt Handle:::: */
void H_Bridge_SD0_3_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_SD0_3_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_SD0_3_HANDLE);

        switch (p_HB_SD_0_3_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_0_3_IRQn->pulse_count++;

            if (p_HB_SD_0_3_IRQn->pulse_count >= ((p_HB_SD_0_3_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(H_BRIDGE_SD0_3_HANDLE);
                return;
            }

            if (*p_HB_SD_0_3_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_0_3_IRQn->PWM, 1000.0 / (float)p_HB_SD_0_3_IRQn->off_time_ms, 0);
                *p_HB_SD_0_3_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_0_3_IRQn->PWM, 1000.0 / (float)p_HB_SD_0_3_IRQn->on_time_ms, 0);
                *p_HB_SD_0_3_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_0_3_IRQn->PWM.TIMx, p_HB_SD_0_3_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_0_3_IRQn->PWM.TIMx, p_HB_SD_0_3_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;

        default:
            break;
        }

        p_HB_SD_0_3_IRQn->is_setted = true;
    }
}

/* ::::H_Bridge SD4_7 Interupt Handle:::: */
void H_Bridge_SD4_7_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_SD4_7_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_SD4_7_HANDLE);

        switch (p_HB_SD_4_7_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_4_7_IRQn->pulse_count++;

            if (p_HB_SD_4_7_IRQn->pulse_count >= ((p_HB_SD_4_7_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(H_BRIDGE_SD4_7_HANDLE);
                return;
            }

            if (*p_HB_SD_4_7_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_4_7_IRQn->PWM, 1000.0 / (float)p_HB_SD_4_7_IRQn->off_time_ms, 0);
                *p_HB_SD_4_7_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_4_7_IRQn->PWM, 1000.0 / (float)p_HB_SD_4_7_IRQn->on_time_ms, 0);
                *p_HB_SD_4_7_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_4_7_IRQn->PWM.TIMx, p_HB_SD_4_7_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_4_7_IRQn->PWM.TIMx, p_HB_SD_4_7_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;

        default:
            break;
        }

        p_HB_SD_4_7_IRQn->is_setted = true;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void HB_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Limit the duty to 100
    if (_Duty > 100)
      return;

    // Set PWM DUTY for channel 1
    _Duty = (PWMx->Freq * (_Duty / 100.0));
    switch (PWMx->Channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(PWMx->TIMx, _Duty);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(PWMx->TIMx, _Duty);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(PWMx->TIMx, _Duty);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(PWMx->TIMx, _Duty);
        break;

    default:
        break;
    }

    LL_TIM_EnableUpdateEvent(PWMx->TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(PWMx->TIMx))
        {
            LL_TIM_DisableIT_UPDATE(PWMx->TIMx);
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
            LL_TIM_EnableIT_UPDATE(PWMx->TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
        }
    }
}

__STATIC_INLINE void HB_Set_OC(PWM_TypeDef *PWMx, uint32_t _OC, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Set PWM DUTY for channel 1
    //PWMx->Duty = _OC;
    switch (PWMx->Channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(PWMx->TIMx, _OC);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(PWMx->TIMx, _OC);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(PWMx->TIMx, _OC);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(PWMx->TIMx, _OC);
        break;

    default:
        break;
    }

    LL_TIM_EnableUpdateEvent(PWMx->TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(PWMx->TIMx))
        {
            LL_TIM_DisableIT_UPDATE(PWMx->TIMx);
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
            LL_TIM_EnableIT_UPDATE(PWMx->TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
        }
    }
}

__STATIC_INLINE void HB_Set_Freq(PWM_TypeDef *PWMx, float _Freq, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Set PWM FREQ
    //PWMx->Freq = __LL_TIM_CALC_ARR(APB1_TIMER_CLK, LL_TIM_GetPrescaler(PWMx->TIMx), _Freq);
    PWMx->Freq = (float)APB1_TIMER_CLK / ((float)(LL_TIM_GetPrescaler(PWMx->TIMx) + 1) * _Freq);
    LL_TIM_SetAutoReload(PWMx->TIMx, PWMx->Freq);

    LL_TIM_EnableUpdateEvent(PWMx->TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(PWMx->TIMx))
        {
            LL_TIM_DisableIT_UPDATE(PWMx->TIMx);
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
            LL_TIM_EnableIT_UPDATE(PWMx->TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
        }
    }
}

__STATIC_INLINE void HB_Set_ARR(PWM_TypeDef *PWMx, uint32_t _ARR, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Set PWM FREQ
    PWMx->Freq = _ARR;
    LL_TIM_SetAutoReload(PWMx->TIMx, PWMx->Freq);

    LL_TIM_EnableUpdateEvent(PWMx->TIMx);

    if(apply_now == 1)
    {
        if (LL_TIM_IsEnabledIT_UPDATE(PWMx->TIMx))
        {
            LL_TIM_DisableIT_UPDATE(PWMx->TIMx);
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
            LL_TIM_EnableIT_UPDATE(PWMx->TIMx);
        }
        else
        {
            LL_TIM_GenerateEvent_UPDATE(PWMx->TIMx);
        }
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PWM_TypeDef HB_PWM_SD0_3 =
{
    .TIMx       =   H_BRIDGE_SD0_3_HANDLE,
    .Prescaler  =   1199,
    .Mode       =   LL_TIM_OCMODE_FORCED_INACTIVE,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   3, //100us
    .Freq       =   0,
};

PWM_TypeDef HB_PWM_SD4_7 =
{
    .TIMx       =   H_BRIDGE_SD4_7_HANDLE,
    .Prescaler  =   1199,
    .Mode       =   LL_TIM_OCMODE_FORCED_INACTIVE,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   3, //100us
    .Freq       =   0,
}; */

/* ::::H_Bridge SD0_3 Interupt Handle:::: */
/* void H_Bridge_SD0_3_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_SD0_3_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_SD0_3_HANDLE);

        switch (p_HB_SD_0_3_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_0_3_IRQn->pulse_count++;

            if (p_HB_SD_0_3_IRQn->pulse_count >= ((p_HB_SD_0_3_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(H_BRIDGE_SD0_3_HANDLE);
                return;
            }
            
            if (*p_HB_SD_0_3_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_0_3_IRQn->PWM, 1000.0 / (float)p_HB_SD_0_3_IRQn->off_time_ms, 0);
                *p_HB_SD_0_3_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_0_3_IRQn->PWM, 1000.0 / (float)p_HB_SD_0_3_IRQn->on_time_ms, 0);
                *p_HB_SD_0_3_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_0_3_IRQn->PWM.TIMx, p_HB_SD_0_3_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_0_3_IRQn->Port, *p_HB_SD_0_3_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_0_3_IRQn->PWM.TIMx, p_HB_SD_0_3_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_0_3_IRQn->PWM.TIMx);
            break;
        
        default:
            break;
        }

        p_HB_SD_0_3_IRQn->is_setted = true;
    }
} */

/* ::::H_Bridge SD4_7 Interupt Handle:::: */
/* void H_Bridge_SD4_7_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(H_BRIDGE_SD4_7_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(H_BRIDGE_SD4_7_HANDLE);

        switch (p_HB_SD_4_7_IRQn->Mode)
        {
        case H_BRIDGE_MODE_PULSE:
            p_HB_SD_4_7_IRQn->pulse_count++;

            if (p_HB_SD_4_7_IRQn->pulse_count >= ((p_HB_SD_4_7_IRQn->set_pulse_count * 2) + 1))
            {
                LL_TIM_DisableCounter(H_BRIDGE_SD4_7_HANDLE);
                return;
            }

            if (*p_HB_SD_4_7_IRQn->Pin_State == 1)
            {
                LL_GPIO_SetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_4_7_IRQn->PWM, 1000.0 / (float)p_HB_SD_4_7_IRQn->off_time_ms, 0);
                *p_HB_SD_4_7_IRQn->Pin_State = 0;
            }
            else
            {
                LL_GPIO_ResetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);
                HB_Set_Freq(&p_HB_SD_4_7_IRQn->PWM, 1000.0 / (float)p_HB_SD_4_7_IRQn->on_time_ms, 0);
                *p_HB_SD_4_7_IRQn->Pin_State = 1;
            }

            break;
        case H_BRIDGE_MODE_HS_ON:
            LL_GPIO_SetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_4_7_IRQn->PWM.TIMx, p_HB_SD_4_7_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_LS_ON:
            LL_GPIO_ResetOutputPin(p_HB_SD_4_7_IRQn->Port, *p_HB_SD_4_7_IRQn->Pin);

            LL_TIM_OC_SetMode(p_HB_SD_4_7_IRQn->PWM.TIMx, p_HB_SD_4_7_IRQn->PWM.Channel, LL_TIM_OCMODE_FORCED_ACTIVE);
            LL_TIM_GenerateEvent_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_ClearFlag_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        case H_BRIDGE_MODE_FLOAT:
            LL_TIM_DisableIT_UPDATE(p_HB_SD_4_7_IRQn->PWM.TIMx);
            break;
        
        default:
            break;
        }

        p_HB_SD_4_7_IRQn->is_setted = true;
    }
} */

// void H_Bridge_Set_Pulse_Timing(H_Bridge_typdef* H_Bridge_x, uint16_t Set_delay_time_ms, uint16_t Set_on_time_ms, uint16_t Set_off_time_ms, uint16_t Set_pulse_count)
// {
//     LL_TIM_DisableIT_UPDATE(H_Bridge_x->PWM.TIMx);
//     LL_TIM_DisableCounter(H_Bridge_x->PWM.TIMx);
//     LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

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
     
//     H_Bridge_x->PWM.Prescaler   = (((APB1_TIMER_CLK / 1000) * max_time_ms) / (UINT16_MAX)) + 1;
//     H_Bridge_x->PWM.Prescaler   = (H_Bridge_x->PWM.Prescaler > UINT16_MAX) ? UINT16_MAX : H_Bridge_x->PWM.Prescaler;

//     H_Bridge_x->PWM.Duty        = ((APB1_TIMER_CLK / 1000) * min_time_ms) / (H_Bridge_x->PWM.Prescaler * 100);
//     H_Bridge_x->PWM.Duty        = (H_Bridge_x->PWM.Duty > SD_DUTY_MIN) ? H_Bridge_x->PWM.Duty : SD_DUTY_MIN;

//     H_Bridge_x->delay_time_ms   = Set_delay_time_ms;

//     H_Bridge_x->on_time_ms      = Set_on_time_ms;
//     H_Bridge_x->off_time_ms     = Set_off_time_ms;

//     H_Bridge_x->set_pulse_count = Set_pulse_count;
//     H_Bridge_x->pulse_count     = 0;

//     LL_TIM_SetPrescaler(H_Bridge_x->PWM.TIMx, H_Bridge_x->PWM.Prescaler);
//     LL_TIM_GenerateEvent_UPDATE(H_Bridge_x->PWM.TIMx);
//     LL_TIM_ClearFlag_UPDATE(H_Bridge_x->PWM.TIMx);

//     LL_TIM_EnableIT_UPDATE(H_Bridge_x->PWM.TIMx);
//     LL_TIM_EnableCounter(H_Bridge_x->PWM.TIMx);
// }