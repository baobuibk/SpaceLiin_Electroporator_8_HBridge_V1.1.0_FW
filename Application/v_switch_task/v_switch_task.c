/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdbool.h>

#include "stm32f4xx_ll_gpio.h"

#include "app.h"

#include "pwm.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
PWM_TypeDef V_Switch_1_PWM =
{
    .TIMx       =   V_SWITCH_LIN1_HANDLE,
    .Channel    =   V_SWITCH_LIN1_CHANNEL,
    .Prescaler  =   100,
    .Mode       =   LL_TIM_OCMODE_PWM1,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   0,
    .Freq       =   0,
};
PWM_TypeDef V_Switch_2_PWM =
{
    .TIMx       =   V_SWITCH_LIN2_HANDLE,
    .Channel    =   V_SWITCH_LIN2_CHANNEL,
    .Prescaler  =   100,
    .Mode       =   LL_TIM_OCMODE_PWM1,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   0,
    .Freq       =   0,
};

static V_Switch_state_typdef V_Switch_State = V_SWITCH_STOP_STATE;
static bool is_v_switch_set_up = false;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static inline void V_Switch_Set_Freq(PWM_TypeDef *PWMx, uint32_t _Freq);
static inline void V_Switch_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty);
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
V_Switch_state_typdef Channel_Set = V_SWITCH_STOP_STATE;
bool is_v_switch_enable = false;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: V_Switch Task Init :::::::: */
void V_Switch_Task_Init(void)
{
    // V SWITCH 1 INIT
    LL_GPIO_ResetOutputPin(V_SWITCH_HIN1_PORT, V_SWITCH_HIN1_PIN);
    PWM_Init(&V_Switch_1_PWM);
    PWM_Set_Freq(&V_Switch_1_PWM, 10000, 1);
    PWM_Set_Duty(&V_Switch_1_PWM, 50, 1);
    LL_TIM_DisableIT_UPDATE(V_SWITCH_LIN1_HANDLE);
    PWM_Enable(&V_Switch_1_PWM);

    // V SWITCH 2 INIT
    LL_GPIO_ResetOutputPin(V_SWITCH_HIN2_PORT, V_SWITCH_HIN2_PIN);
    PWM_Init(&V_Switch_2_PWM);
    PWM_Set_Freq(&V_Switch_2_PWM, 10000, 1);
    PWM_Set_Duty(&V_Switch_2_PWM, 50, 1);
    LL_TIM_DisableIT_UPDATE(V_SWITCH_LIN2_HANDLE);
    PWM_Enable(&V_Switch_2_PWM);
}

/* :::::::::: V_Switch Task ::::::::::::: */
void V_Switch_Task(void*)
{
    switch (V_Switch_State)
    {
    case V_SWITCH_STOP_STATE:
        if (is_v_switch_set_up == false)
        {
            LL_GPIO_ResetOutputPin(V_SWITCH_HIN1_PORT, V_SWITCH_HIN1_PIN);
            LL_TIM_OC_SetMode(V_SWITCH_LIN1_HANDLE, V_SWITCH_LIN1_CHANNEL, LL_TIM_OCMODE_PWM1);

            LL_GPIO_ResetOutputPin(V_SWITCH_HIN2_PORT, V_SWITCH_HIN2_PIN);
            LL_TIM_OC_SetMode(V_SWITCH_LIN2_HANDLE, V_SWITCH_LIN2_CHANNEL, LL_TIM_OCMODE_PWM1);

            is_v_switch_set_up = true;
        }

        if (is_v_switch_enable == true)
        {
            V_Switch_State = Channel_Set;
            is_v_switch_set_up = false;
        }
        break;
    case V_SWITCH_C1_STATE:
        if (is_v_switch_enable == false)
        {
            V_Switch_State = V_SWITCH_STOP_STATE;
            is_v_switch_set_up = false;
            break;
        }
        
        if (Channel_Set != V_Switch_State)
        {
            V_Switch_State = Channel_Set;
            is_v_switch_set_up = false;
        }
        else if(is_v_switch_set_up == false)
        {
            LL_GPIO_ResetOutputPin(V_SWITCH_HIN2_PORT, V_SWITCH_HIN2_PIN);
            LL_TIM_OC_SetMode(V_SWITCH_LIN2_HANDLE, V_SWITCH_LIN2_CHANNEL, LL_TIM_OCMODE_PWM1);

            // STOP THE CNT AND RESET IT TO 0.
            LL_TIM_DisableCounter(V_SWITCH_LIN1_HANDLE);
            PWM_Set_Freq(&V_Switch_1_PWM, 10000, 1);
            PWM_Set_Duty(&V_Switch_1_PWM, 0, 1);
            LL_TIM_OC_SetMode(V_SWITCH_LIN1_HANDLE, V_SWITCH_LIN1_CHANNEL, LL_TIM_OCMODE_PWM1);
            V_Switch_Set_Freq(&V_Switch_1_PWM, 10000);
            V_Switch_Set_Duty(&V_Switch_1_PWM, 50);

            //ENABLE IT UPDATE, ENABLE CNT AND GENERATE EVENT
            LL_TIM_ClearFlag_UPDATE(V_SWITCH_LIN1_HANDLE);
            LL_TIM_EnableIT_UPDATE(V_SWITCH_LIN1_HANDLE);
            LL_TIM_EnableCounter(V_SWITCH_LIN1_HANDLE);
        }

        break;
    case V_SWITCH_C2_STATE:
        if (is_v_switch_enable == false)
        {
            V_Switch_State = V_SWITCH_STOP_STATE;
            is_v_switch_set_up = false;
            break;
        }

        if (Channel_Set != V_Switch_State)
        {
            V_Switch_State = Channel_Set;
            is_v_switch_set_up = false;
        }
        else if(is_v_switch_set_up == false)
        {
            LL_GPIO_ResetOutputPin(V_SWITCH_HIN1_PORT, V_SWITCH_HIN1_PIN);
            LL_TIM_OC_SetMode(V_SWITCH_LIN1_HANDLE, V_SWITCH_LIN1_CHANNEL, LL_TIM_OCMODE_PWM1);

            // STOP THE CNT AND RESET IT TO 0.
            LL_TIM_DisableCounter(V_SWITCH_LIN2_HANDLE);
            PWM_Set_Freq(&V_Switch_2_PWM, 10000, 1);
            PWM_Set_Duty(&V_Switch_2_PWM, 0, 1);
            LL_TIM_OC_SetMode(V_SWITCH_LIN2_HANDLE, V_SWITCH_LIN2_CHANNEL, LL_TIM_OCMODE_PWM1);
            V_Switch_Set_Freq(&V_Switch_2_PWM, 10000);
            V_Switch_Set_Duty(&V_Switch_2_PWM, 50);

            //ENABLE IT UPDATE, ENABLE CNT AND GENERATE EVENT
            LL_TIM_ClearFlag_UPDATE(V_SWITCH_LIN2_HANDLE);
            LL_TIM_EnableIT_UPDATE(V_SWITCH_LIN2_HANDLE);
            LL_TIM_EnableCounter(V_SWITCH_LIN2_HANDLE);            
        }

        break;

    default:
        break;
    }
}

/* ::::V_Switch 1 Interupt Handle:::: */
void V_Switch_1_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(V_SWITCH_LIN1_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(V_SWITCH_LIN1_HANDLE);
        LL_GPIO_SetOutputPin(V_SWITCH_HIN1_PORT, V_SWITCH_HIN1_PIN);
        LL_TIM_DisableIT_UPDATE(V_SWITCH_LIN1_HANDLE);
        //is_v_switch_set_up = true;
    }
}

/* ::::V_Switch 2 Interupt Handle:::: */
void V_Switch_2_Interupt_Handle()
{
    if(LL_TIM_IsActiveFlag_UPDATE(V_SWITCH_LIN2_HANDLE) == true)
    {
        LL_TIM_ClearFlag_UPDATE(V_SWITCH_LIN2_HANDLE);
        LL_GPIO_SetOutputPin(V_SWITCH_HIN2_PORT, V_SWITCH_HIN2_PIN);
        LL_TIM_DisableIT_UPDATE(V_SWITCH_LIN2_HANDLE);
        //is_v_switch_set_up = true;
    }
}

static inline void V_Switch_Set_Freq(PWM_TypeDef *PWMx, uint32_t _Freq)
{
    uint16_t SD_ARR;
    SD_ARR = __LL_TIM_CALC_ARR(APB1_TIMER_CLK, LL_TIM_GetPrescaler(PWMx->TIMx), _Freq);
    LL_TIM_SetAutoReload(PWMx->TIMx, SD_ARR);
}

static inline void V_Switch_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty)
{
    // Limit the duty to 100
    if (_Duty > 100)
      return;

    // Set PWM DUTY for channel 1
    PWMx->Duty = (PWMx->Freq * (_Duty / 100.0));

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
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
