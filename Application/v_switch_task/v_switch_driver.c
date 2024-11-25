/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "app.h"
#include "stm32f4xx_ll_gpio.h"

#include "v_switch_driver.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
PWM_TypeDef V_Switch_HV_PWM =
{
    .TIMx       =   V_SWITCH_LIN1_HANDLE,
    .Channel    =   V_SWITCH_LIN1_CHANNEL,
    .Prescaler  =   300,
    .Mode       =   LL_TIM_OCMODE_FORCED_INACTIVE,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   0,
    .Freq       =   0,
};
PWM_TypeDef V_Switch_LV_PWM =
{
    .TIMx       =   V_SWITCH_LIN2_HANDLE,
    .Channel    =   V_SWITCH_LIN2_CHANNEL,
    .Prescaler  =   300,
    .Mode       =   LL_TIM_OCMODE_FORCED_INACTIVE,
    .Polarity   =   LL_TIM_OCPOLARITY_HIGH,
    .Duty       =   0,
    .Freq       =   0,
};

V_Switch_typdef V_Switch_HV =
{
    .Port               = V_SWITCH_HIN1_PORT,
    .Pin                = V_SWITCH_HIN1_PIN,
    .is_on              = 0,
    .PWM                = &V_Switch_HV_PWM,
    .is_on              = false,
};

V_Switch_typdef V_Switch_LV =
{
    .Port               = V_SWITCH_HIN2_PORT,
    .Pin                = V_SWITCH_HIN2_PIN,
    .is_on              = 0,
    .PWM                = &V_Switch_LV_PWM,
    .is_on              = false,
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void VS_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now);
__STATIC_INLINE void VS_Set_OC(PWM_TypeDef *PWMx, uint32_t _OC, bool apply_now);
__STATIC_INLINE void VS_Set_Freq(PWM_TypeDef *PWMx, uint32_t _Freq, bool apply_now);
__STATIC_INLINE void VS_Set_ARR(PWM_TypeDef *PWMx, uint32_t _ARR, bool apply_now);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: V_Switch Task Init :::::::: */
void V_Switch_Driver_Init(void)
{
    // V SWITCH HV INIT
    PWM_Init(V_Switch_HV.PWM);
    PWM_Enable(V_Switch_HV.PWM);
    LL_GPIO_ResetOutputPin(V_Switch_HV.Port, V_Switch_HV.Pin);
    LL_TIM_DisableIT_UPDATE(V_Switch_HV.PWM->TIMx);

    // V SWITCH LV INIT
    PWM_Init(V_Switch_LV.PWM);
    PWM_Enable(V_Switch_LV.PWM);
    LL_GPIO_ResetOutputPin(V_Switch_LV.Port, V_Switch_LV.Pin);
    LL_TIM_DisableIT_UPDATE(V_Switch_LV.PWM->TIMx);
}

void V_Switch_Set_Mode(V_Switch_mode SetMode)
{
    LL_TIM_DisableCounter(V_Switch_HV.PWM->TIMx);
    //LL_TIM_DisableCounter(V_Switch_LV.PWM->TIMx);

    switch (SetMode)
    {
    case V_SWITCH_MODE_HV_ON:
        if (V_Switch_HV.is_on == true)
            break;

        LL_TIM_OC_SetMode(V_Switch_LV.PWM->TIMx, V_Switch_LV.PWM->Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_GPIO_ResetOutputPin(V_Switch_LV.Port, V_Switch_LV.Pin);
        V_Switch_LV.is_on = false;

        LL_TIM_OC_SetMode(V_Switch_HV.PWM->TIMx, V_Switch_HV.PWM->Channel, LL_TIM_OCMODE_PWM2);
        VS_Set_Freq(V_Switch_HV.PWM, 5000, 0);
        VS_Set_Duty(V_Switch_HV.PWM, 50, 0);
        LL_GPIO_SetOutputPin(V_Switch_HV.Port, V_Switch_HV.Pin);
        V_Switch_HV.is_on = true;

        break;
    case V_SWITCH_MODE_LV_ON:
        if (V_Switch_LV.is_on == true)
            break;

        LL_TIM_OC_SetMode(V_Switch_HV.PWM->TIMx, V_Switch_HV.PWM->Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_GPIO_ResetOutputPin(V_Switch_HV.Port, V_Switch_HV.Pin);
        V_Switch_HV.is_on = false;

        LL_TIM_OC_SetMode(V_Switch_LV.PWM->TIMx, V_Switch_LV.PWM->Channel, LL_TIM_OCMODE_PWM2);
        VS_Set_Freq(V_Switch_LV.PWM, 5000, 0);
        VS_Set_Duty(V_Switch_LV.PWM, 50, 0);
        LL_GPIO_SetOutputPin(V_Switch_LV.Port, V_Switch_LV.Pin);
        V_Switch_LV.is_on = true;

        break;
    case V_SWITCH_MODE_ALL_OFF:
        LL_TIM_OC_SetMode(V_Switch_LV.PWM->TIMx, V_Switch_LV.PWM->Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_GPIO_ResetOutputPin(V_Switch_LV.Port, V_Switch_LV.Pin);
        V_Switch_LV.is_on = false;

        LL_TIM_OC_SetMode(V_Switch_HV.PWM->TIMx, V_Switch_HV.PWM->Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_GPIO_ResetOutputPin(V_Switch_HV.Port, V_Switch_HV.Pin);
        V_Switch_HV.is_on = false;

        break;
    case V_SWITCH_MODE_CB_ON:
        LL_TIM_OC_SetMode(V_Switch_LV.PWM->TIMx, V_Switch_LV.PWM->Channel, LL_TIM_OCMODE_FORCED_INACTIVE);
        LL_GPIO_ResetOutputPin(V_Switch_LV.Port, V_Switch_LV.Pin);
        V_Switch_LV.is_on = false;

        LL_TIM_OC_SetMode(V_Switch_HV.PWM->TIMx, V_Switch_HV.PWM->Channel, LL_TIM_OCMODE_PWM2);
        VS_Set_Freq(V_Switch_HV.PWM, 1000, 0);
        VS_Set_Duty(V_Switch_HV.PWM, 50, 0);

        break;
    case V_SWITCH_MODE_CB_OFF:
        LL_TIM_OC_SetMode(V_Switch_HV.PWM->TIMx, V_Switch_HV.PWM->Channel, LL_TIM_OCMODE_FORCED_INACTIVE);

        break;
    
    default:
        break;
    }

    LL_TIM_GenerateEvent_UPDATE(V_Switch_HV.PWM->TIMx);
    //LL_TIM_GenerateEvent_UPDATE(V_Switch_LV.PWM->TIMx);

    LL_TIM_EnableCounter(V_Switch_HV.PWM->TIMx);
    //LL_TIM_EnableCounter(V_Switch_LV.PWM->TIMx);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
__STATIC_INLINE void VS_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Limit the duty to 100
    if (_Duty > 100)
      return;

    // Set PWM DUTY for channel 1
    PWMx->Duty = (PWMx->Freq * (_Duty / 100.0));
    switch (PWMx->Channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(PWMx->TIMx, PWMx->Duty);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(PWMx->TIMx, PWMx->Duty);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(PWMx->TIMx, PWMx->Duty);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(PWMx->TIMx, PWMx->Duty);
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

__STATIC_INLINE void VS_Set_OC(PWM_TypeDef *PWMx, uint32_t _OC, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Set PWM DUTY for channel 1
    PWMx->Duty = _OC;
    switch (PWMx->Channel)
    {
    case LL_TIM_CHANNEL_CH1:
        LL_TIM_OC_SetCompareCH1(PWMx->TIMx, PWMx->Duty);
        break;
    case LL_TIM_CHANNEL_CH2:
        LL_TIM_OC_SetCompareCH2(PWMx->TIMx, PWMx->Duty);
        break;
    case LL_TIM_CHANNEL_CH3:
        LL_TIM_OC_SetCompareCH3(PWMx->TIMx, PWMx->Duty);
        break;
    case LL_TIM_CHANNEL_CH4:
        LL_TIM_OC_SetCompareCH4(PWMx->TIMx, PWMx->Duty);
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

__STATIC_INLINE void VS_Set_Freq(PWM_TypeDef *PWMx, uint32_t _Freq, bool apply_now)
{
    LL_TIM_DisableUpdateEvent(PWMx->TIMx);

    // Set PWM FREQ
    PWMx->Freq = __LL_TIM_CALC_ARR(APB1_TIMER_CLK, LL_TIM_GetPrescaler(PWMx->TIMx), _Freq);
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

__STATIC_INLINE void VS_Set_ARR(PWM_TypeDef *PWMx, uint32_t _ARR, bool apply_now)
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
