#include "pwm.h"

/**
  * @brief  Define the behavior of the PWM.
  * @note   The Pin is set to high so that the FET is turned off.
  * @param  PWMx    PWM instance
  * @param  TIMx    TIM instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  * @retval None
  */
void PWM_Init(PWM_TypeDef *PWMx)
{
    // Set PWM MODE
    LL_TIM_OC_SetMode(PWMx->TIMx, PWMx->Channel, PWMx->Mode);
    LL_TIM_OC_SetPolarity(PWMx->TIMx, PWMx->Channel, PWMx->Polarity);
    LL_TIM_SetPrescaler(PWMx->TIMx, PWMx->Prescaler);

    // Set PWM FREQ
    if (PWMx->Freq > 0)
    {
        PWM_Set_Freq(PWMx, PWMx->Freq, 1);
    }
    
    // Set PWM DUTY
    if (PWMx->Duty > 0)
    {
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
    }

    // Enable the PWM FREQ and PWM DUTY
    LL_TIM_EnableARRPreload(PWMx->TIMx);
    LL_TIM_OC_EnablePreload(PWMx->TIMx, PWMx->Channel);

    LL_TIM_SetUpdateSource(PWMx->TIMx, LL_TIM_UPDATESOURCE_REGULAR);

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

/**
  * @brief  Enable the PWM.
  * @param  PWMx->TIMx Timer instance
  * @param  PWMx->Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval None
  */
void PWM_Enable(PWM_TypeDef *PWMx)
{
	/*
    if((PWMx->TIMx == TIM14) || (PWMx->TIMx == TIM15) || (PWMx->TIMx == TIM16) || (PWMx->TIMx == TIM17))
    {
        // Sellect off state when run or disable
        LL_TIM_SetOffStates(PWMx->TIMx, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);

        // Enable the PWM to output
        LL_TIM_CC_EnableChannel(PWMx->TIMx, PWMx->Channel);

        // Disable Automatic output enable
        // so that MOE (Main output enable) can be
        // set only by software
        LL_TIM_DisableAutomaticOutput(PWMx->TIMx);

        // Enable MOE (Main output enable)
        LL_TIM_EnableAllOutputs(PWMx->TIMx);
    }
    */

	 // Enable the PWM to output
	LL_TIM_CC_EnableChannel(PWMx->TIMx, PWMx->Channel);
	LL_TIM_EnableCounter(PWMx->TIMx);
}

/**
  * @brief  Disable the PWM.
  * @param  PWMx->TIMx Timer instance
  * @param  PWMx->Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval None
  */
void PWM_Disable(PWM_TypeDef *PWMx)
{
	/*
    if((PWMx->TIMx == TIM14) || (PWMx->TIMx == TIM15) || (PWMx->TIMx == TIM16) || (PWMx->TIMx == TIM17))
    {
        // Enable MOE (Main output enable)
        LL_TIM_DisableAllOutputs(PWMx->TIMx);
    }
    */
	// Disable the PWM to output
	LL_TIM_DisableCounter(PWMx->TIMx);
	LL_TIM_CC_DisableChannel(PWMx->TIMx, PWMx->Channel);
}

void PWM_Set_Duty(PWM_TypeDef *PWMx, uint32_t _Duty, bool apply_now)
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

void PWM_Set_OC(PWM_TypeDef *PWMx, uint32_t _OC, bool apply_now)
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

void PWM_Set_Freq(PWM_TypeDef *PWMx, uint32_t _Freq, bool apply_now)
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

void PWM_Set_ARR(PWM_TypeDef *PWMx, uint32_t _ARR, bool apply_now)
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
