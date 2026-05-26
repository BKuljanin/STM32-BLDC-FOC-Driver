#include "pwm.h"

TIM_HandleTypeDef htim1;

PhasesDuty_t phases_duty_cycles;

void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

  // CCR of channel 4 set to ARR to get channel 4 event on every ARR counted
  TIM1->CCR4 = TIM1_CH4_ARR;

  // Initializing channels 1-3
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
}

void tim1_pwm_set_duty_percent(PhasesDuty_t phases_duty_cycle)
{
	// Input is duty cycle in %

    if (phases_duty_cycle.u_duty > MAX_DUTY_CYCLE)
    	phases_duty_cycle.u_duty = MAX_DUTY_CYCLE;
    if (phases_duty_cycle.u_duty < 0.0f)
    	phases_duty_cycle.u_duty = 0.0f;

    if (phases_duty_cycle.v_duty > MAX_DUTY_CYCLE)
    	phases_duty_cycle.v_duty = MAX_DUTY_CYCLE;
    if (phases_duty_cycle.v_duty < 0.0f)
    	phases_duty_cycle.v_duty = 0.0f;

    if (phases_duty_cycle.w_duty > MAX_DUTY_CYCLE)
    	phases_duty_cycle.w_duty = MAX_DUTY_CYCLE;
    if (phases_duty_cycle.w_duty < 0.0f)
    	phases_duty_cycle.w_duty = 0.0f;

    // Setting up count value as a percentage of ARR
    uint32_t count_value_u = (uint32_t)(phases_duty_cycle.u_duty * TIM1->ARR) / 100; // Mode 1. First multiplication then division because of integer division
    uint32_t count_value_v = (uint32_t)(phases_duty_cycle.v_duty * TIM1->ARR) / 100;
    uint32_t count_value_w = (uint32_t)(phases_duty_cycle.w_duty * TIM1->ARR) / 100;

    TIM1->CCR1 = count_value_u;
    TIM1->CCR2 = count_value_v;
    TIM1->CCR3 = count_value_w;

}
