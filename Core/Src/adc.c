#include "adc.h"

volatile uint16_t adc_iu_raw;
volatile uint16_t adc_iv_raw;

volatile PhaseCurrents_t phase_currents;
volatile PhaseCurrentsOffsets_t phase_currents_offsets;

volatile uint8_t current_init_done;
volatile uint16_t current_calibration_count;

volatile uint8_t bldc_init_done;

ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}

void calculate_currents(void)
{
	float v_u = ADC_RATIO * (adc_iu_raw - phase_currents_offsets.i_u_offset);
	float v_v = ADC_RATIO * (adc_iv_raw - phase_currents_offsets.i_v_offset);

	phase_currents.i_u = v_u / VOLTAGE_TO_CURRENT_RATIO;
	phase_currents.i_v = v_v / VOLTAGE_TO_CURRENT_RATIO;
	phase_currents.i_w = - phase_currents.i_u - phase_currents.i_v;
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
	// Read raw adc values from data registers of injected ADCs
    adc_iu_raw = ADC1->JDR1;
    adc_iv_raw = ADC1->JDR2;

    // Toggle to observe timing on logic analyzer
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

    /* Setting offset for phases U and V */
    if (current_init_done == 0)
    {
    	phase_currents_offsets.i_u_offset += adc_iu_raw;
		phase_currents_offsets.i_v_offset += adc_iv_raw;

    	current_calibration_count++;

    	if(current_calibration_count == CURRENT_OFFSET_SAMPLES)
    	{
    		phase_currents_offsets.i_u_offset = phase_currents_offsets.i_u_offset / CURRENT_OFFSET_SAMPLES;
    		phase_currents_offsets.i_v_offset = phase_currents_offsets.i_v_offset / CURRENT_OFFSET_SAMPLES;

    		current_init_done = 1;
    	}
    }

    // Initialize BLDC into a know position and set encoder to 0
    else if (bldc_init_done == 0)
    {
    	bldc_init();
    	bldc_init_done = 1;
    }

    /* Normal operation after system has been initialized
     * In this interrupt main FOC logic is implemented
     * The code has ~25 us to run, from PWM low (ARR) to next PWM cycle (count = 0) */
    else
    {
    	foc_update();
    }

  }
}
