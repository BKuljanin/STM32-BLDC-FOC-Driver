#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

typedef struct {
    float i_u;   // Phase U current
    float i_v;   // Phase V current
    float i_w;   // Phase W current
} PhaseCurrents_t;

typedef struct {
    float i_u_offset;   	// Phase U current
    float i_v_offset;   	// Phase V current
} PhaseCurrentsOffsets_t;

extern ADC_HandleTypeDef hadc1;
extern volatile PhaseCurrents_t phase_currents;
extern volatile PhaseCurrentsOffsets_t phase_currents_offsets;

void MX_ADC1_Init(void);

#define CURRENT_OFFSET_SAMPLES 100
#define SHUNT_RESISTOR_VALUE 0.33f // [Ohm]
#define AMPLIFICATION_VALUE 1.53f
#define ADC_VOLTAGE_LEVEL 3.3f // [V]
#define ADC_RESOLUTION 4096
#define ADC_RESOLUTION_HALF 2048
#define ADC_RATIO (ADC_VOLTAGE_LEVEL/ADC_RESOLUTION)
#define VOLTAGE_TO_CURRENT_RATIO (SHUNT_RESISTOR_VALUE * AMPLIFICATION_VALUE)

#endif /* INC_ADC_H_ */
