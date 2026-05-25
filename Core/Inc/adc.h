#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);

typedef struct {
    float i_a;   // Phase A current
    float i_b;   // Phase B current
    float i_c;   // Phase C current
} PhaseCurrents_t;

#endif /* INC_ADC_H_ */
