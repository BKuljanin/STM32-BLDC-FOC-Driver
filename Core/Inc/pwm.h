#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"

typedef struct {
    float u_duty;   // Phase U duty cycle
    float v_duty;   // Phase V duty cycle
    float w_duty;   // Phase W duty cycle
} PhasesDuty_t;

extern TIM_HandleTypeDef htim1;
extern PhasesDuty_t phases_duty_cycles;

void MX_TIM1_Init(void);
void tim1_pwm_set_duty_percent(PhasesDuty_t phases_duty_cycle);

#define MAX_DUTY_CYCLE 80
#define TIM1_CH4_ARR (4500-1)

#endif /* INC_PWM_H_ */
