#ifndef INC_FOC_H_
#define INC_FOC_H_

#include "adc.h"
#include "as5600.h"
#include "pwm.h"
#include "math.h"
#include "trigonometry.h"

typedef struct {
      float i_alpha, i_beta;
      float id, iq;
      float id_ref, iq_ref;
      float vd, vq;
      float v_alpha, v_beta;
      float theta_e;
      float speed_ref;
      float position_ref;
      float id_integral, iq_integral;
      PhasesDuty_t duty;
  } FOC_t;

extern FOC_t foc;

void foc_update(void);

#define PI 3.141592f
#define SQRT_3 1.73205f
#define SQRT_3_HALF (SQRT_3/2)

#endif /* INC_FOC_H_ */
