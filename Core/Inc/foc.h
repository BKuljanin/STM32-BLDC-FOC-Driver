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
      float flux_alpha, flux_beta;  // observer flux state
      float theta_obs;              // estimated electrical angle [rad]
      float speed_ref;
      float position_ref;
      PhasesDuty_t duty;
  } FOC_t;

typedef enum {
    FOC_ENCODER,
    FOC_SENSORLESS
} FOC_Mode_t;

extern FOC_t foc;
extern FOC_Mode_t foc_mode;

// Phase resistance [Ohm] — measure on hardware, this is a starting estimate for A2208
#define MOTOR_R          0.1f
// Observer LP cutoff [rad/s] — sets minimum electrical speed for reliable angle estimate
#define OBSERVER_OMEGA_C 20.0f

void foc_update(void);
void bemf_observer_update(void);

#define SQRT_3 1.73205f
#define SQRT_3_HALF (SQRT_3/2)
#define VBUS_NOMINAL 12.0f

void svpwm_update(void);

#endif /* INC_FOC_H_ */
