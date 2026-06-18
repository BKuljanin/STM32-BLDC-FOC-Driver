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
      float speed_ref;
      float position_ref;
      PhasesDuty_t duty;
  } FOC_t;

extern FOC_t foc;

// Debug plotting copies, updated every 4th FOC cycle (5kHz)
extern volatile float plot_id;
extern volatile float plot_iq;
extern volatile float plot_vq;
extern volatile float plot_theta;   // electrical angle [rad]
extern volatile float plot_mech;    // mechanical angle [rad]

void foc_update(void);

#define SQRT_3 1.73205f
#define SQRT_3_INV (1/SQRT_3)
#define SQRT_3_HALF (SQRT_3/2)
#define VBUS_NOMINAL 12.0f
#define T_PWM 1.0f	// Normalized T pwm
#define T_PWM_HALF (T_PWM/2)

// Modulation method select
#define MODULATION_SVPWM 0
#define MODULATION_SPWM  1
#define MODULATION_TYPE  MODULATION_SVPWM

// Open loop V/f calibration test: bypass encoder + current loop and spin a
// fixed magnitude voltage vector at fixed electrical frequency

// Set to 0 for normal closed-loop FOC
#define OPEN_LOOP_VF_TEST 1
#define OL_VF_VOLTAGE     2.0f    // [V] stator vector magnitude (keep small)
#define OL_VF_FREQ_HZ     2.0f    // [Hz] electrical frequency (slow)

void svpwm_update(void);
void spwm_update(void);

#endif /* INC_FOC_H_ */
