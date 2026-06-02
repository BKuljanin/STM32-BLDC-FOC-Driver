#include "foc.h"
#include "controller.h"

FOC_t foc;

void clarke_transform(void)
{
	// Projecting U,V,W to alpha,beta, applying Clarke transform
	foc.i_alpha = phase_currents.i_u - 0.5f * (phase_currents.i_v + phase_currents.i_w);
	foc.i_beta = SQRT_3_HALF * (phase_currents.i_v - phase_currents.i_w);

	// Rescaling values
	/* Clarke transform overscales values by 1.5, now magnitude is scaled back */
	foc.i_alpha = foc.i_alpha / 1.5f;
	foc.i_beta = foc.i_beta / 1.5f;
}

void park_transform(void)
{
	// Compute sin and cos theta

	float sin_theta = 0.0f;
	float cos_theta = 0.0f;
	/*	Custom lookup table for sin and cos to save time on doing trigonometry
	compute_sin_cos(encoder.electrical_angle_rad, &sin_theta, &cos_theta);
	*/
	sin_theta = sinf(encoder.electrical_angle_rad);
	cos_theta = cosf(encoder.electrical_angle_rad);

	// Apply Park transform
	foc.id = foc.i_alpha * cos_theta + foc.i_beta * sin_theta;
	foc.iq = - foc.i_alpha * sin_theta + foc.i_beta * cos_theta;
}

void inverse_park_transform(void)
{
	// Compute sin and cos theta
	float sin_theta = 0.0f;
	float cos_theta = 0.0f;
	// compute_sin_cos(encoder.electrical_angle_rad, &sin_theta, &cos_theta);
	sin_theta = sinf(encoder.electrical_angle_rad);
	cos_theta = cosf(encoder.electrical_angle_rad);

	// Apply inverse Park transform
	foc.v_alpha = foc.vd * cos_theta - foc.vq * sin_theta;
	foc.v_beta = foc.vd * sin_theta + foc.vq * cos_theta;
}

uint8_t svpwm_get_sector(float v_alpha, float v_beta)
  {
        float abs_alpha = fabsf(v_alpha);

        if (v_beta >= 0.0f) {
                // Upper half: u_beta >= 0
                if (v_beta > SQRT_3 * abs_alpha) {	// u_alpha = u_beta / sqrt(3) is limit between sector 1 and 2
                        return 2;
                } else {
                        return (v_alpha >= 0.0f) ? 1 : 3;
                }
        } else {
                // Lower half: u_beta < 0
                if (-v_beta > SQRT_3 * abs_alpha) {
                        return 5;
                } else {
                        return (v_alpha >= 0.0f) ? 6 : 4;
                }
        }
  }

void svpwm_calc_times(uint8_t sector, float v_alpha, float v_beta, float *t_k, float *t_k1)
{
      const float g = SQRT_3_HALF * T_PWM / VBUS_NOMINAL;
      float k_angle   = sector       * (PI / 3.0f);   //  k    * pi/3
      float k1_angle = (sector - 1) * (PI / 3.0f);   // (k-1) * pi/3

      *t_k = g * (  sinf(k_angle)   * v_alpha - cosf(k_angle)   * v_beta );
      *t_k1 = g * ( -sinf(k1_angle) * v_alpha + cosf(k1_angle) * v_beta );
}

void svpwm_update(void)
{
	// Space vector pulse width modulation

	// Determine sector
	uint8_t sector = svpwm_get_sector(foc.v_alpha, foc.v_beta);

	/*
	 * Active vectors:
		V0 = 000
		V1 = 001
		V2 = 011
		V3 = 010
		V4 = 100
		V5 = 101
		V6 = 110
		V7 = 111

		tk time spent in first active vector
		tk+1 time spent in second active vector
	 * */

	float t_k;
	float t_k1;

	svpwm_calc_times(sector, foc.v_alpha, foc.v_beta, &t_k, &t_k1);

	// Calculate short, middle, long times
	float t_0 = T_PWM_HALF - t_k - t_k1;
	  if (t_0 < 0.0f) {
	      float sum = t_k + t_k1;
	      t_k  = T_PWM_HALF * t_k  / sum;
	      t_k1 = T_PWM_HALF * t_k1 / sum;
	      t_0  = 0.0f;
	  }
	float t_short = t_0 / 2.0f;
	// middle phase is high in only one corner. odd sectors -> 2nd corner (t_k1), even -> 1st corner (t_k).
	// book's formula only works for odd sectors.
	float t_middle = (sector & 1) ? (t_0/2.0f + t_k1) : (t_0/2.0f + t_k);
	float t_long =  t_0 / 2.0f + t_k1 + t_k;

	float t_u;
	float t_v;
	float t_w;

	switch (sector) {
		case 1:
			t_u = t_long;
			t_v = t_middle;
			t_w = t_short;
			break;
		case 2:
			t_u = t_middle;
			t_v = t_long;
			t_w = t_short;
			break;
		case 3:
			t_u = t_short;
			t_v = t_long;
			t_w = t_middle;
			break;
		case 4:
			t_u = t_short;
			t_v = t_middle;
			t_w = t_long;
			break;
		case 5:
			t_u = t_middle;
			t_v = t_short;
			t_w = t_long;
			break;
		case 6:
			t_u = t_long;
			t_v = t_short;
			t_w = t_middle;
			break;
		default:
			t_u = 0.5f;
			t_v = 0.5f;
			t_w = 0.5f;
			break;
	}

	foc.duty.u_duty = t_u * 2.0f * 100.0f;	// *2 because half time is computed
	foc.duty.v_duty = t_v * 2.0f * 100.0f;
	foc.duty.w_duty = t_w * 2.0f * 100.0f;

	tim1_pwm_set_duty_percent(foc.duty);
}

void spwm_update(void)
{
	// Sinusoidal PWM. Inverse Clarke: alpha,beta -> phase voltages, then center each at 50% duty.
	float v_u = foc.v_alpha;
	float v_v = -0.5f * foc.v_alpha + SQRT_3_HALF * foc.v_beta;
	float v_w = -0.5f * foc.v_alpha - SQRT_3_HALF * foc.v_beta;

	// duty = 50% (midpoint) + phase voltage as fraction of Vbus. Max |v| = Vbus/2 -> 0..100%.
	foc.duty.u_duty = 50.0f + (v_u / VBUS_NOMINAL) * 100.0f;
	foc.duty.v_duty = 50.0f + (v_v / VBUS_NOMINAL) * 100.0f;
	foc.duty.w_duty = 50.0f + (v_w / VBUS_NOMINAL) * 100.0f;

	tim1_pwm_set_duty_percent(foc.duty);
}

void foc_update(void)
{
	// Calculate i_u, i_v, i_w from raw values of i_u and i_v and calibration offsets
	calculate_currents();

	// Clarke transform, 3 phases currents to alpha, beta framework
	clarke_transform();

	// Park transform, alpha beta to d,q framework
	park_transform();

	// Call position PI, speed PI, current PI (iq and id)
	controller_task();

	// Inverse Park transform, v_d and v_q to alpha, beta framework
	inverse_park_transform();

	// Modulation: alpha, beta -> duty cycles for phases U, V, W
#if MODULATION_TYPE == MODULATION_SVPWM
	svpwm_update();
#else
	spwm_update();
#endif

}
