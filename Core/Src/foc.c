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
	compute_sin_cos(encoder.electrical_angle_rad, &sin_theta, &cos_theta);

	// Apply Park transform
	foc.id = foc.i_alpha * cos_theta + foc.i_beta * sin_theta;
	foc.iq = - foc.i_alpha * sin_theta + foc.i_beta * cos_theta;
}

void inverse_park_transform(void)
{
	// Compute sin and cos theta
	float sin_theta = 0.0f;
	float cos_theta = 0.0f;
	compute_sin_cos(encoder.electrical_angle_rad, &sin_theta, &cos_theta);

	// Apply inverse Park transform
	foc.v_alpha = foc.v_d * cos_theta - foc.v_q * sin_theta;
	foc.v_beta = foc.v_d * sin_theta + foc.v_q * cos_theta;
}

void svpwm_update(void)
{
	// Space vector pulse width modulation

	// Projecting from alpha, beta to u, v, w axes
	float v_u = foc.v_alpha;
	float v_v = -0.5f * foc.v_alpha + SQRT_3_HALF * foc.v_beta;
	float v_w = -0.5f * foc.v_alpha - SQRT_3_HALF * foc.v_beta;
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

	// Space vector PWM, from alpha, beta framework to duty cycles for phases U, V, W
	svpwm_update();

}
