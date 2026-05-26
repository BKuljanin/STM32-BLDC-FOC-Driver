#include "foc.h"

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

void foc_update(void)
{
	// Calculate i_u, i_v, i_w from raw values of i_u and i_v and calibration offsets
	calculate_currents();

	// Clarke transform
	clarke_transform();

	// Park transform
	park_transform();

}
