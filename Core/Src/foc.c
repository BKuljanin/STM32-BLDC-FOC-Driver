#include "foc.h"

FOC_t foc;

void foc_update(void)
{
	// Calculate i_u, i_v, i_w from raw values of i_u and i_v and calibration offsets
	calculate_currents();

	// Clarke transform
	clarke_transform();

}

void clarke_transform(void)
{
	// Projecting U,V,W to alpha,beta
	foc.i_alpha = phase_currents.i_u - 0.5f * (phase_currents.i_v + phase_currents.i_w);
	foc.i_beta = SQRT_3_HALF * (phase_currents.i_v - phase_currents.i_w);

	// Rescaling values
	foc.i_alpha = foc.i_alpha / 1.5f;
	foc.i_beta = foc.i_beta / 1.5f;

}
