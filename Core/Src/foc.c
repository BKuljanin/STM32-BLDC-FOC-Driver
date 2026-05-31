#include "foc.h"
#include "controller.h"

FOC_t foc;
FOC_Mode_t foc_mode = FOC_SENSORLESS;

static float get_theta(void)
{
    return (foc_mode == FOC_SENSORLESS) ? foc.theta_obs : encoder.electrical_angle_rad;
}

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
	compute_sin_cos(get_theta(), &sin_theta, &cos_theta);

	// Apply Park transform
	foc.id = foc.i_alpha * cos_theta + foc.i_beta * sin_theta;
	foc.iq = - foc.i_alpha * sin_theta + foc.i_beta * cos_theta;
}

void inverse_park_transform(void)
{
	// Compute sin and cos theta
	float sin_theta = 0.0f;
	float cos_theta = 0.0f;
	compute_sin_cos(get_theta(), &sin_theta, &cos_theta);

	// Apply inverse Park transform
	foc.v_alpha = foc.vd * cos_theta - foc.vq * sin_theta;
	foc.v_beta = foc.vd * sin_theta + foc.vq * cos_theta;
}

void svpwm_update(void)
{
	// Space vector pulse width modulation

	// Projecting from alpha, beta to u, v, w axes
	float v_u = foc.v_alpha;
	float v_v = -0.5f * foc.v_alpha + SQRT_3_HALF * foc.v_beta;
	float v_w = -0.5f * foc.v_alpha - SQRT_3_HALF * foc.v_beta;

	// Determine Sector using a 3 bit digital code based on positive projections
	uint8_t sector_code = 0;
	if (v_u > 0.0f) sector_code |= 1; // Bit 0
	if (v_v > 0.0f) sector_code |= 2; // Bit 1
	if (v_w > 0.0f) sector_code |= 4; // Bit 2

	uint8_t sector = 0;
	float t_1 = 0.0f, t_2 = 0.0f, t_0 = 0.0f;

	/*
	 * Active vectors:
		V0 = 000
		V1 = 100
		V2 = 110
		V3 = 010
		V4 = 011
		V5 = 001
		V6 = 101
		V7 = 111

		t1 time spent in first active vector
		t2 time spent in second active vector
	 * */

	switch(sector_code) {
	        case 3: // 011 (W low, V high, U high)
	            sector = 1;
	            t_1 = SQRT_3 * v_v / VBUS_NOMINAL;
	            t_2 = SQRT_3 * v_u / VBUS_NOMINAL;
	            break;
	        case 1: // 001
	            sector = 2;
	            t_1 = SQRT_3 * v_u / VBUS_NOMINAL;
	            t_2 = -SQRT_3 * v_w / VBUS_NOMINAL;
	            break;
	        case 5: // 101
	            sector = 3;
	            t_1 = -SQRT_3 * v_w / VBUS_NOMINAL;
	            t_2 = SQRT_3 * v_v / VBUS_NOMINAL;
	            break;
	        case 4: // 100
	            sector = 4;
	            t_1 = SQRT_3 * v_w / VBUS_NOMINAL;
	            t_2 = -SQRT_3 * v_u / VBUS_NOMINAL;
	            break;
	        case 6: // 110
	            sector = 5;
	            t_1 = -SQRT_3 * v_u / VBUS_NOMINAL;
	            t_2 = -SQRT_3 * v_v / VBUS_NOMINAL;
	            break;
	        case 2: // 010
	            sector = 6;
	            t_1 = -SQRT_3 * v_v / VBUS_NOMINAL;
	            t_2 = -SQRT_3 * v_w / VBUS_NOMINAL;
	            break;
	        default:
	            t_1 = 0.0f;
	            t_2 = 0.0f;
	            break;
	    }

	// Clamp t_1 and t_2
	if (t_1 + t_2 > 1.0f) {
		float scale = 1.0f / (t_1 + t_2);
		t_1 *= scale;
		t_2 *= scale;
	}

	t_0 = 1.0f - t_1 - t_2;
	float t_z = t_0 * 0.5f;	// Symmetric PWM splits zero time

	float d_u, d_v, d_w;

	// Each phase duty is built from t_z (null time) + its active vector time. The phase that
	// leads the sector gets 1-t_z (on for almost the whole period), the rest get t_z or t_z+t_n.
	switch (sector) {
		case 1: d_u = 1.0f - t_z;  d_v = t_z + t_2;   d_w = t_z;         break;
		case 2: d_u = t_z + t_1;   d_v = 1.0f - t_z;  d_w = t_z;         break;
		case 3: d_u = t_z;         d_v = 1.0f - t_z;  d_w = t_z + t_2;   break;
		case 4: d_u = t_z;         d_v = t_z + t_1;   d_w = 1.0f - t_z;  break;
		case 5: d_u = t_z + t_2;   d_v = t_z;         d_w = 1.0f - t_z;  break;
		case 6: d_u = 1.0f - t_z;  d_v = t_z;         d_w = t_z + t_1;   break;
		default: d_u = 0.5f; d_v = 0.5f; d_w = 0.5f;  break;
	}

	foc.duty.u_duty = d_u * 100.0f;
	foc.duty.v_duty = d_v * 100.0f;
	foc.duty.w_duty = d_w * 100.0f;

	tim1_pwm_set_duty_percent(foc.duty);
}

void bemf_observer_update(void)
{
    // LPF-based flux observer. The OBSERVER_OMEGA_C term corrects integrator drift —
    // pure integration would accumulate DC error from R mismatch and ADC offset.
    foc.flux_alpha += (foc.v_alpha - MOTOR_R * foc.i_alpha - OBSERVER_OMEGA_C * foc.flux_alpha) * CURRENT_LOOP_DT;
    foc.flux_beta  += (foc.v_beta  - MOTOR_R * foc.i_beta  - OBSERVER_OMEGA_C * foc.flux_beta)  * CURRENT_LOOP_DT;

    float theta = atan2f(-foc.flux_alpha, foc.flux_beta);
    if (theta < 0.0f) theta += 2.0f * PI;
    foc.theta_obs = theta;
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

	// Update flux observer — needs v_alpha/beta (after inv-park) and i_alpha/beta (after clarke)
	// Result used by park/inv-park on the next cycle
	if (foc_mode == FOC_SENSORLESS)
		bemf_observer_update();
}
