#include "controller.h"

PI_Controller_t id_pi       = { ID_PI_KP,       ID_PI_KI,       ID_PI_INTEGRAL_SAT,       ID_PI_OUT_UPPER,       ID_PI_OUT_LOWER,       0.0f };
PI_Controller_t iq_pi       = { IQ_PI_KP,       IQ_PI_KI,       IQ_PI_INTEGRAL_SAT,       IQ_PI_OUT_UPPER,       IQ_PI_OUT_LOWER,       0.0f };
PI_Controller_t speed_pi    = { SPEED_PI_KP,    SPEED_PI_KI,    SPEED_PI_INTEGRAL_SAT,    SPEED_PI_OUT_UPPER,    SPEED_PI_OUT_LOWER,    0.0f };
PI_Controller_t position_pi = { POSITION_PI_KP, POSITION_PI_KI, POSITION_PI_INTEGRAL_SAT, POSITION_PI_OUT_UPPER, POSITION_PI_OUT_LOWER, 0.0f };

float pi_controller(PI_Controller_t *ctrl, float setpoint, float measurement, float dt)
{
	float error = setpoint - measurement;

	ctrl->integral += error * dt;

	float i_term = ctrl->integral * ctrl->Ki;

	if (i_term > ctrl->integral_saturation)
	{
		i_term = ctrl->integral_saturation;
		ctrl->integral = ctrl->integral_saturation / ctrl->Ki;
	}
	if (i_term < -ctrl->integral_saturation)
	{
		i_term = -ctrl->integral_saturation;
		ctrl->integral = -ctrl->integral_saturation / ctrl->Ki;
	}

	float out = ctrl->Kp * error + i_term;

	if (out > ctrl->output_upper_saturation)
	{
		out = ctrl->output_upper_saturation;
	}
	if (out < ctrl->output_lower_saturation)
	{
		out = ctrl->output_lower_saturation;
	}

	return out;

}
