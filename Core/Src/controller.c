#include "controller.h"
#include "foc.h"

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

void controller_task(void)
{
	static uint16_t speed_loop_counter;
	static uint16_t position_loop_counter;
	// Provides vq to follow the desired iq
	foc.vq = pi_controller(&iq_pi, foc.iq_ref, foc.iq, CURRENT_LOOP_DT);

	// Provides vd to follow the desired id
	foc.vd = pi_controller(&id_pi, ID_SETPOINT, foc.id, CURRENT_LOOP_DT);

	// Incrementing counter for scheduling speed control loop
	speed_loop_counter++;

	if (speed_loop_counter >= SPEED_LOOP_COUNT)
	{
		foc.iq_ref = pi_controller(&speed_pi, foc.speed_ref, encoder.angular_speed, SPEED_LOOP_DT);
		speed_loop_counter = 0;

		// Incrementing counter for scheduling position control loop
		position_loop_counter++;
		if (position_loop_counter >= POSITION_LOOP_COUNT)
			{
				foc.speed_ref = pi_controller(&position_pi, foc.position_ref, encoder.angle, POSITION_LOOP_DT);
				position_loop_counter = 0;
			}
	}
}

