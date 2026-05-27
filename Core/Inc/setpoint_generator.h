#ifndef INC_SETPOINT_GENERATOR_H_
#define INC_SETPOINT_GENERATOR_H_

#include "foc.h"

#define POSITION_TARGET_RAD     (2.0f * PI)     // One full mechanical revolution
#define POSITION_RAMP_RATE      (PI / 2.0f)     // rad/s - how fast the setpoint moves toward target

float ramp_position_setpoint(float target, float ramp_rate, float dt);

#endif /* INC_SETPOINT_GENERATOR_H_ */
