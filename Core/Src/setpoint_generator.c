#include "setpoint_generator.h"
#include "math.h"

float ramp_position_setpoint(float target, float ramp_rate, float dt)
{
    static float current = 0.0f;
    float step = ramp_rate * dt;

    if (current < target)       current = fminf(current + step, target);
    else if (current > target)  current = fmaxf(current - step, target);

    return current;
}
