#include "bldc.h"
//#include "pwm.h"
//#include "adc.h"
#include "as5600.h"
#include "timer.h"
#include <math.h>

uint8_t volatile step = 0;  // current commutation step (exposed for debugging)
float electrical_angle;
uint32_t commutation_done;
uint8_t sector;
uint8_t new_step;
BLDC_Direction_t bldc_direction = BLDC_REVERSE;

static void phase_enable(BLDC_Phase_t phase)
{
    switch(phase)
    {
    case PHASE_U:
        GPIOC->BSRR = (1U << 10);
        break;
    case PHASE_V:
        GPIOC->BSRR = (1U << 11);
        break;
    case PHASE_W:
        GPIOC->BSRR = (1U << 12);
        break;
    default:
        break;
    }
}

static void phase_disable(BLDC_Phase_t phase)
{
    switch(phase)
    {
    case PHASE_U:
        GPIOC->BSRR = (1U << (10 + 16));	// Reference manual p188. First 16 bits for setting next 16 for resetting. Atomic (no readaing)
        break;
    case PHASE_V:
        GPIOC->BSRR = (1U << (11 + 16));
        break;
    case PHASE_W:
        GPIOC->BSRR = (1U << (12 + 16));
        break;
    default:
        break;
    }
}

void bldc_update_step(void)
{
    float ea = encoder.angle * BLDC_POLE_PAIRS;
    while (ea >= 360.0f) ea -= 360.0f;
    electrical_angle = ea;

    float ea_adv = ea + 30.0f;
    if (ea_adv >= 360.0f) ea_adv -= 360.0f;

    sector = (uint8_t)(ea_adv / 60.0f) % 6;
    new_step = (sector + 1) % 6;
    step = new_step;
}

void bldc_run(uint32_t duty, CommutationMode_t mode)
{

}

// Initialize (park) BLDC by bringing PWM to one phase, one is used as sink and one floating
void bldc_init(CommutationMode_t mode) {

  }

// Drives BLDC like a stepper, cycling through 6 commutation steps with fixed delay. Only for testing
void bldc_test_run(uint32_t delay_ms, uint32_t duty_cycle)
{
    while(1)
    {

    }
}


void bldc_noise_floor_test(BLDC_Phase_t phase)
{
    phase_disable(PHASE_U);
    phase_disable(PHASE_V);
    phase_disable(PHASE_W);
    back_emf_float_channel(phase);
    while (1) {}
}

