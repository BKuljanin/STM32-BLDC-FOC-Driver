#include "bldc.h"

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


void bldc_enable_all(void)
{
    phase_enable(PHASE_U);
    phase_enable(PHASE_V);
    phase_enable(PHASE_W);
}

void bldc_disable_all(void)
{
    phase_disable(PHASE_U);
    phase_disable(PHASE_V);
    phase_disable(PHASE_W);
}

uint8_t bldc_init(void)
{
	static uint16_t align_count = 0;

	if (align_count == 0)
		bldc_enable_all();

	// Pull rotor d-axis to align with alpha (phase U). Same as inv-Park at theta=0 with Vd=ALIGNMENT_VD, Vq=0.
	// After settling we zero the encoder, so from here on encoder=0 means d=alpha.
	foc.v_alpha = ALIGNMENT_VD;
	foc.v_beta  = 0.0f;
	svpwm_update();

	align_count++;

	if (align_count >= ALIGNMENT_TICKS)
	{
		as5600_set_reference();
		align_count = 0;
		return 1;
	}

	return 0;
}
