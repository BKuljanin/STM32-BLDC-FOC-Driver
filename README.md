# STM32 BLDC FOC Driver
### Field Oriented Control + Position/Speed/Current Cascade + SVPWM (NUCLEO-F446RE + IHM07M1)

**Note:**
This project implements Field Oriented Control (FOC) for a brushless DC motor on the STM32F446RE. It uses the X-Nucleo IHM07M1 gate driver shield with a 2-shunt current sensing topology and an AS5600 magnetic encoder for rotor position. The full control chain: Clarke/Park transforms, three-loop PI cascade, inverse Park, and SVPWM run inside the ADC injected conversion interrupt at 20 kHz.

---

## This project demonstrates

- **Field Oriented Control** — decoupled torque (Iq) and flux (Id) control in the rotating d/q reference frame
- **Space Vector PWM** — sector detection, normalized T1/T2 times, center aligned symmetric output
- **Three loop PI cascade** — position (20 Hz) → speed (200 Hz) → current (20 kHz)
- **2 shunt resistors current sensing** — injected ADC triggered at the PWM peak for low side sampling
- **AS5600 14 bit absolute magnetic encoder** on I2C1

---

## Hardware Overview

### MCU: STM32F446RET6 (NUCLEO-F446RE)
- Core: ARM Cortex-M4, running at 180 MHz

### Motor Driver: X-Nucleo IHM07M1
- Three half-bridges (L6230 gate drivers)
- Phase enable pins control each gate driver independently
- On-board 0.33 Ω shunt resistors with 1.53× op-amp gain for current sensing

### Motor: A2208/14T 1400 KV Brushless Outrunner
- **Pole pairs:** 7

### Encoder: AS5600 (absolute magnetic, I2C)
- 12-bit output resolution: 4096 counts/revolution

### Power Supply
- VBUS: **12 V**

---

## High-Level Flow

```
TIM1 CC4 falling edge (at PWM peak, 20 kHz)
  └─ ADC1 injected conversion completes
       └─ HAL_ADCEx_InjectedConvCpltCallback
            │
            ├─ [State 1] current_init_done == 0
            │     └─ Accumulate ADC offsets over 100 samples for calibration
            │
            ├─ [State 2] bldc_init_done == 0
            │     └─ bldc_init() every tick
            │           ├─ Inject v_alpha = 1.2V, v_beta = 0 → svpwm_update()
            │           ├─ Hold for 20000 ticks (1 second)
            │           └─ as5600_set_reference() to preset encoder
            │
            └─ [State 3] Normal operation
                  └─ foc_update()
                        ├─ calculate_currents()         // ADC raw → amps
                        ├─ clarke_transform()            // i_u/v/w → i_alpha/beta
                        ├─ park_transform()              // i_alpha/beta → id/iq
                        ├─ controller_task()             // PI cascade → vd, vq
                        ├─ inverse_park_transform()      // vd/vq → v_alpha/beta
                        └─ svpwm_update()                // v_alpha/beta → duty U/V/W

TIM3 IRQ (1 kHz)
  └─ as5600_trigger_read()         // start I2C read
       └─ I2C completion → as5600_calculate_speed()
```

---

## FOC Chain

The entire control chain executes in a single ADC ISR with a ~25 µs window between the PWM peak and the start of the next PWM cycle.

### Clarke Transform

Converts three-phase currents to the stationary α/β frame. The 1.5 scaling factor is removed to preserve current amplitude:

```
i_alpha = (i_u − 0.5*(i_v + i_w)) / 1.5
i_beta  = (√3/2 * (i_v − i_w)) / 1.5
```

### Park Transform

Rotates α/β into the rotor-fixed d/q frame using the electrical angle from the encoder:

```
id =  i_alpha*cos(θ_e) + i_beta*sin(θ_e)
iq = −i_alpha*sin(θ_e) + i_beta*cos(θ_e)
```

### PI Cascade

| Loop     | Rate    | Input              | Output      |
|---------:|--------:|--------------------|-------------|
| Current  | 20 kHz  | id_ref=0, iq_ref   | vd, vq [V]  |
| Speed    | 200 Hz  | speed_ref [rad/s]  | iq_ref [A]  |
| Position | 20 Hz   | position_ref [rad] | speed_ref   |

### Inverse Park Transform

Rotates vd/vq back to the stationary frame:

```
v_alpha = vd*cos(θ_e) − vq*sin(θ_e)
v_beta  = vd*sin(θ_e) + vq*cos(θ_e)
```

### SVPWM

Projects v_alpha/v_beta onto the three phase axes to determine sector and compute dwell times T1, T2 for the two adjacent active vectors. Both are normalized by VBUS so T1+T2 ∈ [0,1]. Remaining time T0 is split symmetrically (Tz = T0/2) giving center aligned symmetric output.

The PWM is configured as center aligned mode 1 with PWM mode 1 (active high). This places V0 at the counter peak (CNT=ARR) and V7 at the counter ends, which is better suited for low side shunt sensing since the ADC fires during V0 when both shunts carry current.

---

## Motor Alignment Sequence

On startup, before entering the normal FOC loop, the rotor is pulled to a known electrical position:

1. Enable all three phases
2. Apply v_alpha = 1.2 V, v_beta = 0 directly (equivalent to inv-Park at θ=0 with Vd=1.2V, Vq=0 — creates a stator field along the U-phase axis)
3. Hold for 1 second
4. Call `as5600_set_reference()` presets encoder. Encoder zero is now aligned with the rotor d-axis
5. Enter normal FOC operation

---

## Current Sensing

2 shunt resistors topology on the low side. Phase U (PA0, ADC1_IN0) and phase V (PC1, ADC1_IN11) are measured directly; phase W is reconstructed by KCL:

```
i_w = −i_u − i_v
```

The ADC injected group is triggered by TIM1_CH4 falling edge (CCR4 = ARR−1), which fires just before the counter peak. At that moment all high-side switches are off and both shunts carry their respective phase currents.


---

## Clock Configuration

| Clock               | Frequency  | Notes                            |
|--------------------:|-----------:|----------------------------------|
| HSI                 | 16 MHz     | PLL source                       |
| PLL                 | —          | M=8, N=180, P=2                  |
| SYSCLK              | 180 MHz    |                                  |
| AHB                 | 180 MHz    | DIV1                             |
| APB2 (TIM1, ADC1)   | 90 MHz     | DIV2 (timer clock: 180 MHz)      |
| APB1 (I2C1, TIM3)   | 45 MHz     | DIV4 (timer clock: 90 MHz)       |

---

## Pinout

### PWM & Phase Enable

| Signal | Pin  | Peripheral  | Description                |
|-------:|------|-------------|----------------------------|
| PWM_U  | PA8  | TIM1_CH1    | Phase U high side PWM      |
| PWM_V  | PA9  | TIM1_CH2    | Phase V high side PWM      |
| PWM_W  | PA10 | TIM1_CH3    | Phase W high side PWM      |
| EN_U   | PC10 | GPIO Output | Phase U gate driver enable |
| EN_V   | PC11 | GPIO Output | Phase V gate driver enable |
| EN_W   | PC12 | GPIO Output | Phase W gate driver enable |

### Current Sense ADC

| Phase | Pin | ADC Channel  | Description         |
|------:|-----|--------------|---------------------|
| U     | PA0 | ADC1_IN0 / JDR1 | Phase U current  |
| V     | PC1 | ADC1_IN11 / JDR2| Phase V current  |
| W     | —   | —            | KCL calculation  |

### Encoder (AS5600, I2C1)

| Signal | Pin | Description          |
|-------:|-----|----------------------|
| SCL    | PB6 | I2C1 clock (100 kHz) |
| SDA    | PB7 | I2C1 data            |

---

## PWM Configuration

| Parameter      | Value          | Notes                                        |
|---------------:|---------------:|----------------------------------------------|
| Frequency      | 20 kHz         |                                              |
| ARR            | 4500           | Center-aligned: period = 2×ARR cycles        |
| Prescaler      | 0              | Timer clock = 180 MHz                        |
| Mode           | Center-aligned 1 | Counter counts up then down               |
| Max duty cycle | 80%            |                                              |
| ADC trigger    | TIM1_CH4       | CCR4 = 4499, falling edge just before peak   |

---

## AS5600 Encoder Interface

| Parameter      | Value        | Notes                                          |
|---------------:|-------------:|------------------------------------------------|
| Interface      | I2C1         | 100 kHz                                        |
| I2C address    | 0x36         | 7 bit                                          |
| Angle register | 0x0C–0x0D   | 12 bit raw angle                               |
| Read rate      | 1 kHz        | TIM3 interrupt starts I2C read         |
| Angle LP filter| 30 Hz        | Filters angle                    |
| Speed LP filter| 20 Hz        | Applied to angle delta / dt                    |

---

## Initialization Sequence

1. SystemClock_Config (180 MHz via HSI PLL)
2. GPIO init (EN_U, EN_V, EN_W on PC10–12)
3. TIM1 init — center-aligned 20 kHz PWM, CCRs to zero
4. ADC1 init — injected group, 2-channel, TIM1_CH4 trigger
5. I2C1 init → AS5600 init
6. TIM3 init — 1 kHz encoder read interrupt
7. Start TIM1 PWM output, start ADC injected conversions
8. ADC ISR takes over — offset calibration → alignment → FOC

---

## File Structure

### `main.c`
Peripheral init and system startup only. No control logic.

### `foc.c / foc.h`
The full FOC math chain.
- `clarke_transform()` — i_u/v/w → i_alpha/beta
- `park_transform()` — i_alpha/beta → id/iq using encoder angle
- `inverse_park_transform()` — vd/vq → v_alpha/beta
- `svpwm_update()` — v_alpha/beta → sector detection → T1/T2 → duty cycles U/V/W
- `foc_update()` — calls the full chain in order

### `controller.c / controller.h`
PI cascade and scheduling.
- `pi_controller()` — PI library
- `controller_task()` — runs current loop every call, speed loop every 100 calls, position loop every 1000 calls

### `adc.c / adc.h`
Current sensing and ADC ISR state machine.
- `MX_ADC1_Init()` — injected 2-channel setup, TIM1_CH4 trigger
- `calculate_currents()` — raw ADC → amps with offset correction
- `HAL_ADCEx_InjectedConvCpltCallback()` — 3 state ISR: calibration → alignment → FOC

### `bldc.c / bldc.h`
Phase enable/disable and motor alignment.
- `bldc_init()` — called every ADC tick during alignment, returns 1 when done
- `bldc_enable_all() / bldc_disable_all()` — gate driver enable pins

### `pwm.c / pwm.h`
TIM1 center aligned PWM.
- `MX_TIM1_Init()` — configures TIM1 CH1–3 at 20 kHz, CH4 as ADC trigger
- `tim1_pwm_set_duty_percent()` — converts duty % to CCR with upper/lower clamp

### `as5600.c / as5600.h`
Encoder driver.
- `as5600_trigger_read()` — starts I2C burst read
- `as5600_calculate_speed()` — delta-angle → filtered speed
- `as5600_set_reference()` — zeros encoder at current position

### `trigonometry.c / trigonometry.h`
sine LUT with linear interpolation.
- `compute_sin_cos()` — returns sin and cos from a single LUT lookup

### `setpoint_generator.c / setpoint_generator.h`
Position ramp generator.
- `ramp_position_setpoint()` — generates setpoint toward target at a fixed rate, called at the position loop rate

### `timer.c / timer.h`
TIM3 for encoder reads.

---

## Sampling Rates

| Task                    | Rate    | Trigger                          |
|------------------------:|--------:|----------------------------------|
| PWM switching           | 20 kHz  | TIM1 center-aligned              |
| Current sensing / FOC   | 20 kHz  | TIM1_CH4 → ADC injected ISR     |
| Speed PI                | 200 Hz  | Every 100 current loop calls     |
| Position PI             | 20 Hz   | Every 10 speed loop calls        |
| Encoder read            | 1 kHz   | TIM3 interrupt → I2C             |

---

## Reference Materials

- **Reference Manual:** RM0390 Rev 7 (STM32F446xx)
- **Datasheet:** DS10693 Rev 10 (STM32F446RE)
- **User Manual:** UM1724 Rev 17 (NUCLEO-F446RE)
- **Cortex-M4 Generic User Guide:** DUI0553A
- AS5600 Datasheet — ams AG
- IHM07M1 User Manual — UM1943 Rev 4
