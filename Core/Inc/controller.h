#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

typedef struct {
    float Kp;
    float Ki;
    float integral_saturation;
    float output_upper_saturation;
    float output_lower_saturation;
    float integral;
} PI_Controller_t;

/* Id current PI output: vd [V] */
#define ID_PI_KP              0.5f
#define ID_PI_KI              5.0f
#define ID_PI_INTEGRAL_SAT    12.0f
#define ID_PI_OUT_UPPER       12.0f
#define ID_PI_OUT_LOWER      -12.0f

/* Iq current PI output: vq [V] */
#define IQ_PI_KP              0.5f
#define IQ_PI_KI              5.0f
#define IQ_PI_INTEGRAL_SAT    12.0f
#define IQ_PI_OUT_UPPER       12.0f
#define IQ_PI_OUT_LOWER      -12.0f

/* Speed PI output: iq_ref [A] */
#define SPEED_PI_KP           0.1f
#define SPEED_PI_KI           0.01f
#define SPEED_PI_INTEGRAL_SAT 5.0f
#define SPEED_PI_OUT_UPPER    5.0f
#define SPEED_PI_OUT_LOWER   -5.0f

/* Position PI output: speed_ref [rad/s] */
#define POSITION_PI_KP              0.5f
#define POSITION_PI_KI              0.01f
#define POSITION_PI_INTEGRAL_SAT    50.0f
#define POSITION_PI_OUT_UPPER       100.0f
#define POSITION_PI_OUT_LOWER      -100.0f

// Iq is minimized to keep stator 90 deg ahead of rotor
#define ID_SETPOINT 0.0f

/* PI task frequencies
 * ADC sample happens in the middle of center aligned PWM 20 kHz frequency.
 * Speed loop is called at 200 Hz (100 current PIs are called between 2 speed loop calls)
 * Position loop is called at 20 Hz (10 speed PIs are called between 2 position loop calls) */
#define SPEED_LOOP_COUNT 100
#define POSITION_LOOP_COUNT 10

#define CURRENT_LOOP_DT (0.000050f) 	// 50 microseconds represented in seconds (1 / 20000 Hz)
#define SPEED_LOOP_DT (CURRENT_LOOP_DT * SPEED_LOOP_COUNT)		// Bigger delta_t than current loop
#define POSITION_LOOP_DT (SPEED_LOOP_DT * POSITION_LOOP_COUNT)	// Bigger delta_t than speed loop


extern PI_Controller_t id_pi;
extern PI_Controller_t iq_pi;
extern PI_Controller_t speed_pi;
extern PI_Controller_t position_pi;

float pi_controller(PI_Controller_t *ctrl, float setpoint, float measurement, float dt);
void controller_task(void);

#endif /* INC_CONTROLLER_H_ */
