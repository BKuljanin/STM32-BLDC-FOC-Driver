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

/* Id current PI — output: vd [V], symmetric */
#define ID_PI_KP              0.5f
#define ID_PI_KI              5.0f
#define ID_PI_INTEGRAL_SAT    12.0f
#define ID_PI_OUT_UPPER       12.0f
#define ID_PI_OUT_LOWER      -12.0f

/* Iq current PI — output: vq [V], symmetric */
#define IQ_PI_KP              0.5f
#define IQ_PI_KI              5.0f
#define IQ_PI_INTEGRAL_SAT    12.0f
#define IQ_PI_OUT_UPPER       12.0f
#define IQ_PI_OUT_LOWER      -12.0f

/* Speed PI — output: iq_ref [A] */
#define SPEED_PI_KP           0.1f
#define SPEED_PI_KI           0.01f
#define SPEED_PI_INTEGRAL_SAT 5.0f
#define SPEED_PI_OUT_UPPER    5.0f
#define SPEED_PI_OUT_LOWER   -5.0f

/* Position PI — output: speed_ref [rad/s] */
#define POSITION_PI_KP              0.5f
#define POSITION_PI_KI              0.01f
#define POSITION_PI_INTEGRAL_SAT    50.0f
#define POSITION_PI_OUT_UPPER       100.0f
#define POSITION_PI_OUT_LOWER      -100.0f

extern PI_Controller_t id_pi;
extern PI_Controller_t iq_pi;
extern PI_Controller_t speed_pi;
extern PI_Controller_t position_pi;

float pi_controller(PI_Controller_t *ctrl, float setpoint, float measurement, float dt);

#endif /* INC_CONTROLLER_H_ */
