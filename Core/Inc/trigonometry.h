#ifndef INC_TRIGONOMETRY_H_
#define INC_TRIGONOMETRY_H_

extern const float sin_lut[256];

void compute_sin_cos(float angle_rad, float *sin_out, float *cos_out);

#endif /* INC_TRIGONOMETRY_H_ */
