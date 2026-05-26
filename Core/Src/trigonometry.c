#include <stdint.h>
#include "trigonometry.h"

void compute_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    // Constant multiplier to scale radians directly to array indices:
    // 256 entries / (2 * PI) = 40.7436654f
    float index_raw = angle_rad * 40.7436654f;

    // Boundary constraint loop: enforce range between 0.0f and 256.0f
    while(index_raw >= 256.0f) index_raw -= 256.0f;
    while(index_raw < 0.0f)    index_raw += 256.0f;

    // Split raw float index into the low integer step and fractional distance
    uint8_t idx_low = (uint8_t)index_raw;
    uint8_t idx_high = idx_low + 1;
    float fractional = index_raw - (float)idx_low;

    // Sin linear interpolation
    float s_low  = sin_lut[idx_low];
    float s_high = sin_lut[idx_high];
    // Formula: Low_Value + Fraction * (High_Value - Low_Value)
    *sin_out = s_low + fractional * (s_high - s_low);

    // Cos linear interpolation
    // Cos leads sine by 90 degrees, which is 64 indices forward
    uint8_t c_idx_low  = idx_low + 64;
    uint8_t c_idx_high = c_idx_low + 1;

    float c_low  = sin_lut[c_idx_low];
    float c_high = sin_lut[c_idx_high];
    *cos_out = c_low + fractional * (c_high - c_low);
}
