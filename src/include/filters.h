#pragma once
#include "common.h"
extern float angle, angle_dot;
void Kalman_Filter(float Accel, float Gyro);
_u16 add_to_avg_filter_u16(_u16 new_data, _u16 * filter, size_t pos, size_t size);
_s16 add_to_avg_filter_s16(_s16 new_data, _s16 * filter, size_t pos, size_t size);
_u32 add_to_avg_filter_u32(_u32 new_data, _u32 * filter, size_t pos, size_t size);
