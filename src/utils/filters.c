#include "common.h"
#include "filters.h"
float angle, angle_dot;
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.5;
float dt = 0.005;
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = {0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter(float Accel, float Gyro)
{
	angle+=(Gyro - Q_bias) * dt;
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0];

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;
	Q_bias	+= K_1 * Angle_err;
	angle_dot   = Gyro - Q_bias;
}
/*
 * 平均滤波函数
 * 将新数据放入滤波器再求平均
 * 数据类型：unsigned short
 */
_u16 add_to_avg_filter_u16(_u16 new_data, _u16 * filter, size_t pos, size_t size)
{
    _s32 * sum = (_s32 *)(filter + size);

    *sum  -= (_s32)filter[pos];
    *sum  += (_s32)new_data;
    filter[pos] = new_data;

    return (_u16)((*sum)/size);
}

/*
 * 平均滤波函数
 * 将新数据放入滤波器再求平均
 * 数据类型：short
 */
_s16 add_to_avg_filter_s16(_s16 new_data, _s16 * filter, size_t pos, size_t size)
{
    _s32 * sum = (_s32 *)(filter + size);

    *sum  -= (_s32)filter[pos];
    *sum  += (_s32)new_data;
    filter[pos] = new_data;

    return (_s16)((*sum)/((_s32)size));
}

/*
 * 平均滤波函数
 * 将新数据放入滤波器再求平均
 * 数据类型：unsigned int
 */
_u32 add_to_avg_filter_u32(_u32 new_data, _u32 * filter, size_t pos, size_t size)
{
    _s64 * sum = (_s64 *)(filter + size);

    *sum  -= (_s64)filter[pos];
    *sum  += (_s64)new_data;
    filter[pos] = new_data;

    return (_u32)((*sum)/size);
}
