#include <math.h>
#include "Serial.h"
#include "tim.h"

const float PI = 3.1415926535f;
const float PI_2 = 1.5707963267f;
const float PI_3 = 1.047197551f;
const float _2PI = 6.283185307f;
const float sqrt3 = 1.73205080756f;
const int PWM_ARR = 280; // PWM的计数周期
const float Udc = 12.4f; // 电机的母线电压

// normalizing radian angle to [0, 2pi]
float _normalizeAngle(float angle)
{
	float a = fmod(angle, _2PI); // fmod()函数用于浮点数的取余运算
	return a >= 0.0f ? a : (a + _2PI);
}

// 使能TIMx的通道y
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

// 输出PWM
void Set_PWM(uint16_t CCR1, uint16_t CCR2, uint16_t CCR3)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, CCR1 * PWM_ARR);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, CCR2 * PWM_ARR);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, CCR3 * PWM_ARR);
}

// FOC核心函数：输入Uq、Ud和电角度，输出三路PWM
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Ts = 1.0f;
	float Ta, Tb, Tc;
	float t1, t2, t3, t4, t5, t6, t7;
	float sum, k_svpwm;

	angle_el = _normalizeAngle(angle_el);

	// Park逆变换
	float U_alpha = -Uq * sin(angle_el) + Ud * cos(angle_el);
	float U_beta = Uq * cos(angle_el) + Ud * sin(angle_el);

	// 扇区判断
	float K = sqrt3 * Ts / Udc; // SVPWM调制比
	float u1 = U_beta * K;
	float u2 = (0.8660254f * U_alpha - 0.5f * U_beta) * K; // sqrt(3)/2 = 0.8660254
	float u3 = (-0.8660254f * U_alpha - 0.5f * U_beta) * K;

	uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2); // sector = A + 2B + 4C

	// 非零矢量和零矢量作用时间的计算
	switch (sector)
	{
	case 3: // 扇区1
		t4 = u2;
		t6 = u1;
		sum = t4 + t6;
		if (sum > Ts) // 过调制处理
		{
			k_svpwm = Ts / sum;
			t4 *= k_svpwm;
			t6 *= k_svpwm;
		}
		t7 = (Ts - t4 - t6) / 2.0f;
		Ta = t4 + t6 + t7;
		Tb = t6 + t7;
		Tc = t7;
		break;
	case 1: // 扇区2
		t2 = -u2;
		t6 = -u3;
		sum = t2 + t6;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t2 *= k_svpwm;
			t6 *= k_svpwm;
		}
		t7 = (Ts - t2 - t6) / 2.0f;
		Ta = t6 + t7;
		Tb = t2 + t6 + t7;
		Tc = t7;
		break;
	case 5: // 扇区3
		t2 = u1;
		t3 = u3;
		sum = t2 + t3;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t2 *= k_svpwm;
			t3 *= k_svpwm;
		}
		t7 = (Ts - t2 - t3) / 2.0f;
		Ta = t7;
		Tb = t2 + t3 + t7;
		Tc = t3 + t7;
		break;
	case 4: // 扇区4
		t1 = -u1;
		t3 = -u2;
		sum = t1 + t3;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t1 *= k_svpwm;
			t3 *= k_svpwm;
		}
		t7 = (Ts - t1 - t3) / 2.0f;
		Ta = t7;
		Tb = t3 + t7;
		Tc = t1 + t3 + t7;
		break;
	case 6: // 扇区5
		t1 = u3;
		t5 = u2;
		sum = t1 + t5;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t1 *= k_svpwm;
			t5 *= k_svpwm;
		}
		t7 = (Ts - t1 - t5) / 2.0f;
		Ta = t5 + t7;
		Tb = t7;
		Tc = t1 + t5 + t7;
		break;
	case 2: // 扇区6
		t4 = -u3;
		t5 = -u1;
		sum = t4 + t5;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t4 *= k_svpwm;
			t5 *= k_svpwm;
		}
		t7 = (Ts - t4 - t5) / 2.0f;
		Ta = t4 + t5 + t7;
		Tb = t7;
		Tc = t5 + t7;
		break;
	default:
		break;
	}

	printf("[Ta,Tb,Tc]:%f,%f,%f\r\n", Ta, Tb, Tc);
}
