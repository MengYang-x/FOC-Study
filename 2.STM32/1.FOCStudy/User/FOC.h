#ifndef __FOC_H
#define __FOC_H

void PWM_Init(void);	
void Set_PWM(uint16_t CCR1, uint16_t CCR2, uint16_t CCR3);
void setPhaseVoltage(float Uq, float Ud, float angle_el);	
#endif
