#ifndef __PWM_Driver_H__
#define __PWM_Driver_H__
     
extern void InitPWM(uint8_t PWM_no);
extern void PWM_Stop(uint8_t PWM_no);
extern void PWM_Servo(uint8_t PWM_no, uint16_t Servo_HiTime);
#endif




