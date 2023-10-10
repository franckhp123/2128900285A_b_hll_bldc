#ifndef __PID_PID_H__
#define __PID_PID_H__

#include "stm32f10x.h"

//#define Incremental_PID

//λ����pid
typedef struct
{
	float Kp;        //����ϵ��Proportional
			
	float Ki;        //����ϵ��Integral
			
	float Kd;        //΢��ϵ��Derivative
											
	float Ek;        //��ǰ���
			
	float Ek1;       //ǰһ����� e(k-1)
			
	float Ek2;       //��ǰһ����� e(k-2)
			
	float LocSum;    //�ۼƻ���λ��
	
}PID_LocTypeDef;

//������pid
typedef struct
{
	float Kp;        //����ϵ��Proportional
	
	float Ki; 			 //����ϵ��Integral
	
	float Kd;				 //΢��ϵ��Derivative
		 
	float Ek;				 //��ǰ���
	
	float Ek1;			 //ǰһ����� e(k-1)
	
	float Ek2;			 //��ǰһ����� e(k-2)
	
}PID_IncTypeDef;
 
void PID_Loc_Init(PID_LocTypeDef *PID);
void PID_Inc_Init(PID_IncTypeDef *PID);
void PID_Loc_Init_B(PID_LocTypeDef *PID);
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef *PID);
float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID);
float PD_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID);
void YAW_PID_init(void);
void PID_Inc_YAWInit(PID_IncTypeDef *PID);
void PID_init(void);
void PID_init_2(void);
void PID_init_BLADE(void);
float PID_realize(float set_val,float rel_val);
float PID_realize_2(float set_val,float rel_val);
float PID_realize_BLADE(float set_val,float rel_val);
float YAW_PID_realize(float set_val,float rel_val);
#endif 
