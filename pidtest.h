#ifndef __PID_PID_H__
#define __PID_PID_H__

#include "stm32f10x.h"

//#define Incremental_PID

//位置型pid
typedef struct
{
	float Kp;        //比例系数Proportional
			
	float Ki;        //积分系数Integral
			
	float Kd;        //微分系数Derivative
											
	float Ek;        //当前误差
			
	float Ek1;       //前一次误差 e(k-1)
			
	float Ek2;       //再前一次误差 e(k-2)
			
	float LocSum;    //累计积分位置
	
}PID_LocTypeDef;

//增量型pid
typedef struct
{
	float Kp;        //比例系数Proportional
	
	float Ki; 			 //积分系数Integral
	
	float Kd;				 //微分系数Derivative
		 
	float Ek;				 //当前误差
	
	float Ek1;			 //前一次误差 e(k-1)
	
	float Ek2;			 //再前一次误差 e(k-2)
	
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
