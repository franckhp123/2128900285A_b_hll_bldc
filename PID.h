
#ifndef __PID_H
#define __PID_H

#include "macrodriver.h"
#include "userdefine.h"
#include "PublicWare.h"
#include "GlobalVar.h" 


/* �ٶȻ� PI ���� */  
//#define SPEED_Kp                        0.05 //P����    
#define SPEED_Kp                        0.01 //P����       
#define SPEED_Ki                        0.02 //I����  
#define SPEED_Kc                        0.05  //��ֵ���ڿ����ֱ����㷨��, �����˱���, ����ʽ�㷨����Ҫ�˲���
#define SPEED_OutMin                    cDUTY_MIN   // �����Сֵ�޷�
#define SPEED_OutMax                    cDUTY_FULL  // ������ֵ�޷�    


/* �ٶȻ�����ʽPI����*/
#define SPEED_K1_Q16            ((int32_t)(65536.0L*SPEED_Kp*(1.0+SPEED_Ki)))
#define SPEED_K2_Q16            ((int32_t)(-65536.0L*SPEED_Kp))
#define SPEED_LastErr           0          
#define SPEED_OUT               0
#define SPEED_OutMin_Q16        ((int32_t)(65536.0L * SPEED_OutMin))
#define SPEED_OutMax_Q16        ((int32_t)(65536.0L * SPEED_OutMax))











s32 IncrementPI_Ctrl(s16 ref,s16 fdb,S_INC_PI_PARA_T *ps);  
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef *PID);


void SpeedIncPI_Init(void); 
void SpeedLocPI_Init(void); 





#endif



























