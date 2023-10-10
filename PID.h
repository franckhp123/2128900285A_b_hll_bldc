
#ifndef __PID_H
#define __PID_H

#include "macrodriver.h"
#include "userdefine.h"
#include "PublicWare.h"
#include "GlobalVar.h" 


/* 速度环 PI 参数 */  
//#define SPEED_Kp                        0.05 //P比例    
#define SPEED_Kp                        0.01 //P比例       
#define SPEED_Ki                        0.02 //I积分  
#define SPEED_Kc                        0.05  //该值用在抗积分饱和算法中, 用于退饱和, 增量式算法不需要此参数
#define SPEED_OutMin                    cDUTY_MIN   // 输出最小值限幅
#define SPEED_OutMax                    cDUTY_FULL  // 输出最大值限幅    


/* 速度环增量式PI参数*/
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



























