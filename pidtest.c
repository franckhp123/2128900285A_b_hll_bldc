/**
  ******************************************************************************
  * @file    pid.c
  * @author  ye
  * @version V1.0.0
  * @date    2019-7-31
  * @brief   pid计算
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "system.h"

/* Define ------------------------------------------------------------------*/

/* Parameter ------------------------------------------------------------------*/
//位置型定义
PID_LocTypeDef PID_Loc_1;
PID_LocTypeDef PID_Loc_2;
PID_LocTypeDef PID_Loc_BLADE;
PID_LocTypeDef yaw_Locpid;
//增量型定义
PID_IncTypeDef PID_Inc_1;
PID_IncTypeDef PID_Inc_2;
PID_IncTypeDef PID_Inc_BLADE;
PID_IncTypeDef yaw_Incpid;

/* Function ------------------------------------------------------------------*/

/***************************************************************************
 * @fn          PID_Loc_Init
 *     
 * @brief       位置型pid初始化
 *     
 * @data        2020年3月5日
 *     
 * @param       PID ----------- PID数据结构
 *              
 * @return      void
 ***************************************************************************
 */
void PID_Loc_Init(PID_LocTypeDef *PID)
{
	PID->Ek = 0.0;
	PID->Ek1 = 0.0;
	PID->Ek2 = 0.0;
	PID->LocSum = 0.0;
	PID->Kp = 0.003;
	PID->Ki = 0.005;
	PID->Kd = 0.0;
}
/***************************************************************************
 * @fn          PID_Loc_Init
 *     
 * @brief       位置型pid初始化
 *     
 * @data        2020年3月5日
 *     
 * @param       PID ----------- PID数据结构
 *              
 * @return      void
 ***************************************************************************
 */
void PID_Loc_Init_B(PID_LocTypeDef *PID)
{
	PID->Ek = 0.0;
	PID->Ek1 = 0.0;
	PID->Ek2 = 0.0;
	PID->LocSum = 0.0;
	PID->Kp = 0.08;
	PID->Ki = 0.002;
	PID->Kd = 0.00382;
}
/***************************************************************************
 * @fn          PID_Inc_Init
 *     
 * @brief       增量型pid初始化
 *     
 * @data        2020年3月5日
 *     
 * @param       PID ----------- PID数据结构
 *              
 * @return      void
 ***************************************************************************
 */ 
void PID_Inc_Init(PID_IncTypeDef *PID)
{
	PID->Ek = 0.0;
	PID->Ek1 = 0.0;
	PID->Ek2 = 0.0;
	PID->Kp = 0.03;
	PID->Ki = 0.015;                          
	PID->Kd = 0.0;
}
/***************************************************************************
 * @fn          PID_Loc
 *     
 * @brief       PID位置(Location)计算
 *     
 * @data        2020年3月5日
 *     
 * @param       SetValue ------ 设置值(期望值)
 *              ActualValue --- 实际值(反馈值)
 *              PID ----------- PID数据结构
 *              
 * @return      PIDLoc -------- PID位置
 ***************************************************************************
 */ 
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef *PID)
{
	float PIDLoc; 													//位置
	 
	PID->Ek = SetValue - ActualValue;
	PID->LocSum += PID->Ek; 							 //累计误差
	 
	PIDLoc = PID->Kp * PID->Ek + PID->Ki * PID->LocSum + PID->Kd * (PID->Ek1 - PID->Ek);
	 
	PID->Ek1 = PID->Ek; 
	return PIDLoc;
}
/***************************************************************************
 * @fn          PID_Inc
 *     
 * @brief       PID增量(Increment)计算
 *     
 * @data        2020年3月5日
 *     
 * @param       SetValue ------ 设置值(期望值)
 *              ActualValue --- 实际值(反馈值)
 *              PID ----------- PID数据结构
 *              
 * @return      PIDInc -------- 本次PID增量(+/-)
 ***************************************************************************
 */  
float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{
	float PIDInc;                    //增量
	 
	PID->Ek = SetValue - ActualValue;
	PIDInc = PID->Kp * (PID->Ek - PID->Ek1) + PID->Ki * PID->Ek + PID->Kd * (PID->Ek - 2*PID->Ek1 + PID->Ek2);
	 
	PID->Ek2 = PID->Ek1;
	PID->Ek1 = PID->Ek; 
	return PIDInc;
}
/***************************************************************************
 * @fn          PD_Inc
 *     
 * @brief       PID增量(Increment)计算
 *     
 * @data        2020年3月5日
 *     
 * @param       SetValue ------ 设置值(期望值)
 *              ActualValue --- 实际值(反馈值)
 *              PID ----------- PID数据结构
 *              
 * @return      PIDInc -------- 本次PID增量(+/-)
 ***************************************************************************
 */
float PD_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{
	float PIDInc;                    //增量
	 
	PID->Ek = ActualValue - SetValue;
	PID->Ek2 = PID->Ek - PID->Ek1;
	
	PIDInc = PID->Ek1 * PID->Kp + PID->Ek2 * PID->Kd;
	
	PID->Ek1 = PID->Ek;
	
		if(PIDInc > 1000) PIDInc = 1000;
		else if(PIDInc < -1000) PIDInc = -1000;	
	return PIDInc;
}
/***************************************************************************
 * @fn          PID_Inc_Init
 *     
 * @brief       增量型pid初始化
 *     
 * @data        2020年3月5日
 *     
 * @param       PID ----------- PID数据结构
 *              
 * @return      void
 ***************************************************************************
 */ 
void PID_Inc_YAWInit(PID_IncTypeDef *PID)	
{
	PID->Ek = 0.0;
	PID->Ek1 = 0.0;
	PID->Ek2 = 0.0;
	PID->Kp = 10.0;//100.0;
	PID->Ki = 0.0;//100.0;                          
	PID->Kd = 0.0;//6.18;
}
/***************************************************************************
 * @fn          PID_Init_1
 *     
 * @brief       初始化pid1
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void YAW_PID_init(void)
{
	//使用位置型的pid计算
	PID_Inc_YAWInit(&yaw_Incpid);
}
/***************************************************************************
 * @fn          PID_Init_1
 *     
 * @brief       初始化pid1
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void PID_init(void)
{
#ifdef Incremental_PID
	//使用增量型的pid计算
	PID_Inc_Init(&PID_Inc_1);	
#else
	//使用位置型的pid计算
	PID_Loc_Init(&PID_Loc_1);
#endif

}
/***************************************************************************
 * @fn          PID_Init_2
 *     
 * @brief       初始化pid2
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void PID_init_2(void)
{
#ifdef Incremental_PID
	//使用增量型的pid计算
	PID_Inc_Init(&PID_Inc_2);	
#else
	//使用位置型的pid计算
	PID_Loc_Init(&PID_Loc_2);	
#endif
}
/***************************************************************************
 * @fn          PID_init_BLADE
 *     
 * @brief       初始化pid2
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void PID_init_BLADE(void)
{
#ifdef Incremental_PID
	//使用增量型的pid计算
	PID_Inc_Init_B(&PID_Inc_BLADE);	
#else
	//使用位置型的pid计算
	PID_Loc_Init_B(&PID_Loc_BLADE);	
#endif
}
/***************************************************************************
 * @fn          PID_realize
 *     
 * @brief       PID使用
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float PID_realize(float set_val,float rel_val)
{
#ifdef Incremental_PID
	return PID_Inc(set_val,rel_val,&PID_Inc_1);  // 返回增量型参数
#else
	return PID_Loc(set_val,rel_val,&PID_Loc_1);  // 返回位置型参数	
#endif
}
/***************************************************************************
 * @fn          PID_realize_2
 *     
 * @brief       PID使用
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float PID_realize_2(float set_val,float rel_val)
{
#ifdef Incremental_PID
	return PID_Inc(set_val,rel_val,&PID_Inc_2);  // 返回增量型参数	
#else
	return PID_Loc(set_val,rel_val,&PID_Loc_2);  // 返回位置型参数	
#endif
}
/***************************************************************************
 * @fn          PID_realize_BLADE
 *     
 * @brief       PID使用
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float PID_realize_BLADE(float set_val,float rel_val)
{
#ifdef Incremental_PID
	return PID_Inc(set_val,rel_val,&PID_Inc_BLADE);  // 返回增量型参数	
#else
	return PID_Loc(set_val,rel_val,&PID_Loc_BLADE);  // 返回位置型参数	
#endif
}
/***************************************************************************
 * @fn          PID_realize
 *     
 * @brief       PID使用
 *     
 * @data        2020年3月5日
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float YAW_PID_realize(float set_val,float rel_val)
{
//	return PID_Inc(set_val,rel_val,&yaw_Incpid);  // 返回增量型参数
	return PD_Inc(set_val,rel_val,&yaw_Incpid);  // 返回增量型参数
}
/*************************************************END OF FILE*********************************************/
