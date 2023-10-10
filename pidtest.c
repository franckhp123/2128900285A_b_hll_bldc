/**
  ******************************************************************************
  * @file    pid.c
  * @author  ye
  * @version V1.0.0
  * @date    2019-7-31
  * @brief   pid����
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "system.h"

/* Define ------------------------------------------------------------------*/

/* Parameter ------------------------------------------------------------------*/
//λ���Ͷ���
PID_LocTypeDef PID_Loc_1;
PID_LocTypeDef PID_Loc_2;
PID_LocTypeDef PID_Loc_BLADE;
PID_LocTypeDef yaw_Locpid;
//�����Ͷ���
PID_IncTypeDef PID_Inc_1;
PID_IncTypeDef PID_Inc_2;
PID_IncTypeDef PID_Inc_BLADE;
PID_IncTypeDef yaw_Incpid;

/* Function ------------------------------------------------------------------*/

/***************************************************************************
 * @fn          PID_Loc_Init
 *     
 * @brief       λ����pid��ʼ��
 *     
 * @data        2020��3��5��
 *     
 * @param       PID ----------- PID���ݽṹ
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
 * @brief       λ����pid��ʼ��
 *     
 * @data        2020��3��5��
 *     
 * @param       PID ----------- PID���ݽṹ
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
 * @brief       ������pid��ʼ��
 *     
 * @data        2020��3��5��
 *     
 * @param       PID ----------- PID���ݽṹ
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
 * @brief       PIDλ��(Location)����
 *     
 * @data        2020��3��5��
 *     
 * @param       SetValue ------ ����ֵ(����ֵ)
 *              ActualValue --- ʵ��ֵ(����ֵ)
 *              PID ----------- PID���ݽṹ
 *              
 * @return      PIDLoc -------- PIDλ��
 ***************************************************************************
 */ 
float PID_Loc(float SetValue, float ActualValue, PID_LocTypeDef *PID)
{
	float PIDLoc; 													//λ��
	 
	PID->Ek = SetValue - ActualValue;
	PID->LocSum += PID->Ek; 							 //�ۼ����
	 
	PIDLoc = PID->Kp * PID->Ek + PID->Ki * PID->LocSum + PID->Kd * (PID->Ek1 - PID->Ek);
	 
	PID->Ek1 = PID->Ek; 
	return PIDLoc;
}
/***************************************************************************
 * @fn          PID_Inc
 *     
 * @brief       PID����(Increment)����
 *     
 * @data        2020��3��5��
 *     
 * @param       SetValue ------ ����ֵ(����ֵ)
 *              ActualValue --- ʵ��ֵ(����ֵ)
 *              PID ----------- PID���ݽṹ
 *              
 * @return      PIDInc -------- ����PID����(+/-)
 ***************************************************************************
 */  
float PID_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{
	float PIDInc;                    //����
	 
	PID->Ek = SetValue - ActualValue;
	PIDInc = PID->Kp * (PID->Ek - PID->Ek1) + PID->Ki * PID->Ek + PID->Kd * (PID->Ek - 2*PID->Ek1 + PID->Ek2);
	 
	PID->Ek2 = PID->Ek1;
	PID->Ek1 = PID->Ek; 
	return PIDInc;
}
/***************************************************************************
 * @fn          PD_Inc
 *     
 * @brief       PID����(Increment)����
 *     
 * @data        2020��3��5��
 *     
 * @param       SetValue ------ ����ֵ(����ֵ)
 *              ActualValue --- ʵ��ֵ(����ֵ)
 *              PID ----------- PID���ݽṹ
 *              
 * @return      PIDInc -------- ����PID����(+/-)
 ***************************************************************************
 */
float PD_Inc(float SetValue, float ActualValue, PID_IncTypeDef *PID)
{
	float PIDInc;                    //����
	 
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
 * @brief       ������pid��ʼ��
 *     
 * @data        2020��3��5��
 *     
 * @param       PID ----------- PID���ݽṹ
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
 * @brief       ��ʼ��pid1
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void YAW_PID_init(void)
{
	//ʹ��λ���͵�pid����
	PID_Inc_YAWInit(&yaw_Incpid);
}
/***************************************************************************
 * @fn          PID_Init_1
 *     
 * @brief       ��ʼ��pid1
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void PID_init(void)
{
#ifdef Incremental_PID
	//ʹ�������͵�pid����
	PID_Inc_Init(&PID_Inc_1);	
#else
	//ʹ��λ���͵�pid����
	PID_Loc_Init(&PID_Loc_1);
#endif

}
/***************************************************************************
 * @fn          PID_Init_2
 *     
 * @brief       ��ʼ��pid2
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void PID_init_2(void)
{
#ifdef Incremental_PID
	//ʹ�������͵�pid����
	PID_Inc_Init(&PID_Inc_2);	
#else
	//ʹ��λ���͵�pid����
	PID_Loc_Init(&PID_Loc_2);	
#endif
}
/***************************************************************************
 * @fn          PID_init_BLADE
 *     
 * @brief       ��ʼ��pid2
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
void PID_init_BLADE(void)
{
#ifdef Incremental_PID
	//ʹ�������͵�pid����
	PID_Inc_Init_B(&PID_Inc_BLADE);	
#else
	//ʹ��λ���͵�pid����
	PID_Loc_Init_B(&PID_Loc_BLADE);	
#endif
}
/***************************************************************************
 * @fn          PID_realize
 *     
 * @brief       PIDʹ��
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float PID_realize(float set_val,float rel_val)
{
#ifdef Incremental_PID
	return PID_Inc(set_val,rel_val,&PID_Inc_1);  // ���������Ͳ���
#else
	return PID_Loc(set_val,rel_val,&PID_Loc_1);  // ����λ���Ͳ���	
#endif
}
/***************************************************************************
 * @fn          PID_realize_2
 *     
 * @brief       PIDʹ��
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float PID_realize_2(float set_val,float rel_val)
{
#ifdef Incremental_PID
	return PID_Inc(set_val,rel_val,&PID_Inc_2);  // ���������Ͳ���	
#else
	return PID_Loc(set_val,rel_val,&PID_Loc_2);  // ����λ���Ͳ���	
#endif
}
/***************************************************************************
 * @fn          PID_realize_BLADE
 *     
 * @brief       PIDʹ��
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float PID_realize_BLADE(float set_val,float rel_val)
{
#ifdef Incremental_PID
	return PID_Inc(set_val,rel_val,&PID_Inc_BLADE);  // ���������Ͳ���	
#else
	return PID_Loc(set_val,rel_val,&PID_Loc_BLADE);  // ����λ���Ͳ���	
#endif
}
/***************************************************************************
 * @fn          PID_realize
 *     
 * @brief       PIDʹ��
 *     
 * @data        2020��3��5��
 *     
 * @param       void
 *              
 * @return      void
 ***************************************************************************
 */
float YAW_PID_realize(float set_val,float rel_val)
{
//	return PID_Inc(set_val,rel_val,&yaw_Incpid);  // ���������Ͳ���
	return PD_Inc(set_val,rel_val,&yaw_Incpid);  // ���������Ͳ���
}
/*************************************************END OF FILE*********************************************/
