
#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H

#include "macrodriver.h"
#include "userdefine.h"
#include "PublicWare.h"
#include "BLDC.h"
#include "GlobalVar.h"  
#include "sau.h"
#include "tau.h"
#include "PID.h"
#include "r_cg_it.h"

//�궨��
//#define cSTANDBY_1ms_CNT         40
#define cSTANDBY_1ms_CNT         20  //��ʼ����ʱ,����״̬ʱ���޸�Ϊ5ms 
#define cBOOT_1ms_CNT            40
#define cPOSITION_1ms_CNT        10
#define cSTART_1ms_CNT           1
#define cRUN_1ms_CNT             10 
#define cSTOP_1ms_CNT            10 
#define cBRAKE_1ms_CNT           10 
#define cERROR_1ms_CNT           10 


void Main_MotorTask(void);
void Motor_Standby_Act(void); //��������ִ��

void Motor_Run_Act(void);     //���ж���ִ��
void Motor_Stop_Act(void);     //ͣ������ִ��
void Motor_Brake_Act(void);     //ɲ������ִ��
void Motor_Error_Act(void);      //���϶���ִ��
void Motor_StateCnt(u8 *pStatCnt); //״̬����


void Task_DisableIntDependOnMotorStat(void); //�رչؼ��ж�
void MotorWalkDisp_InRunSta(void);  //����״̬�µĵ������
void MotorCurrentSamp(void);
void RstVarBeforInErrSta(void);
void ErrorStateHandle(void);
void BusVdcLowErrRelieve(void); //ĸ�ߵ�ѹ�͹��Ͻ��
void BusCurOCErrRelieve(void);
void MosTmpHighErrRelieve(void); 
void ObstaclesErrRelieve(void);
void ObstacleCheck(void);
void GetMotionGrade(void); //��û����¶�


















#endif































