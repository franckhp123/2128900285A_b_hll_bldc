
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

//宏定义
//#define cSTANDBY_1ms_CNT         40
#define cSTANDBY_1ms_CNT         20  //开始启动时,待机状态时间修改为5ms 
#define cBOOT_1ms_CNT            40
#define cPOSITION_1ms_CNT        10
#define cSTART_1ms_CNT           1
#define cRUN_1ms_CNT             10 
#define cSTOP_1ms_CNT            10 
#define cBRAKE_1ms_CNT           10 
#define cERROR_1ms_CNT           10 


void Main_MotorTask(void);
void Motor_Standby_Act(void); //待机动作执行

void Motor_Run_Act(void);     //运行动作执行
void Motor_Stop_Act(void);     //停机动作执行
void Motor_Brake_Act(void);     //刹车动作执行
void Motor_Error_Act(void);      //故障动作执行
void Motor_StateCnt(u8 *pStatCnt); //状态计数


void Task_DisableIntDependOnMotorStat(void); //关闭关键中断
void MotorWalkDisp_InRunSta(void);  //工作状态下的电机处理
void MotorCurrentSamp(void);
void RstVarBeforInErrSta(void);
void ErrorStateHandle(void);
void BusVdcLowErrRelieve(void); //母线电压低故障解除
void BusCurOCErrRelieve(void);
void MosTmpHighErrRelieve(void); 
void ObstaclesErrRelieve(void);
void ObstacleCheck(void);
void GetMotionGrade(void); //获得机器坡度


















#endif































