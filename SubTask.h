
#ifndef __SUBTASK_H
#define __SUBTASK_H

#include "macrodriver.h"  
#include "port.h"
#include "userdefine.h"
#include "GlobalVar.h"
#include "sau.h"
#include "PublicWare.h"
#include "GlobalVar.h"
#include "BLDC.h"

//宏定义
#define cADMosHighTmp 688
#define cADMosNorTmp  380

#define cOCValLow    50      //一档:5A(放大10倍)/20S过流保护 
#define cOCTimeLow   2000  
#define cOCValMid    70      //二挡:7A/5S  
#define cOCTimeMid   500
#define cOCValHigh   90      //三挡:9A/500ms  
#define cOCTimeHigh  50

#define cNorCurVal   30      //正常电流3A/3S清过流标志
#define cNorCurTime  300

//函数声明
void Sub_LoopTask(void);
//void Sub_Unclass(void);        // 0 未分类任务  
static void Sub_MotorDrive(void);     // 1 驱动
void Sub_Current(void);                // 2 电流        
static void Sub_Voltage(void);         // 3 电压
static void Sub_Temperature(void);   // 4 温度
static void Sub_HMI(void);              // 5 人机接口
static void Sub_WireComm(void);     // 6 有线通讯(UART,CAN,LIN)
static void Sub_Wireless(void);        // 7 无线通讯
static void Sub_Record(void);          // 8 记录再存贮
static void Sub_Reserved(void);       // 9 预留


void MosTmpErrChk(S_FILT_DATA_T *psMosTmp,U_ERROR_FLAG_T *psflag);

static void TaskAndStaChg_InSlefChk(void);
static void TaskAndStaChg_InWait(void);
static void TaskAndStaChg_InRun(void);
static void TaskAndStaChg_InStop(void);
static void TaskAndStaChg_InErr(void);
void MosSelfChk(void);
void RstVarBeforInRunSta(void);
void RstVarBeforInStopSta(void); //恢复进入刹车停机状态前的变量初始值



#endif
































