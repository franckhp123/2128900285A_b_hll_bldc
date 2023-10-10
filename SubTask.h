
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

//�궨��
#define cADMosHighTmp 688
#define cADMosNorTmp  380

#define cOCValLow    50      //һ��:5A(�Ŵ�10��)/20S�������� 
#define cOCTimeLow   2000  
#define cOCValMid    70      //����:7A/5S  
#define cOCTimeMid   500
#define cOCValHigh   90      //����:9A/500ms  
#define cOCTimeHigh  50

#define cNorCurVal   30      //��������3A/3S�������־
#define cNorCurTime  300

//��������
void Sub_LoopTask(void);
//void Sub_Unclass(void);        // 0 δ��������  
static void Sub_MotorDrive(void);     // 1 ����
void Sub_Current(void);                // 2 ����        
static void Sub_Voltage(void);         // 3 ��ѹ
static void Sub_Temperature(void);   // 4 �¶�
static void Sub_HMI(void);              // 5 �˻��ӿ�
static void Sub_WireComm(void);     // 6 ����ͨѶ(UART,CAN,LIN)
static void Sub_Wireless(void);        // 7 ����ͨѶ
static void Sub_Record(void);          // 8 ��¼�ٴ���
static void Sub_Reserved(void);       // 9 Ԥ��


void MosTmpErrChk(S_FILT_DATA_T *psMosTmp,U_ERROR_FLAG_T *psflag);

static void TaskAndStaChg_InSlefChk(void);
static void TaskAndStaChg_InWait(void);
static void TaskAndStaChg_InRun(void);
static void TaskAndStaChg_InStop(void);
static void TaskAndStaChg_InErr(void);
void MosSelfChk(void);
void RstVarBeforInRunSta(void);
void RstVarBeforInStopSta(void); //�ָ�����ɲ��ͣ��״̬ǰ�ı�����ʼֵ



#endif
































