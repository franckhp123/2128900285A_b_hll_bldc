
#ifndef __GOLBALVAR_H
#define __GOLBALVAR_H

#include "macrodriver.h"  
//#include "tmrj.h"
//#include "tmrd.h"
#include "adc.h"
#include "tau.h"
#include "PublicWare.h"
#include "BLDC.h"


//#include "fsl.h"
//#include "Fsl_usr.h"

extern  U_EVENT_TRIG_FLAG_T  g_sFlagEve;              //�¼�������־λ
extern  U_ERROR_FLAG_T       g_sFlagErr;              //����λ��
//extern  U_ERROR_T            g_sFlagError;            //����λ��
extern  S_FILT_DATA_T        g_sBusVdc;               //ĸ�ߵ�ѹ
extern  S_FILT_DATA_T        g_sBusIdc;               //ĸ�ߵ���
extern  S_FILT_DATA_T        g_sMosTmp;               //MOS�¶�
extern  E_SYS_STA_T          g_eSysState;             //ϵͳ����״̬
extern  S_MOTOR_T            g_sMotor;                //�����ر���
extern  FNP_VOID_T           g_fnpaCommutationTab[7]; //�������ָ������
extern  S_DRIVE_T            g_sDrive;                //������ر���        
extern  S_PHASE_AD_CHANNEL_T g_sPhaseAdChannel;       //��������ر���
extern  E_ADC_CHANNEL_T      eaAdFifoChannelBuf[3];   //AD FIFOͨ������
extern  S_FILT_DATA_T        g_sPeakIdc;              //��ֵ��ѹ


extern  u8     g_u8CommAdcIndex;           //����ͨ������ֵ 
extern  u8     g_u8FluxBemfStat;           //flux bemf�л�״̬
extern  u8     g_u8FluxToBemfFlag;         //����ͨ���л�
extern  u8     g_u8PwmToADelayTimeFlag;    //tmrd��ʱ����
extern  u16    g_u16LastBemfAd;            //��һ�η��綯��ֵ
extern  u8     g_u8CommutationCnt;         //�������
extern  u8     g_u16TimeOutCnt;            //���೬ʱ����
extern  u8     g_u8NoAdcStartFlag;         //δִ���껻�༴ʹ����tmrdҲ����������ADC
extern  u8     g_u8AdcIndex;
extern  u16    g_u16ADCBuffer[12];         //ADC buf
extern  u16    g_u16L_dIs_dt;              //fluֵ
extern  u8     g_bCommutationOK;
extern  u8     g_aNextPhase[7];
extern  u8     g_u16BlockCnt;              //��ת����
extern  u16    u16TimerRegBuf;             //���ζ�ʱ��ֵ
extern  u16    g_aTzc[3];                  //����ʱ������  
extern  u16    u16LastZeroCrossTimerReg;   //���ι��㶨ʱ��ֵ
extern  u8     g_u8ZeroCrosscnt;           //���綯�ƹ������
extern  u16    g_u16ZeroCompareVal;        //���綯�ƹ���Ƚ�ֵ

extern  u16    g_u16LastZeroTimerVal;      //��һ�ι��㶨ʱ��ֵ  
extern  u16    g_u16NowZeroTimerVal;       //���ι��㶨ʱ��ֵ

extern  E_ADC_CHANNEL_T  eaComAdChannel[4]; //����ͨ������
extern  U_FLUX_AD_RES_T  uFluxAdRes;        //��ռ�ձ���AD FIFO�������(�ʹ��������������)
extern  S_FLUX_VAR_T     uFluxVar;
extern  U_BEMF_AD_RES_T  uBemfAdRes;    //��ռ�ձ���AD FIFO�������(�й���ͨ��)
extern  S_INC_PI_PARA_T  g_sSpeedIncPI; //�ٶ�����ʽPI
extern  PID_LocTypeDef   g_sSpeedLocPi; //�ٶ�ֱ��ʽPI
extern  S_OBSTACL_T      g_sObstacl;    //���Ͻṹ��

//Debug ����
extern u8 d_u8UartRecvFinish;
extern u8 d_RecvBuf;

extern uint8_t uHallValue;
//extern const uint8_t HallTable[6]={1,3,2,6,4,5};
extern uint8_t MotorPosition;
extern uint8_t uHallValueExpect;
extern uint8_t HallAry[200];
extern uint8_t MotorRunningFlag;
extern uint8_t uTimeCntBlock;
extern uint8_t MotorDirection;
extern uint8_t s_u8BrkOnceFlag ;

extern volatile uint8_t hall_port_value;      //HALL�˿�ֵ
extern volatile u8 phaseSetor ;

extern uint16_t VrAdValue;
extern uint16_t TargetDuty;
extern uint16_t CurrentDuty;
extern uint8_t DischargeReq;

extern uint8_t mu8TestKey ;
extern uint16_t gu16OffDelay ;
extern uint8_t Flag_1ms;
extern u8 uart1_test_data;
extern uint16_t  block_cnt;



#endif





























