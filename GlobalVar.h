
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

extern  U_EVENT_TRIG_FLAG_T  g_sFlagEve;              //事件触发标志位
extern  U_ERROR_FLAG_T       g_sFlagErr;              //故障位域
//extern  U_ERROR_T            g_sFlagError;            //故障位域
extern  S_FILT_DATA_T        g_sBusVdc;               //母线电压
extern  S_FILT_DATA_T        g_sBusIdc;               //母线电流
extern  S_FILT_DATA_T        g_sMosTmp;               //MOS温度
extern  E_SYS_STA_T          g_eSysState;             //系统工作状态
extern  S_MOTOR_T            g_sMotor;                //电机相关变量
extern  FNP_VOID_T           g_fnpaCommutationTab[7]; //换相表函数指针数组
extern  S_DRIVE_T            g_sDrive;                //驱动相关变量        
extern  S_PHASE_AD_CHANNEL_T g_sPhaseAdChannel;       //反电势相关变量
extern  E_ADC_CHANNEL_T      eaAdFifoChannelBuf[3];   //AD FIFO通道缓存
extern  S_FILT_DATA_T        g_sPeakIdc;              //峰值电压


extern  u8     g_u8CommAdcIndex;           //公共通道采样值 
extern  u8     g_u8FluxBemfStat;           //flux bemf切换状态
extern  u8     g_u8FluxToBemfFlag;         //采样通道切换
extern  u8     g_u8PwmToADelayTimeFlag;    //tmrd延时采样
extern  u16    g_u16LastBemfAd;            //上一次反电动势值
extern  u8     g_u8CommutationCnt;         //换相计数
extern  u8     g_u16TimeOutCnt;            //换相超时计数
extern  u8     g_u8NoAdcStartFlag;         //未执行完换相即使进入tmrd也不允许启动ADC
extern  u8     g_u8AdcIndex;
extern  u16    g_u16ADCBuffer[12];         //ADC buf
extern  u16    g_u16L_dIs_dt;              //flu值
extern  u8     g_bCommutationOK;
extern  u8     g_aNextPhase[7];
extern  u8     g_u16BlockCnt;              //堵转计数
extern  u16    u16TimerRegBuf;             //本次定时器值
extern  u16    g_aTzc[3];                  //过零时间数组  
extern  u16    u16LastZeroCrossTimerReg;   //本次过零定时器值
extern  u8     g_u8ZeroCrosscnt;           //反电动势过零计数
extern  u16    g_u16ZeroCompareVal;        //反电动势过零比较值

extern  u16    g_u16LastZeroTimerVal;      //上一次过零定时器值  
extern  u16    g_u16NowZeroTimerVal;       //本次过零定时器值

extern  E_ADC_CHANNEL_T  eaComAdChannel[4]; //公共通道缓存
extern  U_FLUX_AD_RES_T  uFluxAdRes;        //低占空比下AD FIFO结果缓存(和磁链运算变量关联)
extern  S_FLUX_VAR_T     uFluxVar;
extern  U_BEMF_AD_RES_T  uBemfAdRes;    //高占空比下AD FIFO结果缓存(有公共通道)
extern  S_INC_PI_PARA_T  g_sSpeedIncPI; //速度增量式PI
extern  PID_LocTypeDef   g_sSpeedLocPi; //速度直接式PI
extern  S_OBSTACL_T      g_sObstacl;    //避障结构体

//Debug 变量
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

extern volatile uint8_t hall_port_value;      //HALL端口值
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





























