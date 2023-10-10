/************************************************************
   includes
************************************************************/
#include "macrodriver.h"
#include "userdefine.h"
#include "BLDC.h"

U_EVENT_TRIG_FLAG_T   g_sFlagEve;        //事件触发位域
U_ERROR_FLAG_T        g_sFlagErr;        //故障位域   
//U_ERROR_T             g_sFlagError;      //故障位域          
S_FILT_DATA_T         g_sBusVdc;         //母线电压
S_FILT_DATA_T         g_sBusIdc;         //母线电流
S_FILT_DATA_T         g_sMosTmp;         //MOS温度
E_SYS_STA_T           g_eSysState;       //系统工作状态
S_MOTOR_T             g_sMotor;          //电机相关变量
S_DRIVE_T             g_sDrive;          //驱动相关变量
S_PHASE_AD_CHANNEL_T  g_sPhaseAdChannel; //导通相/反电势电压通道  
S_FILT_DATA_T         g_sPeakIdc;        //峰值电流



E_ADC_CHANNEL_T eaAdFifoChannelBuf[3];   //AD FIFO通道缓存  


//
FNP_VOID_T  g_fnpaCommutationTab[7]={0,0,0,0,0,0,0}; //换相表函数指针数组
u8          g_u8CommAdcIndex=0;         //公共通道采样值     
u8          g_u8FluxBemfStat=0;         //flux bemf切换状态
u8          g_u8FluxToBemfFlag=0;       //采样通道切换
u8          g_u8PwmToADelayTimeFlag=0;  //tmrd延时采样   
u16         g_u16LastBemfAd=0;          //上一次反电动势值
u8          g_u8CommutationCnt=0;       //换相计数
u8          g_u16TimeOutCnt=0;          //换相超时计数
u8          g_u8NoAdcStartFlag=0;       //未执行完换相即使进入tmrd也不允许启动ADC
u8          g_u8AdcIndex = 0;           //PWM-on AD采样计数  
u16         g_u16ADCBuffer[12]={0};     //ADC buf  
u16         g_u16L_dIs_dt=0;            //flu值   
u8          g_bCommutationOK=0;
u8          g_aNextPhase[7]={0,3,6,2,5,1,4};   //下一个相位查表  
u8          g_u16BlockCnt=0;               //堵转计数
u16         u16TimerRegBuf = 0;            //本次定时器值
u16         u16LastZeroCrossTimerReg = 0;  //本次过零定时器值

u16         g_aTzc[3]={0,0,0};             //过零时间数组    
//u16         g_aTzc[6]={0,0,0,0,0,0};             //过零时间数组


u8          g_u8ZeroCrosscnt=0;            //反电动势过零计数
u16         g_u16ZeroCompareVal=0;         //反电动势过零比较值    

u16         g_u16LastZeroTimerVal=0;       //上一次过零定时器值  
u16         g_u16NowZeroTimerVal=0;        //本次过零定时器值

U_BEMF_AD_RES_T  uBemfAdRes={0,0,0,0,0,0,0,0}; //高占空比下AD FIFO结果缓存(有公共通道)     
E_ADC_CHANNEL_T  eaComAdChannel[4] = {MosTmp_ADCH,IAvr_ADCH,BusVdc_ADCH,UVW_CMM_ADCH}; //公共通道缓存
//E_ADC_CHANNEL_T  eaComAdChannel[2] = {MosTmp_ADCH,IAvr_ADCH}; //公共通道缓存

U_FLUX_AD_RES_T  uFluxAdRes={0,0,0,0}; //低占空比下AD FIFO结果缓存(和磁链运算变量关联)
S_FLUX_VAR_T     uFluxVar={0,0,0};     //磁链运算相关变量实际值

//S_INC_PI_PARA_T  g_sSpeedIncPI={0,0,0,0,0,0}; //速度增量式PI       
S_INC_PI_PARA_T  g_sSpeedIncPI; //速度增量式PI 
PID_LocTypeDef   g_sSpeedLocPi; //速度直接式PI

S_OBSTACL_T      g_sObstacl;    //避障结构体      

//u8 MosOverTmpErrCnt=0; //mos高温故障计数器

volatile u8 phaseSetor = 0;
   


//Debug 变量
//u8 d_u8UartRecvFinish=0;
//u8 d_RecvBuf=0;

uint8_t uHallValue;
//const uint8_t HallTable[6]={1,3,2,6,4,5};
uint8_t MotorPosition;
uint8_t uHallValueExpect;
uint8_t HallAry[200];
uint8_t MotorRunningFlag;
uint8_t uTimeCntBlock;
uint8_t MotorDirection;
volatile uint8_t hall_port_value=0;      //HALL端口值
uint16_t VrAdValue;
uint16_t TargetDuty;
uint16_t CurrentDuty;
uint8_t DischargeReq;

uint8_t mu8TestKey = 0;
uint16_t gu16OffDelay = 0;
uint8_t Flag_1ms=0;
u8 uart1_test_data=0;
 uint8_t s_u8BrkOnceFlag = 0;
 uint16_t  block_cnt=0;

#if 0
//变量-----------------------------------
uint8_t     u8Flag_1ms = 0;




















uint8_t     g_u8LedShowMode=0;       // 开机电量显示模式 不同电压范围显示灯个数
uint8_t     g_u8MosCheckCnt=0;       //mos自检次数
uint8_t     g_u8ReadFlashMosCheck=0; //读取flash中记录MOS自检结果









S_FILT_DATA_T   g_sBusVdc;  //母线电压
S_FILT_DATA_T   g_sVR;      //调速信号
S_FILT_DATA_T   g_sBatTemp; //电池温度
S_FILT_DATA_T   g_sMosTemp; //MOS温度

//存贮专用
fsl_descriptor_t my_fsl_descriptor_t;
fsl_write_t      my_fsl_write_t;
uint8_t          data_buff[4]={0,0,0,0};
uint32_t         Parameter=0;
uint16_t         BlankAddress=0;




#endif

















