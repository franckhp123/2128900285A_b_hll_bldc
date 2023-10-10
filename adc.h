
#ifndef __BLDC_H
#define __BLDC_H

#include "iodefine.h"
#include "macrodriver.h" 
#include "userdefine.h"
#include "PublicWare.h"
#include "adc.h"
#include "tau.h"
#include "tmrd.h"


//类型定义
typedef struct
{
   u8 u8Standby;   // 0 待机
   u8 u8Boot;      // 1 自举
   u8 u8Position;  // 2 定位
   u8 u8Run;       // 3 闭环运行
   u8 u8Stop;      // 4 停机
   u8 u8Brake;     // 5 刹车
   u8 u8Error;     // 6 故障
}U_MOTOR_STA_T; //电机工作状态类型

typedef struct
{
   u16 bStandbyFinish   :1;  // 0  待机状态完成,允许进入其他状态
   u16 bBootFinsih      :1;  // 1  自举状态完成,允许进入其他状态
   u16 bPositionFinish  :1;  // 2  定位角度状态完成,允许进入其他状态
   u16 bStartFinish     :1;  // 3  启动状态完成,允许进入其他状态
   u16 bRunFinish       :1;  // 4  闭环运行状态完成,允许进入其他状态
   u16 bStopFinish      :1;  // 5  停机状态完成,允许进入其他状态
   u16 bBrakeFinish     :1;  // 6  刹车状态完成,允许进入其他状态
   u16 bErrorFinish     :1;  // 7  故障状态完成,允许进入其他状态
   u16 bAlarmFinish     :1;  // 8  告警状态完成,允许进入其他状态
   u16 bPowerOffFinish  :1;  // 9  关机掉电状态完成,允许进入其他状态
   u16 reserved10       :1;  // 10 预留
   u16 reserved11		:1;  // 11 预留
   u16 reserved12		:1;  // 12 预留
   u16 reserved13		:1;  // 13 预留
   u16 reserved14		:1;  // 14 预留
   u16 reserved15		:1;  // 15 预留
}U_MOTOR_STAT_FLAG_T; //电机状态切换事件标志类型

typedef struct  //电机状态切换事件触发位
{
   u16 bInsertImpulse    :1;  // 0  需要注入脉冲标志
   u16 bFluxStartRead    :1;  // 1  开始读取磁链标志
   u16 bFluxToBemf       :1;  // 2  置从磁链切换到高速标志
   u16 bDisBemfCalcu     :1;  // 3  禁止反电势运算标志
   u16 bBemfRising       :1;  // 4  反电势上升标志
   u16 bBemfFalling      :1;  // 5  反电势下降标志
   u16 bGetBemfZeroCross :1;  // 6  已获得过零点标志
   u16 bDelayTime        :1;  // 7  换相成功
   u16 bfluxCommuSucess  :1;  // 8  磁链换相成功标志                 
   u16 bRecvFinish       :1;  // 9  数据接收完成标志
   u16 bCrash            :1;  // 10 碰撞标志
   u16 reserved11        :1;  // 11 预留
   u16 reserved12        :1;  // 12 预留
   u16 reserved13        :1;  // 13 预留
   u16 reserved14        :1;  // 14 预留
   u16 reserved15        :1;  // 15 预留
}U_MOTOR_WORK_FLAG_T; //电机工作标志类型

typedef enum
{
   eSTANDBY_ACT,   // 0 停机
   eBOOT_ACT,      // 1 自举
   ePOSITION_ACT,  // 2 定位角度
   eSTART_ACT,     // 3 启动
   eRUN_ACT,       // 4 运行
   eSTOP_ACT,      // 5 停机
   eBRAKE_ACT,     // 6 刹车
   eERROR_ACT,     // 7 故障
}E_MOTOR_STAT_T;

typedef enum
{
   eNULL_PHASE = 0, //无效相位
   eUV = 1,
   eWU = 2,
   eWV = 3,
   eVW = 4,
   eUW = 5,
   eVU = 6,
}E_MOTOR_PHASE_T; //BLDC相位

typedef enum  //CCW和CW定义:从电机非主轴端且与轴平行的视角去看,电机主轴是
{
   eCW,   //正转
   eCCW,  //反转
   eMID   //中间
}E_MOTOR_DIR_T;  //电机旋转方向

typedef struct
{
  u8   u8SampCnt; // 采样数量
  u8   u8Shift;   // 移位除法
  u8   u8Cnt;     // 累加计数
  u16  u16Max;    // 无符号最大值
  u16  u16Min;    // 无符号最小值
  u32  u32Sum;    // 无符号累加值
  s32  i32Max;    // 有符号最大值
  s32  i32Min;    // 有符号最小值
  s32  i32Sum;    // 有符号累加值
}S_EXTREMUM_MEAN_FILT_T; // 去极值平均滤波结构体  

typedef struct
{
   U_MOTOR_STA_T           uStaCnt;        //状态时间计数器  
   U_MOTOR_STAT_FLAG_T     uStatFlag;      //状态标志
   U_MOTOR_WORK_FLAG_T     uWorkFlag;      //工作标志
   E_MOTOR_DIR_T           eDir;           //工作转向
   E_MOTOR_DIR_T           eDirBuf;        //工作转向缓存
   E_MOTOR_DIR_T           eTargetDir;     //目标转向
   E_MOTOR_STAT_T          eAct;           //当前动作状态
   uint8_t                 u8Phase;        //当前相位
   S_EXTREMUM_MEAN_FILT_T  sSpeedFilt;     //电机速度去极值平均滤波     
   u16                     u16Speed;       //当前转速
   u16                     u16RefSpeed;    //给定转速(速度环给定)
   u16                     u16TargetSpeed; //目标转速(用户给定)
   u8                      u8TimeCntForceChgPhase; //强制换相时间
   u8                      u8TimeCntFluxToForce;   //磁链切强拉时间计数器
   u8                      u8MotorRecvBuf[6];      //接收主机数据缓存
   u8                      u8MotorSendBuf[4];      //发送主机数据缓存
   u8                      RecvCnt;                //接收主机数据计数器
   u16                     u16BrakeTime;            //刹车时间
   u16                     u16TimeInterPI;         //进入PI环时间
   u16                     u16ZeroCurAD;           //电机零点电流AD
   u16                     u16CurrADBuf[4];        //电流缓存AD
   u8                      u8MotorStatus;          //电机状态
   u8                      u8MotorErrStaTxCnt;     //电机故障状态发送次数
   u16                     PIRefSpeedUpValue;      //PI环给定速度步进值  
   u8                      u8StartChagedPhaseCnt;  //启动换相次数 
   u8                      u8PWMPerionIntCnt;      //PWM周期中断计数器  
   u8                      u8WorkModeInRun;        //电机工作模式  
   u16                     u16TimeUpInRun;         //工作模式下电机自增工作时间(非PI操作)
}S_MOTOR_T;  //电机结构体

//AD通道枚举    
typedef enum
{
   ADC_CH0  = _00_AD_INPUT_CHANNEL_0,  //AD0   BEMFV
   ADC_CH1  = _01_AD_INPUT_CHANNEL_1,  //AD1   UVW-COMM
   ADC_CH2  = _02_AD_INPUT_CHANNEL_2,  //AD2   BUSVDC
   ADC_CH3  = _03_AD_INPUT_CHANNEL_3,  //AD3   ADS
   ADC_CH16 = _10_AD_INPUT_CHANNEL_16, //AD16 
   ADC_CH17 = _11_AD_INPUT_CHANNEL_17, //AD17  MOSTMP
   ADC_CH18 = _12_AD_INPUT_CHANNEL_18, //AD18  BEMFW
   ADC_CH19 = _13_AD_INPUT_CHANNEL_19, //AD19  BEMFU
   ADC_PGAO = _14_AD_INPUT_PGAO,       //ADPGAO  
}E_ADC_CHANNEL_T; //ADC通道

typedef struct
{
   E_ADC_CHANNEL_T eaConduction[6]; //位置号对应的导通相通道表
   E_ADC_CHANNEL_T eaBemf[6];       //位置号对应的反电势通道表
}S_PHASE_AD_CHANNEL_T; //BLDC相位AD通道结构体类型        

typedef struct
{
   //u16 pwmDuty;             //当前占空比
   //u16 u16RefPwmDuty;       //给定占空比
   u16 u16CurrPwmDuty;     //当前占空比
   u16 u16CurrPwmDutyBuf;  //当前占空比缓存(用在PWM中断中)
   u16 u16TargetPwmDuty;   //目标占空比
}S_DRIVE_T;//驱动结构体(驱动模式占空比档位等)

typedef struct
{
   u32 u16Conduction;     //导通相电压
   u32 u16Bemf;           //反电势相电压
   u32 u16PhaseCurr;      //相电流
}S_FLUX_VAR_T; //磁链运算结构体变量类型

typedef enum
{
   eSTAG0_1stINT,  // 0 阶段0 第1次中断   
   eSTAG0_2stINT,  // 1 阶段0 第2次中断
   eSTAG1_1stINT,  // 2 阶段1 第1次中断
   eSTAG1_2stINT,  // 3 阶段1 第2次中断
   eSTAG2_1stINT,  // 4 阶段2 第1次中断
   eSTAG2_2stINT,  // 5 阶段2 第2次中断
}E_MOTOR_RUN_SUB_STAT_T; //电机运行子状态

//磁链相关
#define FLUX_COMMUATION_ON_CNT  3     //阶段0 磁链换相成功数阈值(达到这个值将从阶段0切换到阶段1)
#define STAG0_FLUX_TIMEOUT_CNT  500   //阶段0 磁链换相超时时间值(*PWM周期US) 20=1ms
#define STAG1_FLUX_TIMEOUT_CNT  250   //阶段1 磁链换相超时时间值(*PWM周期US)

//强拉换相相关
#define FORCE_COMMUATION_ON_CNT  3         //阶段0 强拉换相成功数阈值(达到这个值将从阶段0切换到阶段1)
#define STAG0_FORCE_TIMEOUT_CNT  500       //阶段0 强拉换相超时时间值(*PWM周期US) 20=1ms
#define STAG1_FORCE_TIMEOUT_CNT  250       //阶段1 强拉换相超时时间值(*PWM周期US)
#define FORCE_TO_BEMF_ZERO_CROSS_OK_CNT 6  //强拉阶段连续过N次过零点检测成功后从强拉切换到反电势(该值不能<6)


//反电势相关
#define FLUX_TO_BEMF_ZERO_CROSS_OK_CNT 3 //磁链阶段连续过N次过零点检测成功后从磁链切换到反电势(改值不能<3)


//函数声明 
void UH_VL(void);
void WH_VL(void);
void UH_WL(void);
void WH_UL(void);
void VH_WL(void);
void VH_UL(void);
void PwrDrv_UV_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_UW_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_VW_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_VU_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_WU_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_WV_On(E_MOTOR_DIR_T eMotorDir);
void NullPhase(void);
void UH_VL_HPwmLON(void);
void UH_WL_HPwmLON(void);
void VH_WL_HPwmLON(void);
void VH_UL_HPwmLON(void);
void WH_UL_HPwmLON(void);
void WH_VL_HPwmLON(void);
void BLDC_ConsumeBrake(void); 
void BLDC_StopMotor(void);
void PWM_DutyUpdata(u16 duty);  
void PwmDrv_PwmOutDisable(void);//关6管  
void BLDC_CommutationTableLoad(FNP_VOID_T fnpaCommutationTab[],E_MOTOR_DIR_T eMotorDir);//载入换相表
void BLDC_PhaseConducting(const FNP_VOID_T fnpaCommutationTab[],u8 u8Phase); //根据初始位置得出下一个导通相
void BLDC_PhaseAdChannelConfig(E_MOTOR_DIR_T eMotorDir,S_PHASE_AD_CHANNEL_T * psPhaseAdChannel);
void BLDC_u8ImpulsePosition_2Phase3Dir_Fix(E_MOTOR_DIR_T eMotorDir); //脉冲定位(初始位置检测)两相导通,3方向
u8 BLDC_u8ImpulsePosition_2Phase6Dir_Fix(E_MOTOR_DIR_T eMotorDir,u16 invaildThrval);//脉冲定位(初始位置检测)两相导通,6方向
u8 Func_u8JudgeOneDataLessthanThrVal(const s16 a [ ],u8 ln,s16 thrval);//判断数组中是否有一个数小于规定阈值
u8 Func_u8FindMaxDataPosition_DirectCompare(const s16 a[],u8 ln);//用直接比较方式寻找一维数组中的最大值的位置
void BLDC_FluxCalcuCommutation(S_MOTOR_T *psMotor,const U_FLUX_AD_RES_T *puFluxAdres);
u16 u16FluxNullCalcu(const S_FLUX_VAR_T *psFluxVar,uint16_t L_dIs_dt); //无效相位,空计算
u16 BLDC_u16FluxFormulaCalcu(const S_FLUX_VAR_T *psFluxVar,u16 L_dIs_dt);
u16 BLDC_u16FluxFormulaCalcu2(const S_FLUX_VAR_T * psFluxVar,u16 L_dIs_dt);
u8 BLDC_u8BemfZeroCrossDetect_LowDuty(S_MOTOR_T * psMotor,uint16_t u16BusVdcHalfAd,uint16_t u16BemfAd);
u8 BLDC_u8BemfZeroCrossDetect_MidDuty(S_MOTOR_T *psMotor,u16 BusVdcHalfAd,u16 BemfAd);
u8 BLDC_u8BemfZeroCrossDetect_MidHighDuty(S_MOTOR_T *psMotor,u16 u16BusVdcHalfAd,u16 aAdcRes[ ]);
u8 BLDC_u8BemfZeroCrossDetect_HighDuty(S_MOTOR_T * psMotor,u16 u16BusVdcHalfAd,u16 aAdcRes[ ]);
u8 BLDC_u8ZeroCrossTimeToSpeed(S_MOTOR_T *psMotor,u16 aZeroCrossTime[]); 
void BLDC_ForceChangePhase(S_MOTOR_T *psMotor);
void HallISR(void);
uint8_t Hall_Get(void);
uint8_t GetMotorPosition(uint8_t hallvalue);
void Commutate_Phase(uint8_t Position);
void Self_Commutate_Phase(uint8_t Position);
void Motor_Stop(void);
void Motor_Brake(void);
void Cap_PreCharge(uint8_t Position);
uint8_t Expect_Hall_Value(uint8_t Position);
void Normal_Change_Phase(void);
void Pwm_Update(void);
uint16_t PwmDutyAdjStepCtrl(uint16_t targetAdjPwmDuty, uint16_t currAdjPwmDuty);
void PWM_DutyUpdata(u16 duty); //4799    
extern uint8_t int_flag;

#define   CW     0
#define   CCW    1
#define AH_IO	0b00100000
#define	BH_IO	0b00001000
#define CH_IO	0b00000100

//#define AL_IO	0b00000001
//#define BL_IO	0b00000010
//#define CL_IO	0b00010000

#define AL_IO	0b00010000
#define BL_IO	0b00000010
#define CL_IO	0b00000001
#define AH_PWM	0b11111101
#define	BH_PWM	0b11101111
#define CH_PWM	0b11011111

//#define AH_PWM	0b11111011
//#define BH_PWM	0b11110111
//#define CH_PWM	0b11011111



#define AL_PWM	0b11110111
#define BL_PWM	0b10111111
#define CL_PWM	0b01111111

/* 10K Frequency PWM macro define */
/* PWM duty set for open loop speed */

#define	PWM_ADJUST_LOW	(PWM_MAX/20)   //%5
#define	PWM_ADJUST_HIGH	(PWM_MAX*80L/100)  //80%    2158
#define  TRUE		1
#define  FALSE		0


#endif

#define PWM_DELTAP	(PWM_MAX/1000)  
#define PWM_DELTAN	(PWM_MAX/10)
#define DUTY_STEP	(PWM_MAX/500)
#define PWM_ZERO (0) //

/* Hall ON OFF */
#define	HALL_INT_ON() 	R_INTC0_Start();R_INTC4_Start();R_INTC3_Start()
#define	HALL_INT_OFF() 	R_INTC0_Stop();R_INTC4_Stop();R_INTC3_Stop()
void DelayXus(uint16_t us);

#define PWM_PERIOD  4799

void ClosePwmAndIo(void);


#define PWM_MAX (PWM_PERIOD)       //2698

#define DUTY_FULL               		PWM_MAX//4799//TRDGRA0 // 满占空比
#define DUTY_MIN                        ((uint16_t)(DUTY_FULL * 0.1))//7.2   // 最小占空比

#define DUTY_8                       	((uint16_t)(DUTY_FULL * 0.08))
#define DUTY_10                      	((uint16_t)(DUTY_FULL * 0.10))
#define DUTY_15                      	((uint16_t)(DUTY_FULL * 0.15))
#define DUTY_18                      	((uint16_t)(DUTY_FULL * 0.18))
#define DUTY_20                      	((uint16_t)(DUTY_FULL * 0.20))
#define DUTY_25                      	((uint16_t)(DUTY_FULL * 0.25))
#define DUTY_30                      	((uint16_t)(DUTY_FULL * 0.30))
#define DUTY_35                      	((uint16_t)(DUTY_FULL * 0.35))
#define DUTY_40                      	((uint16_t)(DUTY_FULL * 0.40))
#define DUTY_45                      	((uint16_t)(DUTY_FULL * 0.45))
#define DUTY_50                      	((uint16_t)(DUTY_FULL * 0.50))
#define DUTY_51                      	((uint16_t)(DUTY_FULL * 0.51))
#define DUTY_60                      	((uint16_t)(DUTY_FULL * 0.60))
#define DUTY_70                      	((uint16_t)(DUTY_FULL * 0.70))
#define DUTY_80                      	((uint16_t)(DUTY_FULL * 0.80))
#define DUTY_85                      	((uint16_t)(DUTY_FULL * 0.85))
#define DUTY_90                      	((uint16_t)(DUTY_FULL * 0.90))
#define DUTY_95                         ((uint16_t)(DUTY_FULL * 0.95))
#define DUTY_100                      	((uint16_t)(DUTY_FULL))  //4799




#define PWM_ZERO (0) //































   PwmDrv_PwmOutDisable();
   PwrDrv_UW_On(eMotorDir);
   Delay_us(IMPULSE_INJECT_TIME_uS);
   PwmDrv_PwmOutDisable();
   
//第二轮:采集WV VU UW方向电流,VW UV WU用于抵消
   PwrDrv_WV_On(eMotorDir);    
   Delay_us(IMPULSE_INJECT_TIME_uS);
   iWV = AdcSampSingleChannelOnce(IPeak_ADCH); //读取电流脉冲AD值  
   PwmDrv_PwmOutDisable();  
   PwrDrv_VW_On(eMotorDir);
   Delay_us(IMPULSE_INJECT_TIME_uS);
   PwmDrv_PwmOutDisable();

   PwrDrv_VU_On(eMotorDir);
   Delay_us(IMPULSE_INJECT_TIME_uS);
   iVU = AdcSampSingleChannelOnce(IPeak_ADCH); //读取电流脉冲AD值 
   PwmDrv_PwmOutDisable();
   PwrDrv_UV_On(eMotorDir);
   Delay_us(IMPULSE_INJECT_TIME_uS);
   PwmDrv_PwmOutDisable();

   PwrDrv_UW_On(eMotorDir);   
   Delay_us(IMPULSE_INJECT_TIME_uS);
   iUW = AdcSampSingleChannelOnce(IPeak_ADCH); //读取电流脉冲AD值 
   PwmDrv_PwmOutDisable();
   PwrDrv_WU_On(eMotorDir);   
   Delay_us(IMPULSE_INJECT_TIME_uS);
   PwmDrv_PwmOutDisable();  //脉冲结束关6管  
   g_sFlagEve.Bits.bMosCheckFlag = 0;

//根据三个方向电流大小关系确定转子所在扇区   
   if(Func_u8JudgeOneDataLessthanThrVal(aImpulse,6,invaildThrval) == 1) //有结果为无效数据
   { return 0; }
   else //结果为有效数据,正常位置下判断电流大小关系        
   {
      if     (iVW>iUV && iUV>iWU && iVU>iWV && iWV>iUW) return 1; //大小中,中大小 
	  else if(iWU>iUV && iUV>iVW && iVU>iUW && iUW>iWV) return 3; //小大中,小大中
	  else if(iWU>iVW && iVW>iUV && iWV>iUW && iUW>iVU) return 2; //大小中,大小中
      else if(iUV>iVW && iVW>iWU && iWV>iVU && iVU>iUW) return 6; //大中小,大中小
      else if(iUV>iWU && iWU>iVW && iUW>iVU && iVU>iWV) return 4; //小大中,小大中  
	  else if(iVW>iWU && iWU>iUV && iUW>iWV && iWV>iVU) return 5; //大小中,中小大 Debug OK
	  else //非正常位置
	  {
         impulseNum = Func_u8FindMaxDataPosition_DirectCompare(aImpulse,6);//找最大值
         return(aImpulseToPhaseTab[impulseNum]); //根据最大值得脉冲号返回相位值   
	  }
   }
}

/*********************************************************************
                         强拉换相
     输入:*psMotor电机结构体指针
**********************************************************************/
void BLDC_ForceChangePhase(S_MOTOR_T *psMotor)  
{
   g_bCommutationOK = 0;    //清换相成功标志  
   if(psMotor->eDir == eCW)
   {
      switch(psMotor->u8Phase)
      {
         case 0: NullPhase();     break;
         case 1: WH_VL_HPwmLON(); break;
	     case 2: UH_WL_HPwmLON(); break;
		 case 3: UH_VL_HPwmLON(); break;
		 case 4: VH_UL_HPwmLON(); break;
		 case 5: WH_UL_HPwmLON(); break;
		 case 6: VH_WL_HPwmLON(); break;
		 default:break;
	  }
   }
   else if(psMotor->eDir == eCCW)
   {
      switch(psMotor->u8Phase)
	  {
         case 0: NullPhase();     break;
         case 1: UH_VL_HPwmLON(); break;
		 case 2: WH_UL_HPwmLON(); break;
		 case 3: WH_VL_HPwmLON(); break;
		 case 4: VH_WL_HPwmLON(); break;
		 case 5: UH_WL_HPwmLON(); break;
		 case 6: VH_UL_HPwmLON(); break;     
		 default:break;
      }
   } 
   psMotor->u8Phase = g_aNextPhase[psMotor->u8Phase];
   eaAdFifoChannelBuf[1] = g_sPhaseAdChannel.eaBemf[g_sMotor.u8Phase-1];       //反电势
   eaAdFifoChannelBuf[2] = g_sPhaseAdChannel.eaConduction[g_sMotor.u8Phase-1]; //导通相电压
   g_u16BlockCnt = 0;
   g_bCommutationOK = 1; //置换相成功标志  
   g_sMotor.uWorkFlag.bfluxCommuSucess = 1; //置低速换相成功标志,用来计算过零点
}

/*********************************************************************
                         磁链换相运算
     输入:*psMotor电机结构体指针,*puFluxAdRes 磁链参数AD值
**********************************************************************/
void BLDC_FluxCalcuCommutation(S_MOTOR_T *psMotor,const U_FLUX_AD_RES_T *puFluxAdres)
{
   u16 s_u16FluxValBuf = 0;
   u16 u16BemfErrAd = 0;    //本次和上次反电势差值AD  
   
   g_bCommutationOK = 0;    //清换相成功标志 
   //psMotor->uWorkFlag.bfluxCommuSucess = 0; //清磁链换相成功标志
   if(puFluxAdres->u16Bemf >= g_u16LastBemfAd)  
   { u16BemfErrAd = puFluxAdres->u16Bemf - g_u16LastBemfAd; } //得出相邻反电势电压的差值
   else
   { u16BemfErrAd = g_u16LastBemfAd - puFluxAdres->u16Bemf; }    
   g_u16LastBemfAd = puFluxAdres->u16Bemf;

   if(u16BemfErrAd<319 && u16BemfErrAd>0) //差值=3V*1.5K/11.5K=0.39 0.39*4096/5=319;分压比11.5K/1.5K=7.67
   {
      //磁链运算时AD结果转为实际值,注意:反电势和导通相AD已在AD中断中转换成12位(放大4倍)         
      uFluxVar.u16Conduction = (puFluxAdres->u16Conduction*13)>>1; //导通相电压*1024=(3.3AD*7.67*1024/4096)
      uFluxVar.u16Bemf = (puFluxAdres->u16Bemf*13)>>1;//反电势*1024        
      uFluxVar.u16PhaseCurr = puFluxAdres->u16PhaseCurr/261; //(AD*3.3/4096)/21/0.01 电流是实时值,放在磁链运算中再放大1024倍  
	     
      switch(psMotor->u8Phase)         
	  {
         case 1:
		 case 2:
		 case 4:
		 	   s_u16FluxValBuf = BLDC_u16FluxFormulaCalcu2(&uFluxVar,g_u16L_dIs_dt);
		 	   break;
		 case 3:
		 case 5:
		 case 6:
		 	   s_u16FluxValBuf = BLDC_u16FluxFormulaCalcu(&uFluxVar,g_u16L_dIs_dt);
		 	   break;   
         default:
		 	   s_u16FluxValBuf = u16FluxNullCalcu(&uFluxVar,g_u16L_dIs_dt);
		 	   break;
	  }
	  
      //UartSendData(s_u16FluxValBuf); //Debug 发送磁链实时值                                     
	  //if(s_u16FluxValBuf >= FLUX_THRESHOLD) //执行时间约2-3us               
	  //if(s_u16FluxValBuf >= 30) //自制4对级电机参数执行时间约2-3us 25上坡短,下坡长;30上长,下短; 50:有抖动上长,下短   
      //if(s_u16FluxValBuf >= 10) //2对级OK
	  //if(s_u16FluxValBuf >= 15) // 2对级电机磁链阈值 
	  if(s_u16FluxValBuf >= 35) // 2对级电机磁链阈值   
	  //if(s_u16FluxValBuf >= 20) // 2对级电机磁链阈值HT电机    
	  //if(s_u16FluxValBuf >= 10)
	  {
         if(psMotor->eDir == eCW)   
		 {
            switch(psMotor->u8Phase)
            {
               case 0: NullPhase();     break;
               case 1: WH_VL_HPwmLON(); break;
			   case 2: UH_WL_HPwmLON(); break;
			   case 3: UH_VL_HPwmLON(); break;
			   case 4: VH_UL_HPwmLON(); break;
			   case 5: WH_UL_HPwmLON(); break;
			   case 6: VH_WL_HPwmLON(); break;  
			   default:break;
			}
         }
		 else if(psMotor->eDir == eCCW)
		 {
            switch(psMotor->u8Phase)
			{
               case 0: NullPhase();     break;
               case 1: UH_VL_HPwmLON(); break;
			   case 2: WH_UL_HPwmLON(); break;
			   case 3: WH_VL_HPwmLON(); break;
			   case 4: VH_WL_HPwmLON(); break;
			   case 5: UH_WL_HPwmLON(); break;
			   case 6: VH_UL_HPwmLON(); break;     
			   default:break;
            }
         }
		 //减少函数调用(Stack) 配置ADC采样通道    
		 psMotor->u8Phase = g_aNextPhase[psMotor->u8Phase];
		 eaAdFifoChannelBuf[1] = g_sPhaseAdChannel.eaBemf[g_sMotor.u8Phase-1];       //反电势
		 eaAdFifoChannelBuf[2] = g_sPhaseAdChannel.eaConduction[g_sMotor.u8Phase-1]; //导通相电压
		 g_u16BlockCnt = 0;
		 g_bCommutationOK = 1; //置换相成功标志
		 psMotor->uWorkFlag.bfluxCommuSucess= 1;  //置磁链换相成功标志
	//	 g_sMotor.u8TimeCntFluxToForce = 0;
      }
   }
}

//无效相位,空计算
u16 u16FluxNullCalcu(const S_FLUX_VAR_T *psFluxVar,uint16_t L_dIs_dt) 
{ return 0; }

//根据导通相电压,非导通相电压,相电流计算磁链换相信号
u16 BLDC_u16FluxFormulaCalcu(const S_FLUX_VAR_T *psFluxVar,u16 L_dIs_dt)
{
   u32 R_mux_Is_mag = 0;          //电机相电阻和电流乘积      
   u32 u16NumeratorLeft = 0;      //分子计算式左半边
   u32 u16NumeratorRight = 0;     //分子计算式右半边
   u32 u16NumeratorVal = 0;       //磁链运算公式分子值
   u32 u16DenominatorLeft = 0;    //分母计算式左半边
   u32 u16DenominatorRight = 0;   //分母计算式右半边
   u32 u16DenominatorVal   = 0;   //磁链运算公式分母值
   u32 u16FluxResult	   = 0;   //磁链运算结果值

   //R_mux_Is_mag = psFluxVar->u16PhaseCurr<<11;  //相电阻*相电流=R(1.7R)*Is(放大1024倍):*2Ω*1024 
   R_mux_Is_mag = psFluxVar->u16PhaseCurr<<10;  //相电阻*相电流=R(1.3R)*Is(放大1024倍):*1.3Ω*1024   
   u16NumeratorLeft = R_mux_Is_mag+L_dIs_dt;
   u16NumeratorRight = psFluxVar->u16Bemf;         //反电势
   u16DenominatorLeft = psFluxVar->u16Bemf+R_mux_Is_mag+L_dIs_dt;
   u16DenominatorRight = psFluxVar->u16Conduction; //导通相电压

   if(u16NumeratorLeft >= u16NumeratorRight)
   { u16NumeratorVal = u16NumeratorLeft-u16NumeratorRight; }
   else
   { u16NumeratorVal = u16NumeratorRight-u16NumeratorLeft; }
   
   if(u16DenominatorLeft >= u16DenominatorRight)
   { u16DenominatorVal = u16DenominatorLeft-u16DenominatorRight; }
   else
   { u16DenominatorVal = u16DenominatorRight-u16DenominatorLeft; }

   if(u16DenominatorVal != 0)
   { u16FluxResult = (u16)(u16NumeratorVal*10/u16DenominatorVal); } //磁链值放大10倍   
   //{ u16FluxResult = (u16)(u16NumeratorVal*100/u16DenominatorVal); } //磁链值放大100倍      
   else
   { u16FluxResult = (u16)u16NumeratorVal*10; }     
   //{ u16FluxResult = (u16)u16NumeratorVal*100; }  
   return(u16FluxResult);
}

u16 BLDC_u16FluxFormulaCalcu2(const S_FLUX_VAR_T * psFluxVar,u16 L_dIs_dt)
{
   u32 R_mux_Is_mag = 0;          //电机相电阻和相电流乘积   
   u32 u16NumeratorLeft    = 0;   //分子计算式左半边
   u32 u16NumeratorRight   = 0;   //分子计算式右半边
   u32 u16NumeratorVal    = 0;    //磁链运算公式分子值
   u32 u16DenominatorLeft  = 0;   //分母计算式左半边
   u32 u16DenominatorRight = 0;   //分母计算式右半边
   u32 u16DenominatorVal   = 0;   //磁链运算公式分母值
   u32 u16FluxResult       = 0;   //磁链运算结果值 

   //R_mux_Is_mag = psFluxVar->u16PhaseCurr<<11;  //相电阻*相电流=R(1.7R)*Is(放大1024倍):*2Ω*1024
   R_mux_Is_mag = psFluxVar->u16PhaseCurr<<10;  //相电阻*相电流=R(1.3R)*Is(放大1024倍):*1.3Ω*1024  
   u16NumeratorLeft    = psFluxVar->u16Conduction; //导通相
   u16NumeratorRight   = psFluxVar->u16Bemf+R_mux_Is_mag+L_dIs_dt; //BEMF+Ris_Ldis/dt
   u16DenominatorLeft  = psFluxVar->u16Bemf;
   u16DenominatorRight = R_mux_Is_mag+L_dIs_dt; //导通相电压

   if(u16NumeratorLeft >= u16NumeratorRight)
   { u16NumeratorVal = u16NumeratorLeft-u16NumeratorRight; }
   else
   { u16NumeratorVal = u16NumeratorRight-u16NumeratorLeft; }

   if(u16DenominatorLeft >= u16DenominatorRight)
   { u16DenominatorVal = u16DenominatorLeft-u16DenominatorRight; }
   else
   { u16DenominatorVal = u16DenominatorRight-u16DenominatorLeft; }   

   if(u16DenominatorVal != 0)
   { u16FluxResult = (u16)(u16NumeratorVal*10/u16DenominatorVal);} //磁链值放大10倍
   //{ u16FluxResult = (u16)(u16NumeratorVal*100/u16DenominatorVal);} //磁链值放大100倍
   else
   { u16FluxResult = (u16)u16NumeratorVal*10; }        
   //{ u16FluxResult = (u16)u16NumeratorVal*100; }
   return(u16FluxResult);
}

/*-----------------------------------------------------------------------
  反电势过零检测方法一
  phase:当前相
  正转:6 5 3上升; 4 1 2下降
------------------------------------------------------------------------*/
u8 BLDC_u8BemfZeroCrossDetect_LowDuty(S_MOTOR_T * psMotor,uint16_t u16BusVdcHalfAd,uint16_t u16BemfAd)
{
   u8  u8ZeroCrossFlag = 0;   // 0:无过零信号;1:有过零信号
   u16 u16BemfHighLimit = 0;  //反电势上限值
   u16 u16BemfLowLimit = 0;   //反电势下限值    
   u16 u16MidVol = 0;         //中点电压
   //static u8 RiseZeroCnt = 0; //上坡过零计数
   //static u8 FallZeroCnt = 0; //下坡过零计数

   u16BemfHighLimit = u16BusVdcHalfAd*13>>3; //设置反电势比较区间母线电压的10%-81.25%(PWM OFF时运算)          
   u16BemfLowLimit = u16BusVdcHalfAd/5; 
   u16MidVol = u16BusVdcHalfAd;    
   //u16MidVol = u16BusVdcHalfAd*7/11;//母线电压32%
   //u16MidVol = u16BusVdcHalfAd*6/5;//母线电压60%    
   //u16BemfHighLimit = (u16BusVdcHalfAd*11)>>3; //设置反电势比较区间母线电压的31%-69%(PWM OFF时运算)       
   //u16BemfLowLimit = (u16BusVdcHalfAd*5)>>3; 
   
   if(u16BemfAd>=u16BemfLowLimit && u16BemfAd<=u16BemfHighLimit) //反电势在两者之间再去判断
   {
      switch(psMotor->u8Phase)  
	  {
         case 6:  //上升Bemf>Vbus/2   
		 case 5:
		 case 3:
		 	psMotor->uWorkFlag.bBemfFalling = 0; //在上坡阶段,清下坡标志位
		 	if(psMotor->uWorkFlag.bBemfRising == 0) //无反电势上升标志
		 	{
               //if(u16BemfAd < u16BusVdcHalfAd)   //小于母线电压一半置反电势上升标志
               if(u16BemfAd < u16MidVol)      //小于母线电压32置反电势上升标志
               //if(u16BemfAd < u16BusVdcHalfAd*11/10)   //小于母线电压55%置反电势上升标志  
               //if(u16BemfAd < u16BusVdcHalfAd*6/5)   //小于母线电压60%置反电势上升标志   
               //if(u16BemfAd < u16BusVdcHalfAd*59/50)   //小于母线电压58%置反电势上升标志    
			   { psMotor->uWorkFlag.bBemfRising = 1; }         
			}
			else  //有反电势上升标志     
			{
               //if(u16BemfAd > u16BusVdcHalfAd)   
			   if(u16BemfAd > u16MidVol)
			   //if(u16BemfAd >= u16BusVdcHalfAd*11/10) //大于母线电压55%           
			   //if(u16BemfAd >= u16BusVdcHalfAd*6/5) //大于母线电压60% 
			   //if(u16BemfAd >= u16BusVdcHalfAd*59/50)   //大于母线电压58%置反电势上升标志 
			   {
                  u8ZeroCrossFlag = 1;//检测到了上坡过零,置标志位
                  psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                  psMotor->uWorkFlag.bBemfRising = 0;  
			   }
			}
		 	break;
		 case 4: //下降Bemf>Vbus/2
		 case 1:
		 case 2:
		 	psMotor->uWorkFlag.bBemfRising = 0; //在下坡阶段,清上坡标志位
		 	if(psMotor->uWorkFlag.bBemfFalling == 0) //无反电势下降标志  
		 	{
               if(u16BemfAd > u16MidVol)    
			   { psMotor->uWorkFlag.bBemfFalling = 1; }
			}
			else  //有反电势下坡标志
			{
               if(u16BemfAd <= u16MidVol)  
			   {
                  u8ZeroCrossFlag = 1;//检测到了下坡过零,置标志位
                  psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                  psMotor->uWorkFlag.bBemfFalling = 0;
			   }
			}
		 	break;
        default:break;
	  }
   }
   return (u8ZeroCrossFlag);
}

/*-----------------------------------------------------------------------
  反电势过零检测方法二(<50%占空比)
  phase:当前相
  正转:6 5 3上升; 4 1 2下降   
------------------------------------------------------------------------*/
u8 BLDC_u8BemfZeroCrossDetect_MidDuty(S_MOTOR_T *psMotor,u16 BusVdcHalfAd,u16 BemfAd)
{
   u8  ZeroCrossFlag = 0;  //0:无过零信号;1:有过零信号
   u16 BemfHighLimit = 0;  //反电势上限值
   u16 BemfLowLimit  = 0;  //反电势下限值
   u16 u16MidVot = 0;      //修正后的过零点电压
   static u8 s_u8ZeroCrossRisngCnt = 0;   //反电势过零点次数      
   static u8 s_u8ZeroCrossFallingCnt = 0;
//设置反电势比较区间母线电压的10%-81%(PWM OFF时运算)   
   BemfLowLimit = BusVdcHalfAd/5;      //下限值=母线电压/2/5=母线电压*10%         
   BemfHighLimit = BusVdcHalfAd*13>>3; //上限值=母线电压/2*13/8=母线电压*81%    

   if(BemfAd>=BemfLowLimit && BemfAd<=BemfHighLimit) //反电势在两者之间再去判断  
   {
      switch(psMotor->u8Phase)
	  {
         case 6:  //上升Bemf>Vbus/2          
		 case 5:
		 case 3:
		 	  //爬坡时未检测到反电势上升:Vbemf<Vdc/2置上升;Vbemf>Vdc/2且电流<8A,Vbemf<0.69Vdc(连续2点)时置过零标志        
		 	  psMotor->uWorkFlag.bBemfFalling = 0;    //在上坡阶段,清下坡标志位
		 	 // u16MidVot = BusVdcHalfAd*6/5;           //上坡过零前移:母线电压60%
		 	  //u16MidVot = BusVdcHalfAd*11/10;         //上坡过零前移:母线电压55%    
		 	  u16MidVot = BusVdcHalfAd;             //上坡过零前移:母线电压50% 
		 	  if(psMotor->uWorkFlag.bBemfRising == 0) //无反电势上升标志    
			  {
                 //if(BemfAd < BusVdcHalfAd)   //<Vdc/2置反电势上升标志
                 if(BemfAd < u16MidVot)     //<过零点电压  
                 {
                    s_u8ZeroCrossRisngCnt = 0;
				    psMotor->uWorkFlag.bBemfRising = 1;        
				 }   
				 #if 1  
				 else       
				 {
                    if(g_sPeakIdc.Ins <= 325)  //5A(2倍额定电流)(3.3V*325/1024/21/0.01)
                    {
                       if(BemfAd < BusVdcHalfAd*11/8) //<母线电压的69%
                   // if(BemfAd < BusVdcHalfAd) //<母线电压的69%  
                       {
                          if(++s_u8ZeroCrossRisngCnt >= 2) //启动扭矩存在上升沿无上升 增加检测次数
						  {
                             s_u8ZeroCrossRisngCnt = 0;
                             ZeroCrossFlag = 1; //检测到了上坡过零,置标志位  
                             psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到过零,置标志位
                             psMotor->uWorkFlag.bBemfRising = 0;
							 g_u16BlockCnt = 0; 
						  }     
					   }  
					}
				 }
				 #endif  
			  }
			  else //爬坡时已检测到反电势上升:电流>8A,Vbemf>Vdc*0.6时过零;电流<8A,Vbemf>Vdc*0.5时过零      
			  {
                 if(g_sPeakIdc.Ins < 325)   //<5A
				 {
                    if(BemfAd > u16MidVot)  //bemf>Vdc/2             
                    {
                       ZeroCrossFlag = 1; //检测到了上坡过零,置标志位
                       psMotor->uWorkFlag.bGetBemfZeroCross = 1; //置过零标志(超前换相用)
                       psMotor->uWorkFlag.bBemfRising = 0;
					   g_u16BlockCnt = 0;     
					}
                 }  
                 else    
                 {
                    if(BemfAd > BusVdcHalfAd*12/10)  //过零点后移:>母线电压的60%  
                //   if(BemfAd > BusVdcHalfAd)  //过零点后移:>母线电压的60%  
                    {
                       ZeroCrossFlag = 1; //检测到了上坡过零,置标志位   
                       psMotor->uWorkFlag.bGetBemfZeroCross = 1; //置过零标志(超前换相用)
                       psMotor->uWorkFlag.bBemfRising = 0;
					   g_u16BlockCnt = 0; //成功换相,清堵转计数器
					}
                 }
			  }
		 	  break;
		 case 4: //下降Bemf>Bus/2
		 case 1:
		 case 2:
		 	  u16MidVot = BusVdcHalfAd;         //下坡过零前移:母线电压50%     
		      //u16MidVot = BusVdcHalfAd*11/10;    //下坡过零上移:母线电压55%                
		      //u16MidVot = BusVdcHalfAd*6/5;         //下坡过零前移:母线电压60%  
		      //u16MidVot = BusVdcHalfAd*13/10;         //下坡过零前移:母线电压65%  
		      //u16MidVot = BusVdcHalfAd*7/5;         //下坡过零前移:母线电压70%     
		      psMotor->uWorkFlag.bBemfRising = 0;      //在下坡阶段,清上坡标志位  
		 	  if(psMotor->uWorkFlag.bBemfFalling == 0) //无反电势下降标志
		 	  {
                 //if(BemfAd > BusVdcHalfAd)
				 if(BemfAd > u16MidVot)        //下坡过零点前移  
                 {
                    if(g_sPeakIdc.Ins <= 325)  //<5A        
                    {
                       s_u8ZeroCrossFallingCnt = 0;
                       psMotor->uWorkFlag.bBemfFalling = 1; 
					}
					else
					{
                       if(BemfAd > BusVdcHalfAd*9/10) //电流>8A:大于母线电压45%,置反电势下坡标志
					   {
                          s_u8ZeroCrossFallingCnt = 0;
                          psMotor->uWorkFlag.bBemfFalling = 1;
					   }
					}
                 }
				 else  //无反电势下坡标且反电势电压在过零点以下 直接判断过零点
				 {
                    if(g_sPeakIdc.Ins <= 325)  // Demo里是8A(5V*400/1024/32/0.002)
                    {
                       if(BemfAd < BusVdcHalfAd*7/11)  //<母线电压的32%
                       {
                          if(++s_u8ZeroCrossFallingCnt >= 2) //启动扭矩测试发现下降沿无下降变化增加次数
                          {
                             s_u8ZeroCrossFallingCnt = 0;
							 ZeroCrossFlag = 1; //检测到下坡过零,置标志位
							 psMotor->uWorkFlag.bGetBemfZeroCross = 1; //置过零标志
							 psMotor->uWorkFlag.bBemfFalling = 0;
                             g_u16BlockCnt = 0; 
						  }
					   }
                    }
				 }
			  }
			  else //有反电势下坡标志
			  {
                 if(g_sPeakIdc.Ins >= 325) //>5A    
                 {
                    if(BemfAd <= BusVdcHalfAd*7/10) //<母线电压的35%
                    {
                       ZeroCrossFlag = 1; //检测到下坡过零,置标志位
                       psMotor->uWorkFlag.bGetBemfZeroCross = 1; //置过零标志
                       psMotor->uWorkFlag.bBemfFalling = 0;
					   g_u16BlockCnt = 0;   
					}
				 }
				 else
				 {
                    if(BemfAd < BusVdcHalfAd)   
                    //if(BemfAd < u16MidVot*11/10) //下坡过零前移:母线电压55%   
					{
                       ZeroCrossFlag = 1; //检测到下坡过零,置标志位
                       psMotor->uWorkFlag.bGetBemfZeroCross = 1; //置过零标志   
                       psMotor->uWorkFlag.bBemfFalling = 0;
					   g_u16BlockCnt = 0;  
					}
                 }
			  }
		 	  break;
		 default:break;
      }
   }   
   return (ZeroCrossFlag);  
}

/*-----------------------------------------------------------------------
  反电势过零检测方法一 >50% <80%占空比  
  phase:当前相
  正转:6 5 3上升; 4 1 2下降
------------------------------------------------------------------------*/
u8 BLDC_u8BemfZeroCrossDetect_MidHighDuty(S_MOTOR_T *psMotor,u16 u16BusVdcHalfAd,u16 aAdcRes[ ])
{
   u8  i = 0;
   u8  u8EdgeCnt = 0;        //检测到边沿状态个数(即在过零点前)
   u8  u8ZeroCrossCnt = 0;   //检测到过零次数(即在过零点后)
   u8  u8ZeroCrossFlag = 0;  //0:无过零信号; 1:有过零信号
   u16 u16BemfHighLimit = 0; //反电势上限值
   u16 u16BemfLowLimit = 0;  //反电势下限值
   u16 u16MidVot = 0;        //修正后的过零点电压

   switch(psMotor->u8Phase)  
   {
      case 6: //上升Bemf>Vbus/2  
	  case 5:
	  case 3:
	  	   psMotor->uWorkFlag.bBemfFalling = 0;                 
		   u16BemfLowLimit = u16BusVdcHalfAd/5;    //反电势比较区间下限为母线电压的10%(1/2/5=0.1)     
		   u16MidVot = u16BusVdcHalfAd;            //上坡时过零点前移补偿     
		   //u16MidVot = u16BusVdcHalfAd-(u16BusVdcHalfAd>>2); //上坡时过零点前移补偿1.25倍(1.3V↓0.975V)        
		   if(psMotor->uWorkFlag.bBemfRising == 0) //无反电势上升标志
		   {
              u16BemfHighLimit = (u16BusVdcHalfAd*15)>>3; //反电势比较区间上限调整为母线电压94%
              for(i=2;i<5;i++)
			  {
                 if(aAdcRes[i]>=u16MidVot && aAdcRes[i]<=u16BemfHighLimit) //>=过零点,<上限(上坡后半区)
                 { u8ZeroCrossCnt++; } //过零计数加1
                 else if(aAdcRes[i]<u16MidVot && aAdcRes[i]>=u16BemfLowLimit) //<过零点,>下限(上坡前半区)
                 { u8EdgeCnt++; }      //边沿计数加1  
			  }
			  
			  #if 0   
			  if(u8EdgeCnt >= 1) //有1次以上在上升爬坡区，认为是反电势上升,置标志位
			  { psMotor->uWorkFlag.bBemfRising = 1; }
			  #endif
			  
			  #if 1
			  if(u8ZeroCrossCnt >= 3) //有3次以上过零
			  {
                 u8ZeroCrossFlag = 1; //确认已检测到过零
                 psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                 psMotor->uWorkFlag.bBemfRising = 0;
				 g_u16BlockCnt = 0; //成功换相,清堵转计数器
			  }
			  else if(u8EdgeCnt >= 1) //有1次以上在上升爬坡区，认为是反电势上升,置标志位
			  { psMotor->uWorkFlag.bBemfRising = 1; }
			  #endif
		   }
		   else //已有反电势上升标志
		   {
              u16BemfHighLimit = u16BusVdcHalfAd*15>>3; //反电势比较区间上限为母线电压的94%
              for(i=2;i<5;i++)
			  {
                 if(aAdcRes[i]>=u16MidVot && aAdcRes[i]<u16BemfHighLimit) //>=过零点,<上限(上坡后半区)
                 {
                    u8ZeroCrossFlag = 1; //确认检测到过零
                    psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                    psMotor->uWorkFlag.bBemfRising = 0;
					g_u16BlockCnt = 0; //成功换相,清堵转计数器
					break;
				 }
              }
		   }
	  	   break;
	  case 4:  //下降Bemf>Bus/2
	  case 1:
	  case 2:
	  	   psMotor->uWorkFlag.bBemfRising = 0;
		   u16BemfHighLimit = u16BusVdcHalfAd*15>>3; //设置母线电压上限94%,81%     
		   u16MidVot = u16BusVdcHalfAd;              //下坡时过零点前移补偿                   
		   //u16MidVot = u16BusVdcHalfAd+(u16BusVdcHalfAd>>2); //下坡时过零点前移补偿1.25(1.3V↑1.625V)   
		   //u16MidVot = u16BusVdcHalfAd+(u16BusVdcHalfAd>>1); //下坡时过零点前移补偿1.5(1.3V↑1.95V)  
		   if(psMotor->uWorkFlag.bBemfFalling == 0)  //无反电势下降标志
		   {
              u16BemfLowLimit = u16BusVdcHalfAd/5;   //反电势比较区间下限为母线电压的10%,19%   
              for(i=2;i<5;i++)
              {
                 if(aAdcRes[i]<=u16MidVot && aAdcRes[i]>u16BemfLowLimit) //<过零点,>下限(上坡后半区)
                 { u8ZeroCrossCnt++; }  //过零计数加1
                 else if(aAdcRes[i]>u16MidVot && aAdcRes[i]<u16BemfHighLimit)
				 { u8EdgeCnt++; }       //边沿计数加1    
			  }   
			  
			  #if 1
			  if(u8ZeroCrossCnt >= 3)               
			  {
                 u8ZeroCrossFlag = 1;  //确认已检测到过零
                 psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                 psMotor->uWorkFlag.bBemfFalling = 0;
				 g_u16BlockCnt = 0; //成功换相,清堵转计数器
			  }
			  else if(u8EdgeCnt >=1)  //有2次以上在下降爬坡区
			  { psMotor->uWorkFlag.bBemfFalling = 1; } //认为是反电势下降,置标志位
			  #endif
		   }              
		   else //已有反电势下降标志
		   {
              u16BemfLowLimit = u16BusVdcHalfAd/5; //反电势比较区间的下限为母线电压的12.5%
              for(i=2;i<5;i++)
			  {
                 if(aAdcRes[i]<=u16MidVot && aAdcRes[i]>=u16BemfLowLimit) //<=过零点,>下限(下坡后半区)
                 {
                    u8ZeroCrossFlag = 1; //确认已检测到过零
                    psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                    psMotor->uWorkFlag.bBemfFalling = 0;
				    g_u16BlockCnt = 0; //成功换相,清堵转计数器     
				    break;
				 }
			  }
		   }
           break;
	  default:break;
   }
   return(u8ZeroCrossFlag);
}

/*-----------------------------------------------------------------------
  反电势过零检测方法一 >80%占空比  
  phase:当前相
  正转:6 5 3上升; 4 1 2下降
------------------------------------------------------------------------*/

u8 BLDC_u8BemfZeroCrossDetect_HighDuty(S_MOTOR_T * psMotor,u16 u16BusVdcHalfAd,u16 aAdcRes[ ])
{
   u8  i = 0;
   u8  u8EdgeCnt = 0;        //检测到边沿状态个数(即在过零点前)
   u8  u8ZeroCrossCnt = 0;   //检测到过零次数(即在过零点后)
   u8  u8ZeroCrossFlag = 0;  // 0:无过零信号; 1:有过零信号
   u16 u16BemfHighLimit = 0; //反电势上线值
   u16 u16BemfLowLimit  = 0; //反电势下限值 
   u16 u16MidVot = 0;        //修正后的过零点电压               

   switch(psMotor->u8Phase)
   {
      case 6:  //上升Bemf>Vbus/2
	  case 5:
	  case 3:
	  	   psMotor->uWorkFlag.bBemfFalling = 0;  
		   u16BemfLowLimit = (u16BusVdcHalfAd*1)>>3;         //反电势比较区间下限为母线电压的6.25%
		   u16MidVot = u16BusVdcHalfAd;              //上坡时过零点前移补偿  
		   //u16MidVot = u16BusVdcHalfAd-(u16BusVdcHalfAd>>2); //上坡时过零点前移补偿(6V↓4.5V)   
		   //u16MidVot = u16BusVdcHalfAd-(u16BusVdcHalfAd>>1); //下坡时过零点前移补偿1.5倍(1.3V↑1.95V)             
		   if(psMotor->uWorkFlag.bBemfRising == 0) //无反电势上升标志
		   {
              u16BemfHighLimit = u16BusVdcHalfAd*15>>3; //反电势比较区间上限调整为母线电压94%  
              for(i=2;i<7;i++)
			  {
                 if(aAdcRes[i]>=u16MidVot && aAdcRes[i]<=u16BemfHighLimit) //>=过零点,<上限(上坡后半区)
                 { u8ZeroCrossCnt++; } //过零计数加1
                 else if(aAdcRes[i]<u16MidVot && aAdcRes[i]>=u16BemfLowLimit)//<过零点,>上限(上坡前半区)
                 { u8EdgeCnt++; }      //边沿计数加1
			  }
			  if(u8ZeroCrossCnt >= 3) //有3次以上过零
			  {
                 u8ZeroCrossFlag = 1; //确认已检测到过零
                 psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                 psMotor->uWorkFlag.bBemfRising = 0;
				 g_u16BlockCnt = 0; //成功换相,清堵转计数器
              }
			  else if(u8EdgeCnt >= 1) //有1次以上在上升爬坡区
			  { psMotor->uWorkFlag.bBemfRising = 1; } //认为是反电势上升,置标志位
		   }
		   else //已有反电势上升标志  
		   {
              u16BemfHighLimit = u16BusVdcHalfAd*15>>3; //反电势比较区间上限为母线电压94% 
              for(i=2;i<7;i++)
			  {
                 if(aAdcRes[i]>=u16MidVot && aAdcRes[i]<u16BemfHighLimit) //>=过零点,<上限(上坡后半区)
                 {
                    u8ZeroCrossFlag = 1; //确认已检测到过零
                    psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                    psMotor->uWorkFlag.bBemfRising = 0;
				    g_u16BlockCnt = 0; //成功换相,清堵转计数器
				 }
			  }
		   }
	  	   break;
		   
	  case 4:
	  case 1:
	  case 2: //下降Bemf>Bus/2:分压比1.5K/10K->20*(1.5K/11.5K)=2.6V->1.3V                
	  	   psMotor->uWorkFlag.bBemfRising = 0;
		   u16BemfHighLimit = u16BusVdcHalfAd*15>>3;//设置母线电压上限的94%  
		   u16MidVot = u16BusVdcHalfAd;              //下坡时过零点前移补偿  
		   //u16MidVot = u16BusVdcHalfAd+(u16BusVdcHalfAd>>2); //下坡时过零点前移补偿1.25倍(1.3V↑1.625V)  
		   //u16MidVot = u16BusVdcHalfAd+(u16BusVdcHalfAd>>1); //下坡时过零点前移补偿1.5倍(1.3V↑1.95V)   
		   if(psMotor->uWorkFlag.bBemfFalling == 0) //无反电势下降标志
		   {
              u16BemfLowLimit = (u16BusVdcHalfAd*1)>>3;//反电势比较区间下限为母线电压的6.25%  
              for(i=2;i<7;i++)
			  {
                 if(aAdcRes[i]<=u16MidVot && aAdcRes[i]>u16BemfLowLimit) //<=过零点,>下限(上坡后半区)
                 { u8ZeroCrossCnt++; } //过零计数加1
                 else if(aAdcRes[i]>u16MidVot && aAdcRes[i]<u16BemfHighLimit)
                 { u8EdgeCnt++; }      //边沿计数加1      
              }
              if(u8ZeroCrossCnt >= 3)
			  {
                 u8ZeroCrossFlag = 1; //确认已检测到过零
                 psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                 psMotor->uWorkFlag.bBemfFalling = 0;
				 g_u16BlockCnt = 0; //成功换相,清堵转计数器
              }
			  else if(u8EdgeCnt >= 1) //有2次以上在下降爬坡区
			  { psMotor->uWorkFlag.bBemfFalling = 1; } //认为是反电势下降,置标志位
		   }
		   else //已有反电势下降标志    
		   {
              u16BemfLowLimit = (u16BusVdcHalfAd*1)>>3;//反电势比较区间的下限为母线电压的6.25% 
              for(i=2;i<7;i++)
			  {
                 if(aAdcRes[i]<=u16BusVdcHalfAd && aAdcRes[i]>=u16BemfLowLimit) //<=过零点,>下限(下坡后半区)
                 {
                    u8ZeroCrossFlag = 1; //确认已检测到过零
                    psMotor->uWorkFlag.bGetBemfZeroCross = 1; //检测到了过零,置标志位
                    psMotor->uWorkFlag.bBemfFalling = 0;
				    g_u16BlockCnt = 0; //成功换相,清堵转计数器
				    break;
				 }
			  }
           }
	  	   break;
	  default:break;          
   }
   return (u8ZeroCrossFlag);
}

//判断数组中是否有一个数小于规定阈值
u8 Func_u8JudgeOneDataLessthanThrVal(const s16 a[ ],u8 ln,s16 thrval)
{
   u8 i = 0;
   u8 resFlag = 0;
   for(;i<ln;i++)
   {
      if(a[i] < thrval)
      {  
         resFlag = 1; //有比阈值小的数,置标志故障位
		 break;
	  } 
   }
   
   return resFlag;
}

//用直接比较方式寻找一维数组中的最大值的位置
u8 Func_u8FindMaxDataPosition_DirectCompare(const s16 a[],u8 ln)
{
   s8 i = 0;
   u8 maxDataPosition = 0;

   for(i=1;i<ln;i++)
   {
      if(a[maxDataPosition] < a[i]) 
	  { maxDataPosition = i; }
   }
   return maxDataPosition;
}

/*------------------------------------------------------------------------------------------------------------
   根据过零点转换成电机转速
D48电机:   
   1.通道0用来计算过零点时间:1个clock为24M/2^4=1.5M=2/3us   
   2.三拍时间=2/3*(Zt[0]+Zt[1]+Zt[2])us
   3.1圈(12拍,2对极电机)=8/3*(Zt[0]+Zt[1]+Zt[2])us
   4.转速=60S/[8/3*(Zt[0]+Zt[1]+Zt[2])us]=(60*10^6)/[8/3*(Zt[0]+Zt[1]+Zt[2])]=(225*10^5)/(Zt[0]+Zt[1]+Zt[2])
自制电机4对级:
   1.通道0用来计算过零点时间:1个clock为24M/2^4=1.5M=2/3us 
   2.三拍时间=2/3*(Zt[0]+Zt[1]+Zt[2])us
   3.1圈(24拍,4对极电机)=16/3*(Zt[0]+Zt[1]+Zt[2])us
   4.转速=60S/[16/3*(Zt[0]+Zt[1]+Zt[2])us]=(60*10^6)/[16/3*(Zt[0]+Zt[1]+Zt[2])]=(1125*10^4)/(Zt[0]+Zt[1]+Zt[2]) 
联谊电机2对级: 
   1.通道0用来计算过零点时间:1个clock为24M/2^4=1.5M=2/3us 
   2.三拍时间=2/3*(Zt[0]+Zt[1]+Zt[2])us
   3.1圈(12拍,2对极电机)=8/3*(Zt[0]+Zt[1]+Zt[2])us
   4.转速=60S/[8/3*(Zt[0]+Zt[1]+Zt[2])us]=(60*10^6)/[8/3*(Zt[0]+Zt[1]+Zt[2])]=(2250*10^4)/(Zt[0]+Zt[1]+Zt[2]) 
-------------------------------------------------------------------------------------------------------------*/
u8 BLDC_u8ZeroCrossTimeToSpeed(S_MOTOR_T *psMotor,u16 aZeroCrossTime[])    
{
   u16 u16SpeedBuf = 0;      

   if(aZeroCrossTime[0]!=0 || aZeroCrossTime[1]!=0 || aZeroCrossTime[2]!=0)       
   {
      u16SpeedBuf = (u32)(22500000)/(aZeroCrossTime[0]+aZeroCrossTime[1]+aZeroCrossTime[2]);             
	  psMotor->sSpeedFilt.u32Sum += u16SpeedBuf;    
      psMotor->sSpeedFilt.u8Cnt++;
	  if(psMotor->sSpeedFilt.u8Cnt == 1)  //首次开始累加
      {
         psMotor->sSpeedFilt.u16Max = u16SpeedBuf; //最大值赋值   
         psMotor->sSpeedFilt.u16Min = u16SpeedBuf; //最小值赋值
	  }
	  else
	  {
         if(u16SpeedBuf > psMotor->sSpeedFilt.u16Max)      { psMotor->sSpeedFilt.u16Max = u16SpeedBuf; }
         else if(u16SpeedBuf < psMotor->sSpeedFilt.u16Min) { psMotor->sSpeedFilt.u16Min = u16SpeedBuf; }
	  }
	  if(psMotor->sSpeedFilt.u8Cnt >= 6)   
	  {
         psMotor->u16Speed = (u16)((psMotor->sSpeedFilt.u32Sum-psMotor->sSpeedFilt.u16Max-psMotor->sSpeedFilt.u16Min)>>2);
         psMotor->sSpeedFilt.u8Cnt = 0;
		 psMotor->sSpeedFilt.u32Sum = 0;
		 return 1;  //去极值滤波完成  
	  }
	  else
	  { return 0; } //滤波进行中
   }
   else   
   { return 0; }  
}
uint8_t Hall_Get(void)
{
	/*U:V:W = curhallvalue*/
	#if 0
	volatile uint8_t curhallvalue;	
	if(HALL_U == 1)
		curhallvalue= 0x04;
	else
		curhallvalue= 0; 
	if(HALL_V == 1)
		curhallvalue |= 0x02;
	if(HALL_W == 1)
		curhallvalue |= 0x01;       
	return curhallvalue;
  #endif 
  #if 1
  volatile uint8_t curhallvalue;  
      if(HALL_V == 1)
          curhallvalue= 0x04;
      else
          curhallvalue= 0; 
      if(HALL_U == 1)
          curhallvalue |= 0x02;
      if(HALL_W == 1)
          curhallvalue |= 0x01;       
      return curhallvalue;

  #endif 
}
uint8_t GetMotorPosition(uint8_t hallvalue)
{
	uint8_t i;
	for(i=0;i<6;i++)
	{
		if(HallTable[i]==hallvalue)
			return i;
	}
	return 0xff;
}

void Commutate_Phase(uint8_t Position)
{
	if(MotorDirection == CCW)//6->4->5->1->3->2
	{		
		TRDOER1 = CCW_PWM_OUT[Position];     /*enable relative high bridge PWM pin,*/			
		P1=(P1&0xc0)|CCW_P1_OUT[Position];   /*set high to enabled low bridge pin,  */				
	}
	else if(MotorDirection == CW)   //2->3->1->5->4->6
	{	
		TRDOER1 = CW_PWM_OUT[Position];     /*enable relative high bridge PWM pin,*/			
		P1=(P1&0xc0)|CW_P1_OUT[Position];   /*set high to enabled low bridge pin,  */	 				
	}
}

void Self_Commutate_Phase(uint8_t Position)
{	
	TRDOER1 = SELF_CW_PWM_OUT[Position];     /*enable relative high bridge PWM pin,*/			
	P1=(P1&0xc0)|SELF_CW_P1_OUT[Position];   /*set high to enabled low bridge pin,  */	
}

void Motor_Stop(void)
{	
	TRDOER1 = 0xFF;         /*disable all high bridge PWM pin,*/
	P1 &= 0xc0;             /*set low to all high bridge PWM  pin and all low bridge disabled pin,*/
}
void Motor_Brake(void)
{	
	TRDOER1 = 0xff;   //disable all high bridge PWM pin,
	P1&= 0xc0;        //set low to all high bridge pin,		
	P1|=(AL_IO|BL_IO|CL_IO);   //set all low bridge pin high to brake,  
}

void Cap_PreCharge(uint8_t Position)
{
	TRDOER1 = 0XFF;  //禁止输出
	if(MotorDirection == CW)
	{
		P1|=  CW_P1_OUT[Position];
	}
	else
	{
		P1|= CCW_P1_OUT[Position];
	}
}



uint8_t Expect_Hall_Value(uint8_t Position)
{
	uint8_t NextHallValue;
	if(MotorDirection == CCW)
	{
		NextHallValue=HallTable[(Position+5)%6];
	}
	else if(MotorDirection == CW)
	{
		NextHallValue=HallTable[(Position+1)%6];
	}
	return NextHallValue;
}

uint8_t Read_Hall_Port(void)       //92P
{
   uint8_t hall_value = 0;

#if 1
   if(HALL_U) SETBIT(hall_value,0);
   if(HALL_V) SETBIT(hall_value,1);
   if(HALL_W) SETBIT(hall_value,2);
#endif

#if 0
   if(RB3) SETBIT(hall_value,0);
   if(RB2) SETBIT(hall_value,1);
   if(RB1) SETBIT(hall_value,2);
#endif

   return(hall_value);

}
#if 0
void HallISR(void)
{	
	/* Hall value judge */
	uint8_t cnt;
	//P3_bit.no1 = ~P3_bit.no1;    //Toggle Pin
	if(MotorRunningFlag)
	{
	    
		hall_port_value = Read_Hall_Port();   //92P
		
	    Normal_Change_Phase() ;               //92P		
	    /****************计算换相间隔时间***************/         
          g_u16NowZeroTimerVal = TCR00; //读取本次过零点时刻值       
          g_aTzc[2] = g_aTzc[1];
          g_aTzc[1] = g_aTzc[0];
          if(g_u16LastZeroTimerVal >= g_u16NowZeroTimerVal) //上次过零时刻点>本次过零时刻点  
          { g_aTzc[0] = g_u16LastZeroTimerVal - g_u16NowZeroTimerVal; }  
          else 
          { g_aTzc[0] = _FFFF_TAU_TDR00_VALUE-g_u16NowZeroTimerVal+g_u16LastZeroTimerVal; }
          g_u16LastZeroTimerVal = g_u16NowZeroTimerVal; //存贮本次过零点时刻值(作为上次过零时刻值) 
         /**************************************************/
	}	
}
#endif

void HallISR(void)
{	
	/* Hall value judge */
	uint8_t cnt;
	//P3_bit.no1 = ~P3_bit.no1;    //Toggle Pin
//	if(MotorRunningFlag)
	if(g_sMotor.eAct == eRUN_ACT)
	{
		uHallValue = Hall_Get();
		MotorPosition=GetMotorPosition(uHallValue);							
		Commutate_Phase(MotorPosition);	
		//LampFlash;
		LampFlash();
		block_cnt=0;   //进来清堵转计数
	    /****************计算换相间隔时间***************/         
          g_u16NowZeroTimerVal = TCR00; //读取本次过零点时刻值       
          g_aTzc[2] = g_aTzc[1];
          g_aTzc[1] = g_aTzc[0];
          if(g_u16LastZeroTimerVal >= g_u16NowZeroTimerVal) //上次过零时刻点>本次过零时刻点  
          { g_aTzc[0] = g_u16LastZeroTimerVal - g_u16NowZeroTimerVal; }  
          else 
          { g_aTzc[0] = _FFFF_TAU_TDR00_VALUE-g_u16NowZeroTimerVal+g_u16LastZeroTimerVal; }
          g_u16LastZeroTimerVal = g_u16NowZeroTimerVal; //存贮本次过零点时刻值(作为上次过零时刻值) 
        /**************************************************/
		if(uHallValue != uHallValueExpect)
		{				
			/*if(++uCountHallFail >= 50)
			{
				MotorRunningFlag = FALSE;
				MotorStatus = MOTOR_BRAKE;				
			}*/	
			for(cnt=0;cnt<150;cnt++)
			{	
				HallAry[cnt]=uHallValue;
				DelayXus(4);		
				uHallValue = Hall_Get();	
				//P3_bit.no1 = ~P3_bit.no1; 
			}
		//	Motor_Stop();				
 			MotorRunningFlag = FALSE;
		}
		else
		{
			uTimeCntBlock = 0; 					
									
		}							
		uHallValueExpect = Expect_Hall_Value(MotorPosition);
	}	
}

void Normal_Change_Phase(void)
{
   if(g_sMotor.eDir == CCW)       
   {
      switch(hall_port_value)    //U相和W相颠倒 
      {
         case 6: UH_WL; break;  
		 case 4: UH_VL; break;   
		 case 5: WH_VL; break;
		 case 1: WH_UL; break;
		 case 3: VH_UL; break;
		 case 2: VH_WL; break;
		 default:       break;       
	  }
	 
   }
   
   else if(g_sMotor.eDir == CW)       //反转表格           
   {
      switch(hall_port_value)
      {
         case 6: WH_UL; break;    //U相和W相颠倒      
		 case 4: VH_UL; break;  
		 case 5: VH_WL; break;
		 case 1: UH_WL; break;
		 case 3: UH_VL; break;
		 case 2: WH_VL; break;         
		 default:       break;  
	  }
   }
}


/* Start user code for adding. Do not edit comment generated here */


/**
  * @brief  Pwm update function, update pwm duty to set value.
  * @param  pwmvalue:set value.
  * @retval None
*/
void Pwm_Update(void)
{
	uint16_t iPwmTemp;
   	if(CurrentDuty+PWM_DELTAP <TargetDuty)  //  125+270<2158
	{			
		CurrentDuty += PWM_DELTAP;		//CurrentDuty=125+270=395
	}
	else if(CurrentDuty > (TargetDuty+PWM_DELTAN))
	{
		CurrentDuty -= PWM_DELTAN;		
	}
	else
	{
		CurrentDuty = TargetDuty;
	}	
	if(CurrentDuty >= PWM_MAX)   //PWM_MAX=2698
	{
		iPwmTemp = PWM_MAX;
	}
	else if(CurrentDuty == 0)
	{
		iPwmTemp = PWM_ZERO;   //PWM_ZERO=0
	}
	else 
	{
		iPwmTemp = CurrentDuty;
	}
	TRDGRD0 = PWM_PERIOD-iPwmTemp;   //PEM_PERIOD=4000
	TRDGRC1 = PWM_PERIOD-iPwmTemp;
	TRDGRD1 = PWM_PERIOD-iPwmTemp;	
	//TRDGRD0 = 239;   //PEM_PERIOD=4000
	//TRDGRC1 = 239;
	//TRDGRD1 = 239;	
}

//更新占空比
void PWM_DutyUpdata(u16 duty) //4799    
{
   if(duty > DUTY_FULL)       duty = DUTY_FULL;
   else if(duty < DUTY_MIN)   duty = DUTY_MIN;   // 10%最小占空比  

   if(duty > TRDGRA0)  { duty = TRDGRA0; } 
   else                { duty = TRDGRA0-duty; }                 
     
   TRDMR &= 0x11;   //D1 C1 D0作为通用寄存器; C0作为A0缓冲器       
   TRDGRD0 = duty;
   TRDGRC1 = duty;
   TRDGRD1 = duty;
   TRDMR |= 0xe0;  //D1作为B1(UH)缓冲器; C1作为A1(VH)缓冲器; D0作为B0(WH)缓冲器; 
}

//斜坡函数
 uint16_t PwmDutyAdjStepCtrl(uint16_t targetAdjPwmDuty, uint16_t currAdjPwmDuty)
{  
    if((currAdjPwmDuty + 20) <= targetAdjPwmDuty)    
    { 
        currAdjPwmDuty += 20;
    }
    else if(currAdjPwmDuty > (targetAdjPwmDuty + 20))
    {
        currAdjPwmDuty -= 20;
    }
    else 
    { 
        currAdjPwmDuty = targetAdjPwmDuty; 
    }

    //判断最大、最小值
    if(currAdjPwmDuty > 4799)  //100占空比  最大占空比
    {
        currAdjPwmDuty = 4799;
    }
    else if(currAdjPwmDuty <719) //15占空比 最小占空比 443 475
    {
        currAdjPwmDuty = 719;
    }
    return currAdjPwmDuty;
}

void DelayXus(uint16_t us)
{
	while(us--)
	{
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();	
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
	}
}







