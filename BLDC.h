
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































