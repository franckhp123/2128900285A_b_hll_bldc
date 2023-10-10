
#ifndef __PublicWare_H
#define __PublicWare_H

#include "macrodriver.h"
#include "userdefine.h"
#include "adc.h"

//事件触发标志联合体
typedef union
{
   u16 all;
   struct
   {
      u16 bDirFlash           :1; // 00 闪烁显示标志 
      u16 bFirInitCurBuf      :1; // 01 首次初始化电流缓存标志
	  u16 breserved2	      :1; // 02 保留
	  u16 breserved3          :1; // 03 保留
	  u16 bMosCheckFlag	      :1; // 04 mos允许上电自检
	  u16 breserved5          :1; // 05 保留
	  u16 breserved6	      :1; // 06 保留
	  u16 breserved7          :1; // 07 保留
	  u16 breserved8	      :1; // 08 保留
	  u16 breserved9          :1; // 09 保留
	  u16 bStopInterINTFlag	  :1; // 10 故障或刹车后即进入中断,不执行中断换相函数
	  u16 breserved11         :1; // 11 保留
	  u16 breserved12	      :1; // 12 保留
	  u16 bMosHTmpClr	      :1; // 13 MOS高温故障解除标志
	  u16 bBusIdcOcClr	      :1; // 14 母线电流过流故障解除标志
	  u16 bBusVdcErrClr	      :1; // 15 母线电压低故障解除标志
   }Bits;
}U_EVENT_TRIG_FLAG_T;  

//事件触发标志联合体
typedef union
{
   u16 all;
   struct
   {
      u16 bMosOverTmp     :1; // 00 MOS过温故障      
      u16 bMosTempUnder   :1; // 01 MOS低温故障 
      u16 bBatTempOver    :1; // 02 电池包高温故障
	  u16 bBatTempUnder   :1; // 03 电池包低温故障
	  u16 bStart          :1; // 04 预驱动故障
	  u16 bHall           :1; // 05 HALL故障
	  u16 bIdcLimit       :1; // 06 限流超时
	  u16 bAvgIdcShort    :1; // 07 均值电流短路
	  u16 bAvgIdcOver     :1; // 08 均值电流过流
	  u16 bShortCurrent	  :1; // 09 峰值电流短路
	  u16 bPeakIdcOver    :1; // 10 峰值电流过流
	  u16 bBusVdcOver	  :1; // 11 直流母线过压
	  u16 bBusVdcUnder    :1; // 12 直流母线欠压
	  u16 bMotorBlock	  :1; // 13 电机堵转
	  u16 bPosition	      :1; // 14 定位失败故障  
	  u16 bObstacles	  :1; // 15 障碍物故障   
   }Bits;
}U_ERROR_FLAG_T;  

#if 0
typedef struct
{
   u16 bMosOverTmp     :1; // 00 MOS过温故障      
   u16 bMosTempUnder   :1; // 01 MOS低温故障 
   u16 bBatTempOver    :1; // 02 电池包高温故障
   u16 bBatTempUnder   :1; // 03 电池包低温故障
   u16 bStart          :1; // 04 预驱动故障
   u16 bHall           :1; // 05 HALL故障
   u16 bIdcLimit       :1; // 06 限流超时
   u16 bAvgIdcShort    :1; // 07 均值电流短路
   u16 bAvgIdcOver     :1; // 08 均值电流过流
   u16 bShortCurrent   :1; // 09 峰值电流短路
   u16 bPeakIdcOver    :1; // 10 峰值电流过流
   u16 bBusVdcOver	   :1; // 11 直流母线过压
   u16 bBusVdcUnder    :1; // 12 直流母线欠压
   u16 bMotorBlock	   :1; // 13 电机堵转
   u16 bPosition	   :1; // 14 定位失败故障    
   u16 bAccelOver	   :1; // 15 
}U_ERROR_T;
#endif

typedef enum 
{
   cSTA_SELFCHK , //自检状态
   cSTA_WAIT    , //待机全关状态   
   cSTA_RUN     , //运行状态
   cSTA_STOP    , //停机刹车状态
   cSTA_ERR       //故障状态
}E_SYS_STA_T;


//AD结果结构体类型
typedef struct
{
   u8   cnt;       //滤波次数
   u16  Result;    //二级滤波结果      
   u16  FiltRes1;  //一级滤波结果
   u16  Ins;       //瞬时值 
   u8   RealValue; //实际值(模拟量电压温度等)
   u16  sum;       //累加和
}S_FILT_DATA_T; //滤波数据

typedef union
{
   u16 all[4];
   struct
   {
      u16 u16Common;      // 0 公共通道
      u16 u16Conduction;  // 1 导通相电压
      u16 u16Bemf;        // 2 反电势相电压
      u16 u16PhaseCurr;   // 3 相电流(母线PeakIdc瞬时值)
   };
}U_FLUX_AD_RES_T; //磁链运算时AD值结果结构体(用于AD值转实际值,参与磁链运算)

typedef union
{
   u16 aAllData[8];
   struct
   {
      u16 u16Common;      // 0 公共通道 
      u16 u16Conduction;  // 1 导通相电压
      u16 aBemf[5];       // 2~6反电势相电压
      u16 u16PhaseCurr;   // 7 相电流(母线PeakIdc瞬时值)
   };
}U_BEMF_AD_RES_T; //反电势运算时AD值结果结构体(参与反电势过零检测比较)


typedef struct
{
  s32  K1;       // K1 = Kp * (1 + Ki)
  s32  K2;       // K2 = Kp
  s32  lastErr;  // 上一次偏差值 
  s32  out;      // 上一次输出值
  s32  outMin;   // 最小限幅值
  s32  outMax;   // 最大限幅值
  u8   timePIUpGiveSpeed;   //PI环给定速度时间
  u16  PIPremeChgtime;      //PI参数调节时间 
  u8   u8RefSpeedTimeValue; //PI换给定速度改变时基
  u16  u16PILowSpeedTimeVal;   //启动时PI低速运行时间
}S_INC_PI_PARA_T; // 增量式PI参数 

//位置型pid
typedef struct
{
   float Kp;        //比例系数Proportional
   float Ki;        //积分系数Integral
   float Kd;        //微分系数Derivative
   float Ek;        //当前误差
   float Ek1;       //前一次误差 e(k-1)
   float Ek2;       //再前一次误差 e(k-2)
   float LocSum;    //累计积分位置
}PID_LocTypeDef;


//避障检测结构体   
typedef enum   //坡度枚举
{
   eFlat,      //平地
   eUpSlope1,  //上坡度1
   eUpSlope2,  //上坡度2
   eUpSlope3,  //上坡度3
   eDnSlope1,  //下坡度1
   eDnSlope2,  //下坡度2
   eDnSlope3  //下坡度3   
}E_GRADE_T;  

typedef struct
{
   u8         u8SurgeCurTime;        //涌流时间
   u16        u16SumOneLevelCur;     //一级10个电流总和
   u16        u16Sum32OneLevelCur;   //一级10个电流总和
   u16        sc_value[5];           //斜率计数器值
   u16        u16AvrCurOneLevel[4];  //一级电流缓存    
   u16        u16SumCurTwoAvr;       //二级电流总和平均值 
   u8         u8SampOneLevelCurCnt;  //电流采样计数器
   u16        u16SRTheshod;          //斜率阈值
   u16        u16AvrCurTwoLevel[5];  //二级电流缓存  
   s16        Grade;                 //机器坡度角
   E_GRADE_T  eGrade;                //坡度枚举           
   //u8   ObstaclErrTxCnt;       //碰撞故障发送次数
}S_OBSTACL_T;


//宏定义
#if 0
/* D48 Speed */
#define SPEED_MAX        32767 // 速度最大值(RPM)
#define SPEED_MIN        255   // 速度最小值(RPM)
#define SPEED_RATED      10000 // 额定转速(RPM)
#define SPEED_UP_STEP    400   // 转速增加步进值=额定转速*0.02
#define SPEED_DOWN_STEP  400   // 转速减小步进值=额定转速*0.02
#endif

#define SPEED_MAX        4000  // 速度最大值(RPM)   
#define SPEED_MIN        100   // 速度最小值(RPM)
#define SPEED_RATED      3500  // 额定转速(RPM)
//#define SPEED_UP_STEP    76    // 转速增加步进值=额定转速*0.02
//#define SPEED_DOWN_STEP  76    // 转速减小步进值=额定转速*0.02
#define SPEED_UP_STEP    175    // 转速增加步进值=额定转速*0.05         
#define SPEED_DOWN_STEP  175    // 转速减小步进值=额定转速*0.05   
//#define SPEED_UP_STEP    350    // 转速增加步进值=额定转速*0.1         
//#define SPEED_DOWN_STEP  350    // 转速减小步进值=额定转速*0.1  
//#define SPEED_UP_STEP    500    // 转速增加步进值=额定转速*
//#define SPEED_DOWN_STEP  500    // 转速减小步进值=额定转速*  



//函数声明
void Delay_us(u16 cnt);
void Delay_ms(u16 cnt);
u8 Data_Weight_Filter(S_FILT_DATA_T *psFdata,u8 FiltCnt);
u16 PwmDutyStepUpdate(u16 targetduty,u16 currduty);
u16 MotorSpeedStepCtrl(u16 u16TargetSpeed,u16 u16RefSpeed);
void GetBatVol(S_FILT_DATA_T *psFdata,U_ERROR_FLAG_T *psflag);















#endif





















