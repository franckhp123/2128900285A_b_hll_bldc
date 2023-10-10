
#ifndef _USER_DEF_H
#define _USER_DEF_H

//头文件
#include "macrodriver.h"   

typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;

typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;  

//程序版本
#define DEMOBOARD     0
#define ROBTICMOWER   1
#define DRILL         2

//#define PROJECTNAME   DEMOBOARD  //Demo
#define PROJECTNAME   ROBTICMOWER //RobticMower
//#define PROJECTNAME   DRILL //Drill



//位操作定义
#define TESTBIT(a,b)  ((a)&(1<<(b)))
#define SETBIT(a,b)  ((a)|=(1<<(b)))
#define CLRBIT(a,b)  ((a)&=~(1<<(b))) 


#define LampOn()         SETBIT(P1,6)
#define LampOff()        CLRBIT(P1,6)
#define LampFlash()      P1_bit.no6=!(P1_bit.no6)     

#define DebugIOHigh     P1_bit.no7 = 1       
#define DebugIOLow      P1_bit.no7 = 0
#define DebugIOFlash    P1_bit.no7=!(P1_bit.no7)      
//#define DebugADSHigh    P2_bit.no3 = 1     
//#define DebugADSLow     P2_bit.no3 = 0 
//#define DebugADSFlash   P2_bit.no3=!(P2_bit.no3)


//配置ADC通道对应的实际采集物理量
#define ADCMODE0_SINGSCAN    ADM1_bit.no5 = 1;ADM0_bit.no6 = 1 //单次扫描模式
#define ADCMODE1_SINGSELECT  ADM1_bit.no5 = 1;ADM0_bit.no6 = 0 //单次选择模式:启动转换(ADCS=1)-->转换开始-->标志自动清0(ADCS=0)-->INTAD=1
#define ADCMODE2_CONNSELECT  ADM1_bit.no5 = 0;ADM0_bit.no6 = 0 //连续选择模式:启动转换(ADCS=1)-->转换开始-->INTAD=1;改写通道时开始新转换
#define ADCMODE3_CONNSCAN    ADM1_bit.no5 = 0;ADM0_bit.no6 = 1 //连续扫描模式

#if 0
//AD通道宏定义(调试板)
#define V_BEMF_ADCH     ADC_CH0  
#define UVW_CMM_ADCH    ADC_CH1
#define BusVdc_ADCH     ADC_CH2   //直流母线电压    
#define ADS_ADCH        ADC_CH3
#define PGAO_ADCH       ADC_PGAO
#define MosTmp_ADCH     ADC_CH17 
#define W_BEMF_ADCH     ADC_CH18  
#define U_BEMF_ADCH     ADC_CH19
#endif
//AD通道宏定义(正式板)
#define UVW_CMM_ADCH    ADC_CH0
#define MosTmp_ADCH     ADC_CH1 
#define IAvr_ADCH       ADC_CH2    //母线电流平均值
#define BusVdc_ADCH     ADC_CH3    //直流母线电压   
#define U_BEMF_ADCH     ADC_CH16   //U相相电压   
#define IPeak_ADCH      ADC_CH17   //母线电流峰值
#define V_BEMF_ADCH     ADC_CH18   //V相相电压 
#define W_BEMF_ADCH     ADC_CH19   //W相相电压


//#define IMPULSE_INJECT_TIME_uS   90 //脉冲注入定位脉冲时间
#define IMPULSE_INJECT_TIME_uS   700 //脉冲注入定位脉冲时间
#define POSITION_FALL_CNT        5  //定位失败次数  

//FLUX 磁链
#define FLUX_L_dI_dt_DEFAULT     6144 //磁链简化常数初始默认值
//#define FLUX_THRESHOLD           60   //磁链换相阈值
#define FLUX_THRESHOLD           20   //磁链换相阈值            

#define FLUX_THRESHOLD_SHADOW    60   //磁链影子换相阈值
#define FLUX_BEMF_ERR_V          3    //前后两次反电势差值
#define FLUX_BEMF_ERR_SHADOW_V   4

/* Hall INT pin define */

#define	HALL_U		P3_bit.no1  
#define	HALL_V		P13_bit.no7
#define	HALL_W		P3_bit.no0

typedef void (*FNP_VOID_T)(void); //函数指针

#define TEST_ON()     P1_bit.no6 = 1
#define TEST_OFF()    P1_bit.no6 = 0
#define TEST_REV()    P1_bit.no6 = !(P1_bit.no6)

#define LED_ON()      P1_bit.no7 = 1
#define LED_OFF()     P1_bit.no7 = 0
#define LED_REV()     P1_bit.no7 = !(P1_bit.no7)

#if 0
//宏定义
#define POWER_ON()    P6_bit.no0 = 1
#define POWER_OFF()   P6_bit.no0 = 0
#define POWER_REV()   P6_bit.no0 = !(P6_bit.no0)

#define TEST_ON()     P1_bit.no6 = 1
#define TEST_OFF()    P1_bit.no6 = 0
#define TEST_REV()    P1_bit.no6 = !(P1_bit.no6)

#define LED_ON()      P1_bit.no7 = 1
#define LED_OFF()     P1_bit.no7 = 0
#define LED_REV()     P1_bit.no7 = !(P1_bit.no7)

#define LED1_ON()     P3_bit.no0 = 1
#define LED1_OFF()    P3_bit.no0 = 0
#define LED1_REV()    P3_bit.no0 = !(P3_bit.no0)

#define LED2_ON()     P5_bit.no0 = 1
#define LED2_OFF()    P5_bit.no0 = 0
#define LED2_REV()    P5_bit.no0 = !(P5_bit.no0)

#define LED3_ON()     P3_bit.no1 = 1
#define LED3_OFF()    P3_bit.no1 = 0
#define LED3_REV()    P3_bit.no1 = !(P3_bit.no1)

#define DI_OFF_VAL    P7_bit.no0 //L-关机

#define ADC_FIFO_LEVEL2     1
#define ADC_FIFO_LEVEL3     2
#define ADC_FIFO_LEVEL4     3
#define ADC_FIFO_LEVEL5     4
#define ADC_FIFO_LEVEL6     5
#define ADC_FIFO_LEVEL7     6
#define ADC_FIFO_LEVEL8     7

#define DUTY_FULL      3167 //TRDGRA0 满占空比
#define DUTY_MAX       ((uint16_t)(DUTY_FULL*1.0))
#define DUTY_MIN       ((uint16_t)(DUTY_FULL*0.15))
#define DUTY_BOOST     ((uint16_t)(DUTY_FULL*0.50))
#define DUTY_START     ((uint16_t)(DUTY_FULL*0.20))
#define DUTY_8         ((uint16_t)(DUTY_FULL*0.08))
#define DUTY_15        ((uint16_t)(DUTY_FULL*0.15))
#define DUTY_18        ((uint16_t)(DUTY_FULL*0.18))
#define DUTY_20        ((uint16_t)(DUTY_FULL*0.20))
#define DUTY_25        ((uint16_t)(DUTY_FULL*0.25))
#define DUTY_30        ((uint16_t)(DUTY_FULL*0.30))
#define DUTY_40        ((uint16_t)(DUTY_FULL*0.40))
#define DUTY_45        ((uint16_t)(DUTY_FULL*0.45))
#define DUTY_50        ((uint16_t)(DUTY_FULL*0.50))
#define DUTY_60        ((uint16_t)(DUTY_FULL*0.60))
#define DUTY_70        ((uint16_t)(DUTY_FULL*0.70))
#define DUTY_80        ((uint16_t)(DUTY_FULL*0.80))
#define DUTY_85        ((uint16_t)(DUTY_FULL*0.85))
#define DUTY_90        ((uint16_t)(DUTY_FULL*0.90))
#define DUTY_95        ((uint16_t)(DUTY_FULL*0.95))
#define DUTY_100       ((uint16_t)(DUTY_FULL)) //4799
#define DUTY_STEP_UP_VAL      200
#define DUTY_STEP_DOWN_VAL    100 // 15
#define BARKE_EN              1  //刹车允许

#define SPD_0_DELAY_ANGLE        5
#define SPD_0_CW_DELAY_ANGLE     5
#define SPD_0_CCW_DELAY_ANGLE    5


#define POSITION_CNT             3  //一次定位操作中的定位数量

#define START_FALL_CNT           5  //启动失败次数


//配置ADC通道对应的实际采集物理量
#define ADCMODE0  ADM1_bit.no5 = 1;ADM0_bit.no6 = 1 //单次扫描模式
#define ADCMODE1  ADM1_bit.no5 = 1;ADM0_bit.no6 = 0 //单次选择模式:启动转换(ADCS=1)-->转换开始-->标志自动清0(ADCS=0)-->INTAD=1
#define ADCMODE2  ADM1_bit.no5 = 0;ADM0_bit.no6 = 0 //连续选择模式:启动转换(ADCS=1)-->转换开始-->INTAD=1;改写通道时开始新转换
#define ADCMODE3  ADM1_bit.no5 = 0;ADM0_bit.no6 = 1 //连续扫描模式

#define BusVdc_ADCH    ADC_CH0  //直流母线电压
#define MosTemp_ADCH   ADC_CH1  //MOS温度
#define BatTemp_ADCH   ADC_CH2
#define VR_ADCH        ADC_CH3
#define PeakIdc_ADCH   ADC_CH17 //直流母线电流峰值
#define U_BEMF_ADCH    ADC_CH16
#define V_BEMF_ADCH    ADC_CH18
#define W_BEMF_ADCH    ADC_CH19
#define ZERO_CROSS_TIME_BASE  2250000

//BusVdc
#define BusVdc_HANDLE_EN    true
#define BusVdc_UpDivRes_R    10000 //母线电压的上分压电阻
#define BusVdc_DownDivRes_R  1000  //母线电压的下分压电阻
#define BusVdc_OFFSET_AD     0u    //AD误差校准
#define BusVdc_OVER_V        30    //母线电压过压阈值
#define BusVdc_HIGH_V        25    //母线电压高压阈值
#define BusVdc_HIGH_CLR_V    22    //母线电压高压恢复阈值
#define BusVdc_LOW_CLR_V     18    //母线电压低压恢复阈值
#define BusVdc_LOW_V         15    //母线电压低压阈值
#define BusVdc_UNDER_V       12.5  //母线电压欠压阈值
#define BusVdc_START_V       15    //允许启动母线电压欠压阈值
#define BusVdc_DELTA_V       2     //允许刹车母线电压阈值
#define BusVdc_MEAN_SHIFT    7     //母线电压在算术平均值滤波移位
#define BEMF_UpDivRes_R      10000 //反电势上分压电阻
#define BEMF_DownDivRes_R    2200  //反电势下分压电阻2.2K
#define BEMF_UpDivRes_RD     10000 //反电势上分压电阻10K
#define BEMF_DownDivRes_RD   2200  //反电势下分压电阻2.2K
#define BEMF_DownDeTotal_R   5     // 2.2K/12.2K=5.54

#define AC_VOLT_FREQ_Hz            50    //AC电压的频率
#define AC_VOLT_ZERO_CROSS_PWM_DIV  1





typedef void (*FNP_VOID_T)(void); //函数指针
#endif

#endif
