
#ifndef __PublicWare_H
#define __PublicWare_H

#include "macrodriver.h"
#include "userdefine.h"
#include "adc.h"

//�¼�������־������
typedef union
{
   u16 all;
   struct
   {
      u16 bDirFlash           :1; // 00 ��˸��ʾ��־ 
      u16 bFirInitCurBuf      :1; // 01 �״γ�ʼ�����������־
	  u16 breserved2	      :1; // 02 ����
	  u16 breserved3          :1; // 03 ����
	  u16 bMosCheckFlag	      :1; // 04 mos�����ϵ��Լ�
	  u16 breserved5          :1; // 05 ����
	  u16 breserved6	      :1; // 06 ����
	  u16 breserved7          :1; // 07 ����
	  u16 breserved8	      :1; // 08 ����
	  u16 breserved9          :1; // 09 ����
	  u16 bStopInterINTFlag	  :1; // 10 ���ϻ�ɲ���󼴽����ж�,��ִ���жϻ��ຯ��
	  u16 breserved11         :1; // 11 ����
	  u16 breserved12	      :1; // 12 ����
	  u16 bMosHTmpClr	      :1; // 13 MOS���¹��Ͻ����־
	  u16 bBusIdcOcClr	      :1; // 14 ĸ�ߵ����������Ͻ����־
	  u16 bBusVdcErrClr	      :1; // 15 ĸ�ߵ�ѹ�͹��Ͻ����־
   }Bits;
}U_EVENT_TRIG_FLAG_T;  

//�¼�������־������
typedef union
{
   u16 all;
   struct
   {
      u16 bMosOverTmp     :1; // 00 MOS���¹���      
      u16 bMosTempUnder   :1; // 01 MOS���¹��� 
      u16 bBatTempOver    :1; // 02 ��ذ����¹���
	  u16 bBatTempUnder   :1; // 03 ��ذ����¹���
	  u16 bStart          :1; // 04 Ԥ��������
	  u16 bHall           :1; // 05 HALL����
	  u16 bIdcLimit       :1; // 06 ������ʱ
	  u16 bAvgIdcShort    :1; // 07 ��ֵ������·
	  u16 bAvgIdcOver     :1; // 08 ��ֵ��������
	  u16 bShortCurrent	  :1; // 09 ��ֵ������·
	  u16 bPeakIdcOver    :1; // 10 ��ֵ��������
	  u16 bBusVdcOver	  :1; // 11 ֱ��ĸ�߹�ѹ
	  u16 bBusVdcUnder    :1; // 12 ֱ��ĸ��Ƿѹ
	  u16 bMotorBlock	  :1; // 13 �����ת
	  u16 bPosition	      :1; // 14 ��λʧ�ܹ���  
	  u16 bObstacles	  :1; // 15 �ϰ������   
   }Bits;
}U_ERROR_FLAG_T;  

#if 0
typedef struct
{
   u16 bMosOverTmp     :1; // 00 MOS���¹���      
   u16 bMosTempUnder   :1; // 01 MOS���¹��� 
   u16 bBatTempOver    :1; // 02 ��ذ����¹���
   u16 bBatTempUnder   :1; // 03 ��ذ����¹���
   u16 bStart          :1; // 04 Ԥ��������
   u16 bHall           :1; // 05 HALL����
   u16 bIdcLimit       :1; // 06 ������ʱ
   u16 bAvgIdcShort    :1; // 07 ��ֵ������·
   u16 bAvgIdcOver     :1; // 08 ��ֵ��������
   u16 bShortCurrent   :1; // 09 ��ֵ������·
   u16 bPeakIdcOver    :1; // 10 ��ֵ��������
   u16 bBusVdcOver	   :1; // 11 ֱ��ĸ�߹�ѹ
   u16 bBusVdcUnder    :1; // 12 ֱ��ĸ��Ƿѹ
   u16 bMotorBlock	   :1; // 13 �����ת
   u16 bPosition	   :1; // 14 ��λʧ�ܹ���    
   u16 bAccelOver	   :1; // 15 
}U_ERROR_T;
#endif

typedef enum 
{
   cSTA_SELFCHK , //�Լ�״̬
   cSTA_WAIT    , //����ȫ��״̬   
   cSTA_RUN     , //����״̬
   cSTA_STOP    , //ͣ��ɲ��״̬
   cSTA_ERR       //����״̬
}E_SYS_STA_T;


//AD����ṹ������
typedef struct
{
   u8   cnt;       //�˲�����
   u16  Result;    //�����˲����      
   u16  FiltRes1;  //һ���˲����
   u16  Ins;       //˲ʱֵ 
   u8   RealValue; //ʵ��ֵ(ģ������ѹ�¶ȵ�)
   u16  sum;       //�ۼӺ�
}S_FILT_DATA_T; //�˲�����

typedef union
{
   u16 all[4];
   struct
   {
      u16 u16Common;      // 0 ����ͨ��
      u16 u16Conduction;  // 1 ��ͨ���ѹ
      u16 u16Bemf;        // 2 ���������ѹ
      u16 u16PhaseCurr;   // 3 �����(ĸ��PeakIdc˲ʱֵ)
   };
}U_FLUX_AD_RES_T; //��������ʱADֵ����ṹ��(����ADֵתʵ��ֵ,�����������)

typedef union
{
   u16 aAllData[8];
   struct
   {
      u16 u16Common;      // 0 ����ͨ�� 
      u16 u16Conduction;  // 1 ��ͨ���ѹ
      u16 aBemf[5];       // 2~6���������ѹ
      u16 u16PhaseCurr;   // 7 �����(ĸ��PeakIdc˲ʱֵ)
   };
}U_BEMF_AD_RES_T; //����������ʱADֵ����ṹ��(���뷴���ƹ�����Ƚ�)


typedef struct
{
  s32  K1;       // K1 = Kp * (1 + Ki)
  s32  K2;       // K2 = Kp
  s32  lastErr;  // ��һ��ƫ��ֵ 
  s32  out;      // ��һ�����ֵ
  s32  outMin;   // ��С�޷�ֵ
  s32  outMax;   // ����޷�ֵ
  u8   timePIUpGiveSpeed;   //PI�������ٶ�ʱ��
  u16  PIPremeChgtime;      //PI��������ʱ�� 
  u8   u8RefSpeedTimeValue; //PI�������ٶȸı�ʱ��
  u16  u16PILowSpeedTimeVal;   //����ʱPI��������ʱ��
}S_INC_PI_PARA_T; // ����ʽPI���� 

//λ����pid
typedef struct
{
   float Kp;        //����ϵ��Proportional
   float Ki;        //����ϵ��Integral
   float Kd;        //΢��ϵ��Derivative
   float Ek;        //��ǰ���
   float Ek1;       //ǰһ����� e(k-1)
   float Ek2;       //��ǰһ����� e(k-2)
   float LocSum;    //�ۼƻ���λ��
}PID_LocTypeDef;


//���ϼ��ṹ��   
typedef enum   //�¶�ö��
{
   eFlat,      //ƽ��
   eUpSlope1,  //���¶�1
   eUpSlope2,  //���¶�2
   eUpSlope3,  //���¶�3
   eDnSlope1,  //���¶�1
   eDnSlope2,  //���¶�2
   eDnSlope3  //���¶�3   
}E_GRADE_T;  

typedef struct
{
   u8         u8SurgeCurTime;        //ӿ��ʱ��
   u16        u16SumOneLevelCur;     //һ��10�������ܺ�
   u16        u16Sum32OneLevelCur;   //һ��10�������ܺ�
   u16        sc_value[5];           //б�ʼ�����ֵ
   u16        u16AvrCurOneLevel[4];  //һ����������    
   u16        u16SumCurTwoAvr;       //���������ܺ�ƽ��ֵ 
   u8         u8SampOneLevelCurCnt;  //��������������
   u16        u16SRTheshod;          //б����ֵ
   u16        u16AvrCurTwoLevel[5];  //������������  
   s16        Grade;                 //�����¶Ƚ�
   E_GRADE_T  eGrade;                //�¶�ö��           
   //u8   ObstaclErrTxCnt;       //��ײ���Ϸ��ʹ���
}S_OBSTACL_T;


//�궨��
#if 0
/* D48 Speed */
#define SPEED_MAX        32767 // �ٶ����ֵ(RPM)
#define SPEED_MIN        255   // �ٶ���Сֵ(RPM)
#define SPEED_RATED      10000 // �ת��(RPM)
#define SPEED_UP_STEP    400   // ת�����Ӳ���ֵ=�ת��*0.02
#define SPEED_DOWN_STEP  400   // ת�ټ�С����ֵ=�ת��*0.02
#endif

#define SPEED_MAX        4000  // �ٶ����ֵ(RPM)   
#define SPEED_MIN        100   // �ٶ���Сֵ(RPM)
#define SPEED_RATED      3500  // �ת��(RPM)
//#define SPEED_UP_STEP    76    // ת�����Ӳ���ֵ=�ת��*0.02
//#define SPEED_DOWN_STEP  76    // ת�ټ�С����ֵ=�ת��*0.02
#define SPEED_UP_STEP    175    // ת�����Ӳ���ֵ=�ת��*0.05         
#define SPEED_DOWN_STEP  175    // ת�ټ�С����ֵ=�ת��*0.05   
//#define SPEED_UP_STEP    350    // ת�����Ӳ���ֵ=�ת��*0.1         
//#define SPEED_DOWN_STEP  350    // ת�ټ�С����ֵ=�ת��*0.1  
//#define SPEED_UP_STEP    500    // ת�����Ӳ���ֵ=�ת��*
//#define SPEED_DOWN_STEP  500    // ת�ټ�С����ֵ=�ת��*  



//��������
void Delay_us(u16 cnt);
void Delay_ms(u16 cnt);
u8 Data_Weight_Filter(S_FILT_DATA_T *psFdata,u8 FiltCnt);
u16 PwmDutyStepUpdate(u16 targetduty,u16 currduty);
u16 MotorSpeedStepCtrl(u16 u16TargetSpeed,u16 u16RefSpeed);
void GetBatVol(S_FILT_DATA_T *psFdata,U_ERROR_FLAG_T *psflag);















#endif





















