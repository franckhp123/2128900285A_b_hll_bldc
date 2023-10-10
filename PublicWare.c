
#include "PublicWare.h"
#include "BLDC.h"
#include "GlobalVar.h"

//�ӳ�1us
void Delay_us(u16 cnt)  
{
   u16 i;
    
   for(i=0;i<cnt;i++)
   {
      NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
   }
}

//�ӳ�1ms
void Delay_ms(u16 cnt)  
{
   u16 i;

   for(i=0;i<cnt;i++)
   { Delay_us(1000); }
}

#if 0
//AD���ݼ�Ȩ�˲�
u8 Data_Weight_Filter(S_FILT_DATA_T *psFdata,u8 FiltCnt)
{
   psFdata->sum += psFdata->Ins; //�ۼӺ�
   if(++psFdata->cnt >= FiltCnt)
   {
      psFdata->cnt = 0;
	  psFdata->Result = psFdata->sum/FiltCnt;
	  psFdata->sum = 0;
	  return 1;
   }
   else
   { return 0; }
}
#endif

#if 0
//ռ�ձȱ仯б�º���
#define cUP_Speed_Fix  3
u16 PwmDutyStepUpdate(u16 targetduty,u16 currduty)
{
   if(currduty+cUP_Speed_Fix <= targetduty)     { currduty += cUP_Speed_Fix; }     
   else if(currduty > targetduty+cUP_Speed_Fix) { currduty -= cUP_Speed_Fix; }   
   else currduty = targetduty;

   if(currduty > cDUTY_FULL)       currduty = cDUTY_FULL;
   else if(currduty < cDUTY_MIN)   currduty = cDUTY_MIN;

   return currduty;
}
#endif

/*--------------------------------------------------------------  
             ���ת�ٿ���б�º���  
�Բο��ٶ�ֵ�ò���ֵ���м�/������,ʹ�ο��ٶ�ֵ�ӽ�Ŀ���ٶ�ֵ
---------------------------------------------------------------*/
u16 MotorSpeedStepCtrl(u16 u16TargetSpeed,u16 u16RefSpeed)                                       
{
   //if(u16RefSpeed+SPEED_UP_STEP <= u16TargetSpeed)       { u16RefSpeed += SPEED_UP_STEP;  }
   //else if(u16RefSpeed > u16TargetSpeed+SPEED_DOWN_STEP) { u16RefSpeed -= SPEED_DOWN_STEP; }
   if(u16RefSpeed+g_sMotor.PIRefSpeedUpValue <= u16TargetSpeed) 
   { u16RefSpeed += g_sMotor.PIRefSpeedUpValue;  }
   
   else if(u16RefSpeed > u16TargetSpeed+g_sMotor.PIRefSpeedUpValue) 
   { u16RefSpeed -= g_sMotor.PIRefSpeedUpValue; }
   
   else                                                  
   { u16RefSpeed = u16TargetSpeed; }
   return u16RefSpeed;
}

#if 0 
//�ɼ����������AD                 
void GetMotorZeroCurrent(S_FILT_DATA_T *psFdata,S_MOTOR_T *psMotor)  
{
   u8 i;   

   psFdata->sum = 0;
   
   for(i=0;i<8;i++)         
   {
      psFdata->sum += AdcSampSingleChannelOnce(IPeak_ADCH);             
   }
   psMotor->u16ZeroCurAD = psFdata->sum>>3;
   psFdata->sum = 0;
}
#endif

#if 0
//�ɼ���ذ���ѹAD
//void GetBatVol(S_FILT_DATA_T *psFdata,U_ERROR_FLAG_T *psflag)
//void GetBatVol(S_FILT_DATA_T *psFdata) 
void GetBatVol(S_FILT_DATA_T *psFdata,U_ERROR_FLAG_T *psflag)
{
   u8 i; 
   psFdata->sum = 0;  

   for(i=0;i<8;i++)
   { 
      psFdata->sum += AdcSampSingleChannelOnce(BusVdc_ADCH);  
   }
   psFdata->Ins = (psFdata->sum>>3);
   psFdata->RealValue = (psFdata->Ins>>2);  //��ѹ�Ŵ�10��     
   psFdata->sum = 0;
   if(psFdata->RealValue < 130)            //ĸ�ߵ�ѹ<13.0V���ϵ�Ƿѹ��־
   { psflag->Bits.bBusVdcUnder = 1; }
   //{ g_sFlagErr.Bits.bBusVdcUnder = 1; }    
}
#endif


