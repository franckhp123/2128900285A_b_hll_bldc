
#include "adc.h"  
#include "GlobalVar.h"  
#include "sau.h"

//AD����(10λ/1024):����ѡ��ȴ�ת�����,���жϷ�ʽ
u16 AdcSampSingleChannelOnce(u8 eChannel)
{
   ADS = eChannel;
   ADIF = 0;        //��INTAD�жϱ�־   
   ADCS = 1;        //����ADת������
   while(!ADIF);
   ADIF = 0;
   ADCS = 0;        //��ֹADת��

   return(ADCR>>6); //ת�����������ADCR��10λ  
}


/***********************************************************************************************************************
* Function Name: r_adc_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
/*
   g_u16ADCBuffer[1]:����������AD����            
   g_u16ADCBuffer[2]:��ͨ��
   g_u16ADCBuffer[3]:ĸ�ߵ���   
   ADCRΪ16λ�Ĵ���,��10λ��Ž��(�ֱ���Ϊ10),AD�ж���ת�������10λ����������2λ(Ӧ����4ʵ������2)ת����12λ���ݸ�ʽ��
*/







