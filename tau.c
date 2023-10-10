
#include "macrodriver.h"
#include "tau.h"
#include "userdefine.h"  

/******************************************************

  ��ʱ��0:
          ͨ��1:��ǰ����
          ͨ��3:1ms��ʱ������ѭ��ʱ��
          
******************************************************/

void R_TAU0_Create(void)  
{
//�ṩ����ʱ��(�ܶ�д��ʱ�����е�Ԫ0ʹ�õ�SFR)     
    TAU0EN = 1U;    
/*  TPS0:ʱ��ѡ����(4��),��ѡ��ʱ��0:fclk/2^2; ��ѡ��ʱ��1:fclk/2^4; ��ѡ��ʱ��2:fclk/2^1; ��ѡ��ʱ��3:fclk/2^8;*/
    TPS0 = _0000_TAU_CKM3_fCLK_8 | _0000_TAU_CKM2_fCLK_1 | _0040_TAU_CKM1_fCLK_4| _0002_TAU_CKM0_fCLK_2;

/* �رն�ʱ������ͨ�� */        
//TT0(ͨ��ֹͣ�Ĵ������ڴ����Ĵ���)��Ӧλ��1ֹͣ��ʱ��--->TE0(��ʱ������״̬�Ĵ���ֻ��)��Ӧλ��0--->TT0��Ӧλ�Զ���0
    TT0 = _0800_TAU_CH3_H8_STOP_TRG_ON | _0200_TAU_CH1_H8_STOP_TRG_ON | _0008_TAU_CH3_STOP_TRG_ON | 
          _0004_TAU_CH2_STOP_TRG_ON | _0002_TAU_CH1_STOP_TRG_ON | _0001_TAU_CH0_STOP_TRG_ON;   
	
    DISABLE_T0CHAN_0_INT;       //��ֹ��ʱ��0ͨ��0�ж�  
    CLR_T0CHAN_0_INTFLAG;       //�嶨ʱ��0ͨ��0�жϱ�־
    SET_T0CHAN_0_INTPRIO3;      //���ö�ʱ��0ͨ��0�ж�(�ж�����:INTTM00)���ȼ�Ϊ3 
    
    DISABLE_T0CHAN_1_INT;
	CLR_T0CHAN_1_INTFLAG;
    DISABLE_T0CHAN_1_HIGH8_INT; //ͨ��1�ĸ�8λ��ʱ���ж�ʹ��λINTTM01H:1��ֹ;0����  
	CLR_T0CHAN_1_HIGH8_INTFLAG; //INTTM01H�жϱ�־λ 
	SET_T0CHAN_1_INTPRIO2;      //���ö�ʱ��0ͨ��1�ж�(�ж�����:INTTM01)���ȼ�Ϊ2 
    
    DISABLE_T0CHAN_2_INT;
	CLR_T0CHAN_2_INTFLAG;
	SET_T0CHAN_2_INTPRIO2;      //���ö�ʱ��0ͨ��2�ж�(�ж�����:INTTM02)���ȼ�Ϊ2 

	DISABLE_T0CHAN_3_INT;
	//CLR_T0CHAN_3_INTFLAG;
	//SET_T0CHAN_3_INTPRIO3;      //���ö�ʱ��0ͨ��3�ж�(�ж�����:INTTM03)���ȼ�Ϊ3 

	//DISABLE_T0CHAN_3_HIGH8_INT; //ͨ��3�ĸ�8λ��ʱ���ж�ʹ��λINTTM03H:1��ֹ;0����  
   // CLR_T0CHAN_3_HIGH8_INTFLAG; //INTTM03H�жϱ�־λ
      
//��ʱ��0:ͨ��0����(���㻻����ʱ��)    	
//ͨ��0:ʱ��ѡ��CKM1(fclk/2^4),����ʱ��ѡ����CKSλ����(��CKM1)	,���������ʼ,�����ʱģʽ,�ڿ�ʼ����ʱ��������ʱ���ж�
    TMR00 = _8000_TAU_CLOCK_SELECT_CKM1|_0000_TAU_CLOCK_MODE_CKS|_0000_TAU_TRIGGER_SOFTWARE|
            _0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;
	TDR00 = _FFFF_TAU_TDR00_VALUE; //��ʱ���ȽϼĴ���ֵΪĬ��ֵ
//��ʱ�����:ֻ���ڽ�ֹ��ʱ�����(TOE0=0��ʱ����ͨ�������д�˼Ĵ�����TOmnλ    
	TO0 &= (uint16_t)~_0001_TAU_CH0_OUTPUT_VALUE_1; 
	TOE0 &= (uint16_t)~_0001_TAU_CH0_OUTPUT_ENABLE; //��ֹ��ʱ�����   
	
//��ʱ��0:ͨ��1����
/*  ͨ��1:ʱ��ѡ��CKM1(fclk/2^4),����ʱ��ѡ����CKSλ����(��CKM1),����16λ��ʱ��
          ���������ʼ,�����ʱģʽ,�ڿ�ʼ����ʱ��������ʱ���ж� */
    TMR01 = _8000_TAU_CLOCK_SELECT_CKM1|_0000_TAU_CLOCK_MODE_CKS|_0000_TAU_16BITS_MODE|
	        _0000_TAU_TRIGGER_SOFTWARE|_0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;
    TDR01 = _FFFF_TAU_TDR01_VALUE;
//��ʱ�����ģʽ�Ĵ���:����ͨ�����,����ͨ�����  
    TOM0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_COMBIN;  
//��ʱ�������ƽ�ļĴ���
    TOL0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_LEVEL_L; 
//��ʱ�����:ֻ���ڽ�ֹ��ʱ�����(TOE0=0��ʱ����ͨ�������д�˼Ĵ�����TOmnλ  
    TO0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_VALUE_1;
	TOE0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_ENABLE; //��ֹ��ʱ�����
	
//��ʱ��0:ͨ��2����
/***	ͨ��2:ʱ��ѡ��CKM1(fclk/2^4),����ʱ��ѡ����CKSλ����(��CKM1),��������ͨ��,  
	      ���������ʼ,�����ʱģʽ,�ڿ�ʼ����ʱ��������ʱ���ж� ***/
    TMR02 = _8000_TAU_CLOCK_SELECT_CKM1 | _0000_TAU_CLOCK_MODE_CKS|_0000_TAU_COMBINATION_SLAVE|
	        _0000_TAU_TRIGGER_SOFTWARE|_0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;
    TDR02 = _FFFF_TAU_TDR02_VALUE;
	TOM0 &= (uint16_t)~_0004_TAU_CH2_OUTPUT_COMBIN;  //��ʱ�����ģʽ�Ĵ���:����ͨ�����,����ͨ�����  
	TOL0 &= (uint16_t)~_0004_TAU_CH2_OUTPUT_LEVEL_L; //��ʱ�������ƽ�ļĴ���
	TO0 &= (uint16_t)~_0004_TAU_CH2_OUTPUT_VALUE_1;  //��ʱ�����:ͨ��2���0
	TOE0  &= (uint16_t)~_0004_TAU_CH2_OUTPUT_ENABLE; //��ֹ��ʱ�����
	
//��ʱ��0:ͨ��3����
/*ͨ��3:ʱ��ѡ��CKM0(fclk/2^2),����ʱ��ѡ����CKSλ����(��CKM0),����16λ��ʱ��
	    ���������ʼ,�����ʱģʽ,�ڿ�ʼ����ʱ��������ʱ���ж�     
*/  
  //  TMR03 = _0000_TAU_CLOCK_SELECT_CKM0|_0000_TAU_CLOCK_MODE_CKS|_0000_TAU_16BITS_MODE| 
	//        _0000_TAU_TRIGGER_SOFTWARE|_0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;

//+2����Ϊ��ʼ����ʱ1clock����TDR(�ȽϼĴ���ֵ);1clock�ڵݼ�������0ʱ�����ж�  
//	TDR03 = 5998;  //TDR03:1ms:24M/2^2=6M->(1/6M)*(5998+2)=1ms;  
//	TOM0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_COMBIN;  //��ʱ�����ģʽ�Ĵ���:����ͨ�����,����ͨ�����  
//	TOL0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_LEVEL_L; //��ʱ�������ƽ�ļĴ���
//	TO0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_VALUE_1;  //��ʱ�����:ͨ��3���0
//	TOE0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_ENABLE;  //��ֹ��ʱ��ͨ��3���
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel0_Start
* Description  : This function starts TAU0 channel 0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel0_Start(void)
{
   //CLR_T0CHAN_0_INTFLAG;               //���־
   //ENABLE_T0CHAN_0_INT;                //ʹ���ж�
   
   TS0 |= _0001_TAU_CH0_START_TRG_ON;  //������ʱ��
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel0_Stop
* Description  : This function stops TAU0 channel 0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel0_Stop(void)
{
   TT0 |= _0001_TAU_CH0_STOP_TRG_ON;

   //CLR_T0CHAN_0_INTFLAG;
   //DISABLE_T0CHAN_0_INT;
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel1_Start
* Description  : This function stops TAU0 channel 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel1_Start(void)
{
   CLR_T0CHAN_1_INTFLAG;
   ENABLE_T0CHAN_1_INT;
   
   TS0 |= _0002_TAU_CH1_START_TRG_ON;
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel1_Stop
* Description  : This function stops TAU0 channel 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel1_Stop(void)
{
   TT0 |= _0002_TAU_CH1_STOP_TRG_ON;

   DISABLE_T0CHAN_1_INT;
   CLR_T0CHAN_1_INTFLAG;
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel2_Start
* Description  : This function stops TAU0 channel 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel2_Start(void)
{
   CLR_T0CHAN_2_INTFLAG;
   ENABLE_T0CHAN_2_INT;

   TS0 |= _0004_TAU_CH2_START_TRG_ON;
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel2_Stop
* Description  : This function stops TAU0 channel 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel2_Stop(void)
{
   TT0 |= _0004_TAU_CH2_STOP_TRG_ON;
   
   DISABLE_T0CHAN_2_INT;
   CLR_T0CHAN_2_INTFLAG;
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel3_Start
* Description  : This function stops TAU0 channel 3 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel3_Start(void) //������1msʱ��
{
   DISABLE_T0CHAN_3_INT;
   CLR_T0CHAN_3_INTFLAG;
   
   TS0 |= _0008_TAU_CH3_START_TRG_ON;      //������ʱ��,TE0��Ӧλ��1��,������־�Զ���0
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel3_Stop
* Description  : This function stops TAU0 channe3 1 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel3_Stop(void)
{
   TT0 |= _0008_TAU_CH3_STOP_TRG_ON;

   DISABLE_T0CHAN_3_INT;
   CLR_T0CHAN_3_INTFLAG;
}


