
#include "macrodriver.h"
#include "adc.h"
#include "userdefine.h"

void R_ADC_Create(void)
{
   ADCEN = 1U;     //�����ṩ����ʱ��
   ADM0 = 0x00U;   //��0 ADģʽ�Ĵ���
   
   ADMK = 1U;      //��ֹINTAD�ж�
   ADIF = 0U;      //��INTAD�жϱ�־λ

#if 0
   ADPR1 = 0U;     //����INTAD�ж����ȼ�1 
   ADPR0 = 1U;
#endif   
#if 1
   ADPR1 = 0U;     //����INTAD�ж����ȼ�0 
   ADPR0 = 0U;
#endif
    /* The reset status of ADPC is analog input, so it's unnecessary to set. */
   /* Set ANI0 - ANI3 pin */
   PM2 |= 0x0FU;		//����P2.0-P2.3Ϊ�����
   /* Set ANI16 - ANI19 pin */
  // PMC0 |= 0x03U;      //����P0.0(IRms)-P0.1(BemfU)Ϊģ���
   PM12 |= 0x01U;      //����P120Ϊ�����
   PM14 |= 0x80U;      //����P147Ϊ�����
//   PM0 |= 0x03U;       //����P0.0-P0.1Ϊ�����
   PMC12 |= 0x01U;     //����P120(BemfW)Ϊģ���
   PMC14 |= 0x80U;     //����P147(BemfV)Ϊģ���
   
	
//ע��:��ADCE(ֹͣ/�����ѹ�Ƚ�������)��1�����پ���1usȻ����ܽ�ADCS(ֹͣ/����ת��)λ��1
//ֹͣADת������,ɨ��ģʽ,ֹͣAD��ѹ�Ƚ�������,ת��ģʽ2,ת��ʱ��2.83us(5V����ת��ʱ����2.125-39us֮��)    
   //ADM0 = _40_AD_OPERMODE_SCAN | _30_AD_CONVERSION_CLOCK_4 | _02_AD_TIME_MODE_NORMAL_2; // 0111 0010  
//ֹͣADת������,ɨ��ģʽ,ֹͣAD��ѹ�Ƚ�������,ת��ģʽ2,ת��ʱ��3.54us(3.3V����ת��ʱ����3.1875-39us֮��) 
   ADM0 = _40_AD_OPERMODE_SCAN | _28_AD_CONVERSION_CLOCK_5 | _02_AD_TIME_MODE_NORMAL_2; // 0110 1010
   ADM1 = _00_AD_TRIGGER_SOFTWARE | _00_AD_CONVMODE_SEQSELECT; //�������ģʽ������ת��ģʽ
//ADת������׼��ѹ��VDD�ṩ,����׼��ѹVSS,ADLL��ADCR��ADUL(AREA1)ʱ�����ж��ź�(INTAD),10λ�ֱ���  
   ADM2 = _00_AD_POSITIVE_VDD | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 | _00_AD_RESOLUTION_10BIT;
   ADUL = _FF_AD_ADUL_VALUE; //ת������Ƚ�����ֵ�趨�Ĵ���
   ADLL = _00_AD_ADLL_VALUE; //ת������Ƚ�����ֵ�趨�Ĵ���
   ADS = _11_AD_INPUT_CHANNEL_17; //�趨ADCͨ��0b0001 0001   
    
   ADCE = 1U; //����ת������״̬:����AD��ѹ�Ƚ�������,ע������1US�������λADCS(ֹͣ/����ת��)  
}
/***********************************************************************************************************************
* Function Name: R_ADC_Start
* Description  : This function starts the AD converter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ADC_Start(void)
{
   ADIF = 0U;      //clear  INTAD interrupt flag
   ADMK = 0U;      //enable INTAD interrupt 
   ADCS = 1U;      //enables conversion operation 
}
/***********************************************************************************************************************
* Function Name: R_ADC_Stop
* Description  : This function stops the AD converter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ADC_Stop(void)
{
   ADCS = 0U;      //stops conversion operation 
   ADMK = 1U;      //disable INTAD interrupt 
   ADIF = 0U;      //clear INTAD interrupt flag 
}
/***********************************************************************************************************************
* Function Name: R_ADC_Set_OperationOn
* Description  : This function enables comparator operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ADC_Set_OperationOn(void)
{
    /* regbitset ADCE=1 */
    ADCE = 1U;      /* enables A/D voltage comparator operation */
}
/***********************************************************************************************************************
* Function Name: R_ADC_Set_OperationOff
* Description  : This function stops comparator operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ADC_Set_OperationOff(void)
{
    /* regbitset ADCE=0 */
    ADCE = 0U;      /* stops A/D voltage comparator operation */
}
/***********************************************************************************************************************
* Function Name: R_ADC_Get_Result
* Description  : This function returns the conversion result in the buffer.
* Arguments    : buffer -
*                    the address where to write the conversion result
* Return Value : None
***********************************************************************************************************************/
void R_ADC_Get_Result(uint16_t * const buffer)
{
    *buffer = (uint16_t) (ADCR >> 6U);
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
