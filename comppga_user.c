
#include "comppga.h"
#include "tmrd.h"  
#include "GlobalVar.h"
#include "PublicWare.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt r_comp1_interrupt(vect=INTCMP1)
/***********************************************************************************************************************
* Function Name: r_comp0_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_comp1_interrupt(void)
{

/*
   ���㷽��:CORVM=_51_CORVM_VALUE
   �Ƚ���:��׼ֵ=5V*(_51_CORVM_VALUE/255)=1.59V
          �Ŵ���=�˷�*�ڲ��˷�=2*8=16
   ����:I=��׼ֵ/�Ŵ���(20)=1590/16=99A
*/
    CMPIF1 = 0U;    /* clear INTCMP0 interrupt flag */
	g_sFlagErr.Bits.bShortCurrent = 1;
	R_TMRD0_Stop();
	R_TAU0_Channel0_Stop(); //CH0����ʱ���� CH1��ʱ�����趨
	R_ADC_Stop();
	
	P1 &= 0xc0;
	TRDOER1 = 0xFA; //0xc8=������PWM,0XFA��PWM(ֻ��PWMû�˲���,�жϻ��ᴥ��)

}

