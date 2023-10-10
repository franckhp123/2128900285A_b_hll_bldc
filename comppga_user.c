
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
   计算方法:CORVM=_51_CORVM_VALUE
   比较器:基准值=5V*(_51_CORVM_VALUE/255)=1.59V
          放大倍数=运放*内部运放=2*8=16
   电流:I=基准值/放大倍数(20)=1590/16=99A
*/
    CMPIF1 = 0U;    /* clear INTCMP0 interrupt flag */
	g_sFlagErr.Bits.bShortCurrent = 1;
	R_TMRD0_Stop();
	R_TAU0_Channel0_Stop(); //CH0换相时间监控 CH1延时换相设定
	R_ADC_Stop();
	
	P1 &= 0xc0;
	TRDOER1 = 0xFA; //0xc8=开所有PWM,0XFA关PWM(只是PWM没了波形,中断还会触发)

}

