
#include "macrodriver.h"
#include "tau.h"
#include "adc.h"           //dirll
#include "userdefine.h"   
#include "GlobalVar.h"     //dirll    
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt r_tau0_channel0_interrupt(vect=INTTM00)
#pragma interrupt r_tau0_channel1_interrupt(vect=INTTM01)
#pragma interrupt r_tau0_channel2_interrupt(vect=INTTM02)
#pragma interrupt r_tau0_channel3_interrupt(vect=INTTM03)


static void __near r_tau0_channel0_interrupt(void)
{
   TMIF00 = 0U;


}
/***********************************************************************************************************************
* Function Name: r_tau0_channel1_interrupt
* Description  : This function INTTM00 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_tau0_channel1_interrupt(void) //��ǰ������   
{
   TMIF01 = 0U;     
   
   if(g_sFlagEve.Bits.bStopInterINTFlag == 0) //���ϻ�ͣ�����ٽ���  
   {
      if(g_sMotor.uWorkFlag.bGetBemfZeroCross) //֮ǰ�Ǽ�⵽�����
      {
         if(g_sMotor.eDir == eCW)
         {
            switch(g_sMotor.u8Phase)
			{
               case 0: NullPhase();     break;
			   case 1: WH_VL_HPwmLON(); break;
			   case 2: UH_WL_HPwmLON(); break;
			   case 3: UH_VL_HPwmLON(); break;
			   case 4: VH_UL_HPwmLON(); break;
			   case 5: WH_UL_HPwmLON(); break;
			   case 6: VH_WL_HPwmLON(); break;
               default:break;
			}
         }
		 else if(g_sMotor.eDir == eCCW)
		 {
            switch(g_sMotor.u8Phase)
			{
               case 0: NullPhase();     break;
			   case 1: UH_VL_HPwmLON(); break;
			   case 2: WH_UL_HPwmLON(); break;
			   case 3: WH_VL_HPwmLON(); break;
			   case 4: VH_WL_HPwmLON(); break;
			   case 5: UH_WL_HPwmLON(); break;
			   case 6: VH_UL_HPwmLON(); break;
               default:break;
			}
         }

		 g_sMotor.u8Phase = g_aNextPhase[g_sMotor.u8Phase];

         //ISR_PwmAdcInit1st(ADC_FIFO_LEVEL4) //����ADC����ͨ��
         //���ٺ�������(stack)����ADC����ͨ��
          eaAdFifoChannelBuf[1] = g_sPhaseAdChannel.eaBemf[g_sMotor.u8Phase-1];//������
		  eaAdFifoChannelBuf[2] = g_sPhaseAdChannel.eaConduction[g_sMotor.u8Phase-1];//��ͨ��
		  g_bCommutationOK = 1; //�û���ɹ���־
		  g_sMotor.uWorkFlag.bGetBemfZeroCross = 0; //�巴���ƹ����־
          
		  g_u8NoAdcStartFlag = 0; //��ִ���껻��(PWM�ж��ڿ�ʹ��ADת��)  
		  g_u16BlockCnt = 0;      //�ɹ��������ת������
	   }
	   
	   R_TAU0_Channel1_Stop();
   }
}
/***********************************************************************************************************************
* Function Name: r_tau0_channel2_interrupt
* Description  : This function INTTM02 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_tau0_channel2_interrupt(void)
{
   /* Start user code for adding. Do not edit comment generated here */

   //R_TAU0_Channel2_Stop();
   /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_tau0_channel3_interrupt
* Description  : This function INTTM03 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_tau0_channel3_interrupt(void) // 1ms��ʱ��
{
   #if 0
   u8Flag_1ms = 1;
   #endif
   
   TMIF03 = 0U;
}

