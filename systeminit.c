
#include "macrodriver.h"  
#include "cgc.h"  
#include "port.h"
#include "tau.h"  
#include "sau.h"
#include "adc.h"
#include "comppga.h"
#include "tmrd.h"
#include "elc.h"
#include "userdefine.h"  

void R_Systeminit(void)  
{
#if 1
   PIOR1 = 0x00U;           //��ʱ��RJ TRJO0��P30/P01����
   R_CGC_Get_ResetSource(); //ʱ������,���ڱ����������������ù�
   R_CGC_Create();          //ϵͳʱ������        
   R_PORT_Create();		    //GPIO����
   R_TAU0_Create();		    //��ʱ������  
   R_TMRD0_Create();        //��ʱ��RD����(PWM) // 
   R_SAU0_Create();         //��������
   R_ADC_Create(); 		    //ADC���� 
   R_COMPPGA_Create(); 	    //�Ƚ���/�Ŵ�������  
   R_ELC_Create();          //ELC����
    R_INTC_Create();
    R_IT_Create();
   IAWCTL = 0x00U;
   #endif 
   #if 0
      PIOR1 = 0x00U;           //��ʱ��RJ TRJO0��P30/P01����
   R_CGC_Get_ResetSource(); //ʱ������,���ڱ����������������ù�
   R_CGC_Create();          //ϵͳʱ������        
   R_PORT_Create();		    //GPIO����
   R_TAU0_Create();		    //��ʱ������  
   R_TMRD0_Create();        //��ʱ��RD����(PWM)  
   R_SAU0_Create();         //��������
   R_ADC_Create(); 		    //ADC���� 
 //  R_COMPPGA_Create(); 	    //�Ƚ���/�Ŵ�������  
   R_ELC_Create();          //ELC����
   R_INTC_Create();
   IAWCTL = 0x00U;
   #endif 
}

/***********************************************************************************************************************
* Function Name: hdwinit
* Description  : This function initializes hardware setting.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void hdwinit(void)
{
   DI();
   R_Systeminit();   
}











