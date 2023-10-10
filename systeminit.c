
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
   PIOR1 = 0x00U;           //定时器RJ TRJO0和P30/P01复用
   R_CGC_Get_ResetSource(); //时钟配置,已在编译器配置项内配置过
   R_CGC_Create();          //系统时钟配置        
   R_PORT_Create();		    //GPIO配置
   R_TAU0_Create();		    //定时器配置  
   R_TMRD0_Create();        //定时器RD配置(PWM) // 
   R_SAU0_Create();         //串口配置
   R_ADC_Create(); 		    //ADC配置 
   R_COMPPGA_Create(); 	    //比较器/放大器配置  
   R_ELC_Create();          //ELC配置
    R_INTC_Create();
    R_IT_Create();
   IAWCTL = 0x00U;
   #endif 
   #if 0
      PIOR1 = 0x00U;           //定时器RJ TRJO0和P30/P01复用
   R_CGC_Get_ResetSource(); //时钟配置,已在编译器配置项内配置过
   R_CGC_Create();          //系统时钟配置        
   R_PORT_Create();		    //GPIO配置
   R_TAU0_Create();		    //定时器配置  
   R_TMRD0_Create();        //定时器RD配置(PWM)  
   R_SAU0_Create();         //串口配置
   R_ADC_Create(); 		    //ADC配置 
 //  R_COMPPGA_Create(); 	    //比较器/放大器配置  
   R_ELC_Create();          //ELC配置
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











