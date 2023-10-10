
#include "adc.h"  
#include "GlobalVar.h"  
#include "sau.h"

//AD采样(10位/1024):单次选择等待转化完成,非中断方式
u16 AdcSampSingleChannelOnce(u8 eChannel)
{
   ADS = eChannel;
   ADIF = 0;        //清INTAD中断标志   
   ADCS = 1;        //允许AD转换运行
   while(!ADIF);
   ADIF = 0;
   ADCS = 0;        //禁止AD转换

   return(ADCR>>6); //转换结果存贮在ADCR高10位  
}


/***********************************************************************************************************************
* Function Name: r_adc_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
/*
   g_u16ADCBuffer[1]:存贮反电势AD数据            
   g_u16ADCBuffer[2]:导通相
   g_u16ADCBuffer[3]:母线电流   
   ADCR为16位寄存器,高10位存放结果(分辨率为10),AD中断内转换结果将10位数据少右移2位(应右移4实际右移2)转换成12位数据格式化
*/







