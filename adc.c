
#include "macrodriver.h"
#include "adc.h"
#include "userdefine.h"

void R_ADC_Create(void)
{
   ADCEN = 1U;     //允许提供输入时钟
   ADM0 = 0x00U;   //清0 AD模式寄存器
   
   ADMK = 1U;      //禁止INTAD中断
   ADIF = 0U;      //清INTAD中断标志位

#if 0
   ADPR1 = 0U;     //设置INTAD中断优先级1 
   ADPR0 = 1U;
#endif   
#if 1
   ADPR1 = 0U;     //设置INTAD中断优先级0 
   ADPR0 = 0U;
#endif
    /* The reset status of ADPC is analog input, so it's unnecessary to set. */
   /* Set ANI0 - ANI3 pin */
   PM2 |= 0x0FU;		//设置P2.0-P2.3为输入口
   /* Set ANI16 - ANI19 pin */
  // PMC0 |= 0x03U;      //设置P0.0(IRms)-P0.1(BemfU)为模拟口
   PM12 |= 0x01U;      //设置P120为输入口
   PM14 |= 0x80U;      //设置P147为输入口
//   PM0 |= 0x03U;       //设置P0.0-P0.1为输入口
   PMC12 |= 0x01U;     //设置P120(BemfW)为模拟口
   PMC14 |= 0x80U;     //设置P147(BemfV)为模拟口
   
	
//注意:将ADCE(停止/允许电压比较器运行)置1后至少经过1us然后才能将ADCS(停止/允许转换)位置1
//停止AD转换运行,扫描模式,停止AD电压比较器运行,转换模式2,转换时间2.83us(5V供电转换时间在2.125-39us之间)    
   //ADM0 = _40_AD_OPERMODE_SCAN | _30_AD_CONVERSION_CLOCK_4 | _02_AD_TIME_MODE_NORMAL_2; // 0111 0010  
//停止AD转换运行,扫描模式,停止AD电压比较器运行,转换模式2,转换时间3.54us(3.3V供电转换时间在3.1875-39us之间) 
   ADM0 = _40_AD_OPERMODE_SCAN | _28_AD_CONVERSION_CLOCK_5 | _02_AD_TIME_MODE_NORMAL_2; // 0110 1010
   ADM1 = _00_AD_TRIGGER_SOFTWARE | _00_AD_CONVMODE_SEQSELECT; //软件触发模式，连续转换模式
//AD转换正基准电压由VDD提供,负基准电压VSS,ADLL≤ADCR≤ADUL(AREA1)时产生中断信号(INTAD),10位分辨率  
   ADM2 = _00_AD_POSITIVE_VDD | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 | _00_AD_RESOLUTION_10BIT;
   ADUL = _FF_AD_ADUL_VALUE; //转换结果比较上限值设定寄存器
   ADLL = _00_AD_ADLL_VALUE; //转换结果比较下限值设定寄存器
   ADS = _11_AD_INPUT_CHANNEL_17; //设定ADC通道0b0001 0001   
    
   ADCE = 1U; //进入转换待机状态:允许AD电压比较器运行,注意至少1US后才能置位ADCS(停止/允许转换)  
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
