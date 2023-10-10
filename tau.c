
#include "macrodriver.h"
#include "tau.h"
#include "userdefine.h"  

/******************************************************

  定时器0:
          通道1:超前换相
          通道3:1ms定时主程序循坏时基
          
******************************************************/

void R_TAU0_Create(void)  
{
//提供输入时钟(能读写定时器阵列单元0使用的SFR)     
    TAU0EN = 1U;    
/*  TPS0:时钟选择器(4种),可选择时钟0:fclk/2^2; 可选择时钟1:fclk/2^4; 可选择时钟2:fclk/2^1; 可选择时钟3:fclk/2^8;*/
    TPS0 = _0000_TAU_CKM3_fCLK_8 | _0000_TAU_CKM2_fCLK_1 | _0040_TAU_CKM1_fCLK_4| _0002_TAU_CKM0_fCLK_2;

/* 关闭定时器所有通道 */        
//TT0(通道停止寄存器属于触发寄存器)相应位置1停止定时器--->TE0(定时器允许状态寄存器只读)对应位清0--->TT0相应位自动清0
    TT0 = _0800_TAU_CH3_H8_STOP_TRG_ON | _0200_TAU_CH1_H8_STOP_TRG_ON | _0008_TAU_CH3_STOP_TRG_ON | 
          _0004_TAU_CH2_STOP_TRG_ON | _0002_TAU_CH1_STOP_TRG_ON | _0001_TAU_CH0_STOP_TRG_ON;   
	
    DISABLE_T0CHAN_0_INT;       //禁止定时器0通道0中断  
    CLR_T0CHAN_0_INTFLAG;       //清定时器0通道0中断标志
    SET_T0CHAN_0_INTPRIO3;      //设置定时器0通道0中断(中断向量:INTTM00)优先级为3 
    
    DISABLE_T0CHAN_1_INT;
	CLR_T0CHAN_1_INTFLAG;
    DISABLE_T0CHAN_1_HIGH8_INT; //通道1的高8位定时器中断使能位INTTM01H:1禁止;0允许  
	CLR_T0CHAN_1_HIGH8_INTFLAG; //INTTM01H中断标志位 
	SET_T0CHAN_1_INTPRIO2;      //设置定时器0通道1中断(中断向量:INTTM01)优先级为2 
    
    DISABLE_T0CHAN_2_INT;
	CLR_T0CHAN_2_INTFLAG;
	SET_T0CHAN_2_INTPRIO2;      //设置定时器0通道2中断(中断向量:INTTM02)优先级为2 

	DISABLE_T0CHAN_3_INT;
	//CLR_T0CHAN_3_INTFLAG;
	//SET_T0CHAN_3_INTPRIO3;      //设置定时器0通道3中断(中断向量:INTTM03)优先级为3 

	//DISABLE_T0CHAN_3_HIGH8_INT; //通道3的高8位定时器中断使能位INTTM03H:1禁止;0允许  
   // CLR_T0CHAN_3_HIGH8_INTFLAG; //INTTM03H中断标志位
      
//定时器0:通道0配置(计算换相间隔时间)    	
//通道0:时钟选择CKM1(fclk/2^4),计数时钟选择由CKS位设置(即CKM1)	,软件触发开始,间隔定时模式,在开始计数时不产生定时器中断
    TMR00 = _8000_TAU_CLOCK_SELECT_CKM1|_0000_TAU_CLOCK_MODE_CKS|_0000_TAU_TRIGGER_SOFTWARE|
            _0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;
	TDR00 = _FFFF_TAU_TDR00_VALUE; //定时器比较寄存器值为默认值
//定时器输出:只有在禁止定时器输出(TOE0=0）时才能通过软件改写此寄存器的TOmn位    
	TO0 &= (uint16_t)~_0001_TAU_CH0_OUTPUT_VALUE_1; 
	TOE0 &= (uint16_t)~_0001_TAU_CH0_OUTPUT_ENABLE; //禁止定时器输出   
	
//定时器0:通道1配置
/*  通道1:时钟选择CKM1(fclk/2^4),计数时钟选择由CKS位设置(即CKM1),用做16位定时器
          软件触发开始,间隔定时模式,在开始计数时不产生定时器中断 */
    TMR01 = _8000_TAU_CLOCK_SELECT_CKM1|_0000_TAU_CLOCK_MODE_CKS|_0000_TAU_16BITS_MODE|
	        _0000_TAU_TRIGGER_SOFTWARE|_0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;
    TDR01 = _FFFF_TAU_TDR01_VALUE;
//定时器输出模式寄存器:主控通道输出,从属通道输出  
    TOM0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_COMBIN;  
//定时器输出电平的寄存器
    TOL0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_LEVEL_L; 
//定时器输出:只有在禁止定时器输出(TOE0=0）时才能通过软件改写此寄存器的TOmn位  
    TO0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_VALUE_1;
	TOE0 &= (uint16_t)~_0002_TAU_CH1_OUTPUT_ENABLE; //禁止定时器输出
	
//定时器0:通道2配置
/***	通道2:时钟选择CKM1(fclk/2^4),计数时钟选择由CKS位设置(即CKM1),用做独立通道,  
	      软件触发开始,间隔定时模式,在开始计数时不产生定时器中断 ***/
    TMR02 = _8000_TAU_CLOCK_SELECT_CKM1 | _0000_TAU_CLOCK_MODE_CKS|_0000_TAU_COMBINATION_SLAVE|
	        _0000_TAU_TRIGGER_SOFTWARE|_0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;
    TDR02 = _FFFF_TAU_TDR02_VALUE;
	TOM0 &= (uint16_t)~_0004_TAU_CH2_OUTPUT_COMBIN;  //定时器输出模式寄存器:主控通道输出,从属通道输出  
	TOL0 &= (uint16_t)~_0004_TAU_CH2_OUTPUT_LEVEL_L; //定时器输出电平的寄存器
	TO0 &= (uint16_t)~_0004_TAU_CH2_OUTPUT_VALUE_1;  //定时器输出:通道2输出0
	TOE0  &= (uint16_t)~_0004_TAU_CH2_OUTPUT_ENABLE; //禁止定时器输出
	
//定时器0:通道3配置
/*通道3:时钟选择CKM0(fclk/2^2),计数时钟选择由CKS位设置(即CKM0),用做16位定时器
	    软件触发开始,间隔定时模式,在开始计数时不产生定时器中断     
*/  
  //  TMR03 = _0000_TAU_CLOCK_SELECT_CKM0|_0000_TAU_CLOCK_MODE_CKS|_0000_TAU_16BITS_MODE| 
	//        _0000_TAU_TRIGGER_SOFTWARE|_0000_TAU_MODE_INTERVAL_TIMER|_0000_TAU_START_INT_UNUSED;

//+2是因为开始计数时1clock加载TDR(比较寄存器值);1clock在递减计数到0时触发中断  
//	TDR03 = 5998;  //TDR03:1ms:24M/2^2=6M->(1/6M)*(5998+2)=1ms;  
//	TOM0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_COMBIN;  //定时器输出模式寄存器:主控通道输出,从属通道输出  
//	TOL0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_LEVEL_L; //定时器输出电平的寄存器
//	TO0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_VALUE_1;  //定时器输出:通道3输出0
//	TOE0 &= (uint16_t)~_0008_TAU_CH3_OUTPUT_ENABLE;  //禁止定时器通道3输出
}
/***********************************************************************************************************************
* Function Name: R_TAU0_Channel0_Start
* Description  : This function starts TAU0 channel 0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TAU0_Channel0_Start(void)
{
   //CLR_T0CHAN_0_INTFLAG;               //清标志
   //ENABLE_T0CHAN_0_INT;                //使能中断
   
   TS0 |= _0001_TAU_CH0_START_TRG_ON;  //开启定时器
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
void R_TAU0_Channel3_Start(void) //主程序1ms时基
{
   DISABLE_T0CHAN_3_INT;
   CLR_T0CHAN_3_INTFLAG;
   
   TS0 |= _0008_TAU_CH3_START_TRG_ON;      //开启定时器,TE0对应位置1后,开启标志自动清0
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


