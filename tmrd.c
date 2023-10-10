
#include "macrodriver.h"
#include "tmrd.h"
#include "userdefine.h"

/***********************************************************************************************************************
* Function Name: R_TMRD0_Create
* Description  : This function initializes the TMRD0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if 1

/***********************************************************************************************************************
* Function Name: R_TMRD0_Create
* Description  : This function initializes the TMRD0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_TMRD0_Create(void)    
{
    TRD0EN = 1U;    //允许提供定时器RD输入时钟  

	//CSEL1=1在和TRDGRA1寄存器比较匹配后还继续计数;CSEL0=1在和TRDGRA0寄存器比较匹配后还继续计数;	
    TRDSTR |= _08_TMRD_TRD1_COUNT_CONTINUES | _04_TMRD_TRD0_COUNT_CONTINUES;
	
    //TSTART1=0:TRD1停止计数;TSTART0=0:TRD0停止计数
    TRDSTR &= (uint8_t)~(_02_TMRD_TRD1_COUNT_START | _01_TMRD_TRD0_COUNT_START);  

    /* intpregbitset INTTRD0 disable */
    TRDMK0 = 1U;    //禁止TRD0中断 INTTRD0
    TRDIF0 = 0U;    //清TRD0中断标志位

    /* intpregbitset INTTRD1 disable */
    TRDMK1 = 1U;    //禁止TRD1中断 INTTRD1
    TRDIF1 = 0U;    //清TRD0中断标志位
		
    TRDPR10 = 0U;   //设置INTTRD0优先级为1    
    TRDPR00 = 1U;  
	

//ELC事件输入0(ELCOBE0)允许强制截止定时器RD的脉冲输出; ELC事件输入1(ELCOBE1)允许强制截止定时器RD的脉冲输出;
    TRDELC = _02_TMRD0_CUTOFF_ENABLED | _20_TMRD1_CUTOFF_ENABLED;   
 	
    /* regset TRDMR ContentOnly=[4-7] */    
//TRDGRD1是TRDGRB1的buf;TRDGRC1是TRDGRA1的buf;TRDGRD0是TRDGRB0的buf;TRDGRC0是TRDGRA0的buf;TRD0和TRD1独立运行
    TRDMR = _80_TMRD_TRDGRD1_BUFFER | _40_TMRD_TRDGRC1_BUFFER | _20_TMRD_TRDGRD0_BUFFER | _10_TMRD_TRDGRC0_BUFFER;

//0b0000 1101	选择复位同步PWM模式;正向电平输出:初始输出L,高电平有效;反向电平输出:初始输出L,高电平有效
    TRDFCR |= _01_TMRD_TRANSFER_RESET_SYNCHRONOUS | _04_TMRD_NORMAL_PHASE_LEVEl_LH | _08_TMRD_COUNTER_PHASE_LEVEl_LH;
  

//输出禁止(TRDIOA0引脚为I/O);输出禁止(TRDIOC0引脚为I/O);
//TRDIOB0(WH)允许输出;TRDIOD0(WL)允许输出;TRDIOA1(VH)允许输出;  
//TRDIOB1(UH)允许输出;TRDIOC1(VL)允许输出;TRDIOD1(UL)允许输出;  
	TRDOER1 = _01_TMRD_TRDIOA0_OUTPUT_DISABLE | _00_TMRD_TRDIOB0_OUTPUT_ENABLE | _04_TMRD_TRDIOC0_OUTPUT_DISABLE | 
              _00_TMRD_TRDIOD0_OUTPUT_ENABLE | _00_TMRD_TRDIOA1_OUTPUT_ENABLE | _00_TMRD_TRDIOB1_OUTPUT_ENABLE | 
              _00_TMRD_TRDIOC1_OUTPUT_ENABLE | _00_TMRD_TRDIOD1_OUTPUT_ENABLE;

//定时器 RD 输出控制寄存器:初始输出L电平
    TRDOCR = _00_TMRD_TRDIOC0_INITIAL_OUTPUT_L;    

//定时器RD0数字滤波器功能选择寄存器配置:禁止强制截止
    TRDDF0 = _00_TMRD_TRDIOB_FORCEDCUTOFF_DISABLE | _00_TMRD_TRDIOD_FORCEDCUTOFF_DISABLE;
//定时器RD1数字滤波器功能选择寄存器配置:禁止强制截止
    TRDDF1 = _00_TMRD_TRDIOA_FORCEDCUTOFF_DISABLE | _00_TMRD_TRDIOB_FORCEDCUTOFF_DISABLE | 
             _00_TMRD_TRDIOC_FORCEDCUTOFF_DISABLE | _00_TMRD_TRDIOD_FORCEDCUTOFF_DISABLE;

#if 0
    /* regset TRDDF0 */
    TRDDF0 = _20_TMRD_TRDIOB_LOW_OUTPUT | _00_TMRD_TRDIOC_FORCEDCUTOFF_DISABLE | _02_TMRD_TRDIOD_LOW_OUTPUT;
    /* regset TRDDF1 */
    TRDDF1 = _80_TMRD_TRDIOA_LOW_OUTPUT | _20_TMRD_TRDIOB_LOW_OUTPUT | _08_TMRD_TRDIOC_LOW_OUTPUT | 
             _02_TMRD_TRDIOD_LOW_OUTPUT;
#endif	
    /* regset TRDCR0 */   
	
//0b0010 0000 TRD0在和TRDGRA0寄存器比较匹配时清除TRD0寄存器;计数源选择fclk(24M)或fHOCO(48M)当FRQSEL4=1选fHOCO
    TRDCR0 = _20_TMRD_COUNTER_CLEAR_TRDGRA | _00_TMRD_INETNAL_CLOCK_FCLK_FHOCO;    
	
//定时器RD0中断允许寄存器:0b0000 0010 IMIEB=1允许因IMFB位产生的中断(IMIB)  
    TRDIER0 = _02_TMRD_IMIB_ENABLE;  
    /* regset TRDIER1 */
    /* regset TRD0 */
   
#if 1
#if 0
    TRDGRA0 = _12BF_TMRD_TDRGRA0_VALUE;  //TRDGRA0=0x12bf=4799->fpwm=(1/48M)*(4799+1)=100us=10K  
    TRDGRB0 = _095F_TMRD_TDRGRB0_VALUE;
    TRDGRC0 = _12BF_TMRD_TDRGRA0_VALUE;
    TRDGRD0 = _095F_TMRD_TDRGRD0_VALUE;
    TRDGRA1 = _095F_TMRD_TDRGRA1_VALUE;
    TRDGRB1 = _095F_TMRD_TDRGRB1_VALUE;
    TRDGRC1 = _095F_TMRD_TDRGRC1_VALUE;
    TRDGRD1 = _095F_TMRD_TDRGRD1_VALUE;       
#endif	
#if 1
              
               TRDGRA0 = _12BF_TMRD_TDRGRA0_VALUE;  //TRDGRA0=0x12bf=4799->fpwm=(1/48M)*(4799+1)=100us=10K  
               TRDGRB0 = 0x10df;
               TRDGRC0 = _12BF_TMRD_TDRGRA0_VALUE;
               TRDGRD0 = 0x10df;
               TRDGRA1 = 0x10df;
               TRDGRB1 = 0x10df;
               TRDGRC1 = 0x10df;
               TRDGRD1 = 0x10df; 

#endif 
    


    /* Set TRDIOB0 pin */
    POM1 &= 0xDFU;      //0b1101 1111普通输出模式 P1.5(WH)      
    P1 &= 0xDFU;        //P1.5=0
    PM1 &= 0xDFU;       //P1.5输出口 
    /* Set TRDIOD0 pin */
    P1 &= 0xEFU;        //P1.4=0
    PM1 &= 0xEFU;       //P1.4(WL)输出口 
    /* Set TRDIOA1 pin */
    P1 &= 0xF7U;        //P1.3=0  
    PM1 &= 0xF7U;       //P1.3(VH)输出口 
    /* Set TRDIOB1 pin */
    P1 &= 0xFBU;        //P1.2=0 
    PM1 &= 0xFBU;       //P1.2(UH)输出口 
    /* Set TRDIOC1 pin */
    P1 &= 0xFDU;        //P1.1=0 
    PM1 &= 0xFDU;       //P1.1(VL)输出口
    /* Set TRDIOD1 pin */
    POM1 &= 0xFEU;      //P1.0(UL)普通输出模式               
    P1 &= 0xFEU;        //P1.0=0 
    PM1 &= 0xFEU;       //P1.0输出口  
    #endif 
}

/***********************************************************************************************************************
* Function Name: R_TMRD0_Start
* Description  : This function starts TMRD0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TMRD0_Start(void)
{
   volatile uint8_t trdsr_dummy;
   TRDIF0 = 0U;           //清INTTRD0中断标志  
   trdsr_dummy = TRDSR0;  //在写0前先读TRDSR0
   TRDSR0 = 0x00U;        //清TRD0每个中断请求标志位
	
   trdsr_dummy = TRDSR1;  //在写0前先读TRDSR1  
   TRDSR1 = 0x00U;        //清TRD1每个中断请求标志位
   TRDMK0 = 0U;           //使能中断INTTRD0:定时器RD0和TRDGRA0(设定PWM周期)比较匹配时(PWM周期中断)
     
   TRDSTR |= _04_TMRD_TRD0_COUNT_CONTINUES; //CSEL0=1:TRD0计数和TRDGRA0匹配时继续计数  
   TRDSTR |= _01_TMRD_TRD0_COUNT_START;     //TSTART0=1:TRD0开始计数  
}
/***********************************************************************************************************************
* Function Name: R_TMRD0_Stop
* Description  : This function stops TMRD0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TMRD0_Stop(void)   
{
   volatile uint8_t trdsr_dummy;
   TRDSTR |= _04_TMRD_TRD0_COUNT_CONTINUES;//CSEL0=1:RD0和TRDGRA0寄存器比较匹配后还继续计数;
   TRDSTR &= (uint8_t)~_01_TMRD_TRD0_COUNT_START;//TSTART0=0:TRD0停止计数
	
   TRDMK0 = 1U;          //禁止TRD0 INTTRD0(TRD0计数和TRDGRA0匹配时继续计数)中断
   TRDIF0 = 0U;          //清TRD0中断标志位
   trdsr_dummy = TRDSR0; //在写TRDSR0 0前先读TRDSR0
   TRDSR0 = 0x00U;       //清TRD0每个中断请求标志位(OVF,IMFD,IMFC,IMFB,IMFA)
   trdsr_dummy = TRDSR1; //在写TRDSR1 0前先读TRDSR1
   TRDSR1 = 0x00U;       //清TRD0每个中断请求标志位(OVF,IMFD,IMFC,IMFB,IMFA)
}

#endif 







