
#include "macrodriver.h"
#include "sau.h"
#include "userdefine.h"
void R_SAU0_Create(void)   
{
   SAU0EN = 1U;    //允许提供输入时钟     
   NOP();
   NOP();
   NOP();
   NOP();
//串行时钟选择寄存器:设置串口可选择的两种运行时钟CK00和CK01;CK00=24M/2^2=6M;CK01=24M/2^2=6M;  
  SPS0 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
  
//串行时钟选择寄存器:设置串口可选择的两种运行时钟CK00和CK01;CK00=24M/2^4=1.5M;CK01=24M/2^1=12M;   
  //SPS0 = _0040_SAU_CK01_fCLK_4 | _0001_SAU_CK00_fCLK_1;

#if 0
//串行时钟选择寄存器:设置串口可选择的两种运行时钟CK00和CK01;CK00=24M/2^3=3M;CK01=24M/2^3=3M;
//SPS0 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0; //fclk(24M)
   SPS0 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
#endif

   R_UART0_Create();
   //R_SAU0_Create_UserInit();
}
/***********************************************************************************************************************
* Function Name: R_UART0_Create
* Description  : This function initializes the UART0 module.
                 串口通道0:UART0-Tx
                 串口通道1:UART0-Rx
                 串口通道2:UART1-Tx
                 串口通道3:UART1-Rx
* Arguments    : None
* Return Value : None  
***********************************************************************************************************************/
void R_UART0_Create(void)
{
//串行通道停止寄存器,ST00=1:将SEmn置0转移到通信待机状态后,ST00自动清0     
   ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON; //停止通道0(UART0发送)和通道1(UART0接收)

   STMK0 = 1U;     //禁止INTST0中断处理(UART0发送结束或缓冲器空中断)  
   STIF0 = 0U;     //清INTST0中断标志
    
   SRMK0 = 1U;     //禁止INTSR0(UART0接收结束)中断处理
   SRIF0 = 0U;     //清INTSR0中断标志
    
   SREMK0 = 1U;    //禁止INTSRE0接收错误中断(发生UART0 接收的通信错误)
   SREIF0 = 0U;    //清INTSRE0中断标志
    
   SRPR10 = 1U;    //设置UART0接收中断(INTSR0)优先级3   
   SRPR00 = 1U;

   STPR10 = 1U;    //设置UART0发送中断(INTST0)优先级3
   STPR00 = 1U;

/* 配置通道0 UART0发送 */	
   //SIR00 = _0004_SAU_SIRMN_FECTMN|_0002_SAU_SIRMN_PECTMN|_0001_SAU_SIRMN_OVCTMN;//串行标志清除触发寄存器:帧错误标志,奇偶校验错误标志,溢出错误标志
//串行模式寄存器SMR00  
//CKS00=0运行时钟选择CK00(3M); CCS00=0:传送时钟选择为cks00指定的时钟fMCK而非外部引脚时钟;
//MD002 MD001=01:UART模式; MD000(中断源选择) 0:选择传送结束中断非缓冲器空中断
#if 1   
   SMR00 = _0020_SMR00_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
           _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
#endif
#if 0
   SMR00 = _0020_SMR00_DEFAULT_VALUE | _8000_SAU_CLOCK_SELECT_CK01 | _0000_SAU_CLOCK_MODE_CKS | 
           _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
#endif
//串行通信运行设定寄存器 SCR00
//TXE00 RXE00=10:只进行发送;DAP00 CKP00=00在UART模式中需设为00;EOC=0:禁止产生错误中断INTSRE0
//PTC001 PTC000=00:无奇偶校验位发送且无奇偶校验位接收;DIR=1:优先发送/接收低位;
//SLC001 SLC000=01:1位停止位;DLS001 DLS000=11:8位数据长度
   SCR00 = _0004_SCR00_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
            _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
/* 串行数据寄存器 SDR00 */
//波特率={对象通道的运行时钟(fMCK=3M)频率}/(SDRmn[15:9]+1)/2[bps]    
//SDR00 = _5C00_SAU0_CH0_BAUDRATE_DIVISOR;     
//SDR00 = _9A00_SAU0_CH0_BAUDRATE_DIVISOR; //9A(1001 1010)->SDR00[15:9]=0b1001101=77->3M/(77+1)/2=19230bps   
   SDR00 = _9A00_SAU0_CH0_BAUDRATE_DIVISOR; //9A(1001 1010)->SDR00[15:9]=0b1001101=77->6M/(77+1)/2=38461bps
                                            //9A(1001 1010)->SDR00[15:9]=0b1001101=77->12M/(77+1)/2=76923bps    
/* 配置通道1 UART0接收 */	 
   NFEN0 |= _01_SAU_RXD0_FILTER_ON; //SNFEN00=1,用作RxD0引脚时,此位强制置1,使用RxD0引脚的噪声滤波器
//串行标志清除触发寄存器:帧错误标志,奇偶校验错误标志,溢出错误标志
   SIR01 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN  | _0001_SAU_SIRMN_OVCTMN ;
   SMR01 = _0020_SMR01_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
		   _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
   SCR01 = _0004_SCR01_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
		   _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
   SDR01 = _9A00_SAU0_CH1_BAUDRATE_DIVISOR;

/* 配置UART0输出(通道0 Tx) */   
   SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;             //串口通道0(UART0)输出值为1
   SOL0 &= (uint16_t)~_0001_SAU_CHANNEL0_INVERTED; //串口通道0(UART0)数据直接输出,非反向输出
   SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;            //允许串口通道0(UART0-Tx)输出

   PM5 |= 0x01U;	   //UART0-RX P50:输入口  
   PM5 &= 0xFDU;	   //UART0-TX P51:输出口 
   P5 |= 0x02U; 	   //UART0-TX P51=1  
}
/***********************************************************************************************************************
* Function Name: R_UART0_Start
* Description  : This function starts the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Start(void)
{
   SO0  |= _0001_SAU_CH0_DATA_OUTPUT_1;  //串口通道0(UART0)输出值为1
   SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;  //允许串口通道0(UART0-Tx)输出
   SS0  |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON; //触发开始:使SEmn为1后SS00/01 自动清0
   
   STIF0 = 0U;     //清INTST0(UART0发送完成中断)中断标志
   SRIF0 = 0U;     //清INTSR0(UART0接收完成中断)中断标志 
   SREIF0 = 0U;     //清INTSR0(UART0接收完成中断)中断标志 
   
   //STMK0 = 0U;     //允许UART0发送中断  
   STMK0 = 1U;     //禁止UART0发送中断
   SRMK0  = 0U;     //允许UART0接收中断  
   SREMK0 = 0U;     //允许UART0接收中断  

   //TXD0 = 0x86;
   STIF0 = 1;      //置串口0发送完成标志     
}
/***********************************************************************************************************************
* Function Name: R_UART0_Stop
* Description  : This function stops the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Stop(void)  
{
   STMK0 = 1U;  //禁止UART0发送中断   
   SRMK0 = 1U;  //禁止UART0接收中断
   ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON; //停止通道0(UART0发送)和通道1(UART0接收)
   SOE0 &= (uint16_t)~_0001_SAU_CH0_OUTPUT_ENABLE;               //禁止通道0(UART0-RX)输出
   STIF0 = 0U;  //清UART0发送完成标志
   SRIF0 = 0U;  //清UART0接收完成标志
}

void UartSendData(u8 tdata)
{
   while(!STIF0); //等待上次数据发送完成
   TXD0 = tdata;  //装载发送数据   
   STIF0 = 0;  
}



/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
