/**************************************************************************************************
* Copyright (c) 2020,大叶园林设备有限公司 
* 
* 项    目: DYM221501M040：（晶振内部fclk=24M)
* 文件名称: main.c 
* 文件标识:
* 摘    要: DC18V 智能割草机行走电机R7F0C009B2主控平台
*           电机参数:20V/25W/1.2A/4对级/3500RPM/Rs=1.7Ω/L=2mH 
* Rev 1
*   作者: Lichar
*   日期: 2020年03月09日
*   选项字节:000C0H-000C2H:F77FF0; 000C3:0x84
             000C0:允许看门狗,时间29.68ms,允许溢出中断             
             000C1:检测电压Vlvd,上升2.81V,下降2.75V     
             000C2:内部高速振荡器模式fih=24M,fhoco=48M   
             000C3:允许片上调试,ID验证失败时,擦除闪存数据                                            
**************************************************************************************************/
#include "main.h"

//static u16 cnt = 0; //静态全局变量作用域本文件   
uint8_t Ms1Flag;
uint8_t Ms30Flag;
uint8_t Us125Cnt;
uint16_t MsCnt;
uint16_t MsCnt2;

uint8_t x=0;
uint8_t ff[6]={0x11,0x12,0x13,0x14,0x15,0x16};

void main(void)                         
{
   DI();
   R_INTC0_Start();            //enable hall interrupt
   R_INTC4_Start();
   R_INTC3_Start();
   R_IT_Start();
   Delay_ms(10);  
   
   R_UART0_Start();          //波特率38400-Debug OK  
   R_COMP1_Start();
   R_TMRD0_Stop();
   EI();	        //Enable int.
   LampOff();
   ClosePwmAndIo();
   while(1)     
   {
      if(Flag_1ms)     // 1ms                          
	  {	 
          Flag_1ms = 0;  
		  CLRWDT;//
		  Sub_LoopTask();          
		  Main_MotorTask();
      }
   }
}











