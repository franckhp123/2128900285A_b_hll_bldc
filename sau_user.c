
#include "macrodriver.h"
#include "sau.h"
#include "userdefine.h"
#include "SubTask.h"
volatile u8 * gp_uart0_rx_address;        /* uart1 receive buffer address */

//UART0接收中断
static void __near r_uart0_interrupt_receive(void)               
{
//   u8 err_type;
   u8 Recvdata,i;    

   SRIF0 = 0U;  
   EI();//允许中断嵌套(此中断优先级为3):AD中断(过零点计算)优先级1;定时器0通道1(超前换相)中断优先级2       
   Recvdata = RXD0;
   g_sMotor.u8MotorRecvBuf[g_sMotor.RecvCnt] = Recvdata; 
   
 #if 1
   switch(g_sMotor.RecvCnt)  
   {
      case 0: g_sMotor.RecvCnt = (Recvdata==0x54)?g_sMotor.RecvCnt+1:0;break;  
	  case 1: //接收器件地址      
	  case 2: //接收是否工作数据
	  case 3: //接收坡度角高8位
	  case 4: //接收坡度角低8位  	
	  	     g_sMotor.RecvCnt++; break;                           
	  case 5:
	  	    if(Recvdata == 0xff)     
			{ 
			 
			   g_sMotor.RecvCnt = 0;           
			   g_sMotor.uWorkFlag.bRecvFinish = 1;    
			   
               g_sMotor.u16TargetSpeed = (u16)(64)*(g_sMotor.u8MotorRecvBuf[2]&0x3f);                        
			   switch(g_eSysState)        
			   {
                  case cSTA_WAIT: //待机状态下接收到速度值,置运行状态   
				  	   if(g_sMotor.u8MotorRecvBuf[2] != 0)
					   { 
					      g_eSysState = cSTA_RUN; RstVarBeforInRunSta();
						  if(TESTBIT(g_sMotor.u8MotorRecvBuf[2],7)) 
						  { 
						      g_sMotor.eDir = eCCW;
						  }         
	                      else                                      
	                      { 
	                          g_sMotor.eDir = eCW; 
	                      }
						  
					   }
					   else
                       {
                            for(i=0;i<4;i++)
                            {
                                UartSendData(g_sMotor.u8MotorSendBuf[i]); 
                            }                           
                       }
                       break;
				  case cSTA_RUN:  //运行状态下,接收到停机指令或转向改变时,置刹车停机状态
				  	   if(g_sMotor.u8MotorRecvBuf[2] == 0)
					   { 
					       g_eSysState = cSTA_STOP; 
					       R_INTC0_Stop();
                           R_INTC3_Stop();
                           R_INTC4_Stop();
					       RstVarBeforInStopSta(); 
					   }
					   if(g_sMotor.eDirBuf != g_sMotor.eDir)//转向改变      
                       { 
        
                       }   
					   else
                       {
                            for(i=0;i<4;i++)
                            {
                                UartSendData(g_sMotor.u8MotorSendBuf[i]); 
                            }                         
                       }
					   break;
				  default:       break;         
			   }  
               
			   if(g_eSysState == cSTA_ERR) //停机状态再发送数据给主芯片        
			   { 
			      for(i=0;i<4;i++)
			      { 
			          UartSendData(g_sMotor.u8MotorSendBuf[i]); 
			      }    
			   }
			   
			   if(g_eSysState == cSTA_STOP) //停机状态再发送数据给主芯片 
			   {
			      for(i=0;i<4;i++)
			      { 
			          UartSendData(g_sMotor.u8MotorSendBuf[i]); 
			      } 
			   }
			}
			else                 
			{ 
			    g_sMotor.RecvCnt = 0; 
			    g_sMotor.uWorkFlag.bRecvFinish = 0; 
			}               
	  	    break;
      default:
            break;
   }
   #endif 
}

//UART0发送中断:中断未使能
static void __near r_uart0_interrupt_send(void)  
{
}

static void __near r_uart0_interrupt_error(void)
{
    u8 err_type;

    *gp_uart0_rx_address = RXD0;
    err_type = (uint8_t)(SSR01 & 0x0007U);
    SIR01 = (uint16_t)err_type;
    r_uart0_callback_error(err_type);
}



static void r_uart0_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
   // LED_REV();
    /* End user code. Do not edit comment generated here */
}






