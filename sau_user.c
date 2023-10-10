
#include "macrodriver.h"
#include "sau.h"
#include "userdefine.h"
#include "SubTask.h"
volatile u8 * gp_uart0_rx_address;        /* uart1 receive buffer address */

//UART0�����ж�
static void __near r_uart0_interrupt_receive(void)               
{
//   u8 err_type;
   u8 Recvdata,i;    

   SRIF0 = 0U;  
   EI();//�����ж�Ƕ��(���ж����ȼ�Ϊ3):AD�ж�(��������)���ȼ�1;��ʱ��0ͨ��1(��ǰ����)�ж����ȼ�2       
   Recvdata = RXD0;
   g_sMotor.u8MotorRecvBuf[g_sMotor.RecvCnt] = Recvdata; 
   
 #if 1
   switch(g_sMotor.RecvCnt)  
   {
      case 0: g_sMotor.RecvCnt = (Recvdata==0x54)?g_sMotor.RecvCnt+1:0;break;  
	  case 1: //����������ַ      
	  case 2: //�����Ƿ�������
	  case 3: //�����¶ȽǸ�8λ
	  case 4: //�����¶Ƚǵ�8λ  	
	  	     g_sMotor.RecvCnt++; break;                           
	  case 5:
	  	    if(Recvdata == 0xff)     
			{ 
			 
			   g_sMotor.RecvCnt = 0;           
			   g_sMotor.uWorkFlag.bRecvFinish = 1;    
			   
               g_sMotor.u16TargetSpeed = (u16)(64)*(g_sMotor.u8MotorRecvBuf[2]&0x3f);                        
			   switch(g_eSysState)        
			   {
                  case cSTA_WAIT: //����״̬�½��յ��ٶ�ֵ,������״̬   
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
				  case cSTA_RUN:  //����״̬��,���յ�ͣ��ָ���ת��ı�ʱ,��ɲ��ͣ��״̬
				  	   if(g_sMotor.u8MotorRecvBuf[2] == 0)
					   { 
					       g_eSysState = cSTA_STOP; 
					       R_INTC0_Stop();
                           R_INTC3_Stop();
                           R_INTC4_Stop();
					       RstVarBeforInStopSta(); 
					   }
					   if(g_sMotor.eDirBuf != g_sMotor.eDir)//ת��ı�      
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
               
			   if(g_eSysState == cSTA_ERR) //ͣ��״̬�ٷ������ݸ���оƬ        
			   { 
			      for(i=0;i<4;i++)
			      { 
			          UartSendData(g_sMotor.u8MotorSendBuf[i]); 
			      }    
			   }
			   
			   if(g_eSysState == cSTA_STOP) //ͣ��״̬�ٷ������ݸ���оƬ 
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

//UART0�����ж�:�ж�δʹ��
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






