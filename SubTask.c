
#include "SubTask.h"

void Sub_LoopTask(void)
{
   static uint8_t s_subTaskNum = 0; //��̬�ֲ����������򱾺���,������      
   
   switch(s_subTaskNum)      
   {
      case 0: Sub_Reserved();     break; // 0 Ԥ��      
      case 1: Sub_MotorDrive();   break; // 1 ����
      case 2: Sub_Current();      break; // 2 ����
	  case 3: Sub_Voltage();      break; // 3 ��ѹ
	  case 4: Sub_Temperature();  break; // 4 �¶�
	  case 5: Sub_HMI();          break; // 5 �˻��ӿ�
	  case 6: Sub_WireComm();     break; // 6 ����ͨѶ(UART,CAN,LIN)  
	  case 7: Sub_Wireless();     break; // 7 ����ͨѶ
	  case 8: Sub_Record();       break; // 8 ��¼�ٴ���  
	  case 9: Sub_Reserved();     break; // 9 Ԥ��
      default:break;
   }
   if(++s_subTaskNum > 9) s_subTaskNum = 0;     
}



static void TaskAndStaChg_InSlefChk(void) //�����������ļ����������ļ��к�������
{
   MosSelfChk();   
   
   #if 0
   if(g_sFlagError.bBusVdcUnder)	
   { g_eSysState = cSTA_ERR; }    //�й����л�Ϊ����״̬             
   else
   { g_eSysState = cSTA_WAIT; }   // 
   #endif
}

void MosSelfChk(void)
{
   ;

}

static void TaskAndStaChg_InWait(void)      
{
   #if 0
   if(g_sFlagError.bBusVdcUnder)
   { g_eSysState = cSTA_ERR; }     //�й����л�Ϊ����״̬                    
   
   else
   { 
      #if 1   
	  if(g_sMotor.uWorkFlag.bRecvFinish)   //�ɹ����յ���������            
	  {
         g_sMotor.uWorkFlag.bRecvFinish = 0;   
		 if(g_sMotor.u8MotorRecvBuf[2] != 0) //�����ٶȲ�Ϊ0�ù���״̬         
	     { g_eSysState = cSTA_RUN; RstVarBeforInRunSta(); } //��λ����״̬ǰ����
	  }
	  #endif
   } 
   #endif
}

void RstVarBeforInRunSta(void) //�ָ����빤��״̬ǰ�ı�����ʼֵ
{
   g_sMotor.eAct = eSTANDBY_ACT;   //���빤��״̬ǰ�õ������Ϊ��������
   g_sMotor.uStaCnt.u8Standby = 0; //����״̬������0 
}

static void TaskAndStaChg_InRun(void)      
{
}

void RstVarBeforInStopSta(void) //�ָ�����ɲ��ͣ��״̬ǰ�ı�����ʼֵ
{
   g_sMotor.u16BrakeTime = 0;    
}

static void TaskAndStaChg_InStop(void)    
{
   #if 0
   if(g_sFlagError.bBusVdcUnder)
   { g_eSysState = cSTA_ERR; }          //�й����л�Ϊ����״̬                
   else 
   {
      if(++g_sMotor.u8BrakeTime >= 20)  //ɲ��200msת����״̬   
      { g_sMotor.u8BrakeTime = 0; g_eSysState = cSTA_WAIT; }
   }
   #endif
}

static void TaskAndStaChg_InErr(void)
{
   //if(!g_sFlagError.bBusVdcUnder)
   //{ g_eSysState = cSTA_WAIT; }
}

static void Sub_MotorDrive(void)  //���������� 10ms/time           
{
   if(g_eSysState == cSTA_RUN && g_sMotor.eAct == eRUN_ACT)
   {
      if(++g_u16BlockCnt >= 200)  //����2s�޻����ź�,����ת����    
      { 
         g_u16BlockCnt = 0; 
		 //g_sFlagErr.Bits.bMotorBlock = 1;   
	  }                                                  
   }
   else
   { g_u16BlockCnt = 0; }       
}
void Sub_Current(void)  // 10ms/time �ŵ�STM32оƬ������                        
{
}

//��ѹ����
static void Sub_Voltage(void)  // 10ms/time  �ŵ�STM32оƬ������           
{
}

//�¶ȴ���:60��(605)->3.0043K,70��(498)->2.2212K,80��(406)->1.6669K,90��(330)->1.2676K,��ѹ����4.7K����+5V   
static void Sub_Temperature(void) //�ŵ�STM32оƬ������             
{

}

void MosTmpErrChk(S_FILT_DATA_T *psMosTmp,U_ERROR_FLAG_T *psflag)
{

}


static void Sub_HMI(void)
{
   static u8 DirFlashCnt = 0;     
  
   
   if(++DirFlashCnt >= 50)             //�й��ϵ���     
   { 
	  DirFlashCnt = 0; 
	  g_sFlagEve.Bits.bDirFlash++; 
   }    

   //if(!g_sFlagErr.Bits.bShortCurrent)
   if(!g_sFlagErr.all)  
   {
      
       
   }
    
}


/*--------------------------------------------------------------
              ���ӻ�����ͨѶģ��   
u8MotorRecvBuf[0]:ͷ��0x54
u8MotorRecvBuf[1]:�ӻ���ַ
u8MotorRecvBuf[2]:�ٶ�(Bit5-Bit0)+����(Bit7)+��ݵ��λ(Bit6)  
----------------------------------------------------------------*/
static void Sub_WireComm(void)  //����ͨ�ų���              
{
   if(g_sFlagErr.Bits.bShortCurrent)     //�����·״̬   
   { g_sMotor.u8MotorStatus = 0x01; }
   else if(g_sFlagErr.Bits.bMotorBlock)  //�����ת����   
   { g_sMotor.u8MotorStatus = 0x02; }       
#if 0
   //���÷��ͻ��� 
   g_sMotor.u8MotorSendBuf[0] = 0x54;  
   g_sMotor.u8MotorSendBuf[1] = g_sMotor.u8MotorRecvBuf[1]; //���÷��ʹӻ���ַ  
   g_sMotor.u8MotorSendBuf[2] = g_sMotor.u8MotorStatus;     //�������״̬ g_sBusIdc.RealValue
   g_sMotor.u8MotorSendBuf[3] = 0xff; 
 #endif 
 #if 1
   g_sMotor.u8MotorSendBuf[0] = 0x54;  
   g_sMotor.u8MotorSendBuf[1] = g_sMotor.u8MotorRecvBuf[1]; //���÷��ʹӻ���ַ
  
 //  g_sMotor.u8MotorSendBuf[2] = (g_sMotor.u16Speed <<2)|(g_sMotor.u8MotorStatus);     //�������״̬ g_sBusIdc.RealValue
  //  g_sMotor.u8MotorSendBuf[2] = ((g_sMotor.u16Speed>>6) <<2)|(g_sMotor.u8MotorStatus);     //�������״̬ g_sBusIdc.RealValue//
   
   g_sMotor.u8MotorSendBuf[2] = (g_sMotor.u8MotorStatus);     //�������״̬ g_sBusIdc.RealValue//
   g_sMotor.u8MotorSendBuf[3] = 0xff; 
   #endif 
#if 0
   if(g_sFlagErr.Bits.bShortCurrent)     //�����·״̬   
   { g_sMotor.u8MotorStatus = 0x02; }
   
   else if(g_sFlagErr.Bits.bMosOverTmp)  //mos���¹���
   { g_sMotor.u8MotorStatus = 0x03;}
   
   else if(g_sFlagErr.Bits.bAvgIdcOver)  //�������״̬       
   { g_sMotor.u8MotorStatus = 0x04; }
   
   else if(g_sFlagErr.Bits.bBusVdcUnder) //ĸ��Ƿѹ״̬   
   { g_sMotor.u8MotorStatus = 0x05; }
   
   else if(g_sFlagErr.Bits.bObstacles)   //�������״̬        
   { g_sMotor.u8MotorStatus = 0x06; }
   
   else                                  //�����������״̬0x0   
   { g_sMotor.u8MotorStatus = 0x0;}  

   //UartSendData(g_sMotor.u8MotorStatus);  
   
   //g_sMotor.u8MotorStatus = 0x0;
   //���÷��ͻ���    
   g_sMotor.u8MotorSendBuf[0] = 0x54;     
   g_sMotor.u8MotorSendBuf[1] = g_sMotor.u8MotorRecvBuf[1]; //���÷��ʹӻ���ַ  
   g_sMotor.u8MotorSendBuf[2] = g_sMotor.u8MotorStatus;     //�������״̬ g_sBusIdc.RealValue
   //g_sMotor.u8MotorSendBuf[2] = g_sBusIdc.RealValue;        //���͵������(16ms����1��)   
   g_sMotor.u8MotorSendBuf[3] = 0xff; 
#endif   
}

static void Sub_Wireless(void)
{
   ;
}

static void Sub_Record(void)  
{
  ;
}

static void Sub_Reserved(void)
{
  ;

}
















