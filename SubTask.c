
#include "SubTask.h"

void Sub_LoopTask(void)
{
   static uint8_t s_subTaskNum = 0; //静态局部变量作用域本函数,生存期      
   
   switch(s_subTaskNum)      
   {
      case 0: Sub_Reserved();     break; // 0 预留      
      case 1: Sub_MotorDrive();   break; // 1 驱动
      case 2: Sub_Current();      break; // 2 电流
	  case 3: Sub_Voltage();      break; // 3 电压
	  case 4: Sub_Temperature();  break; // 4 温度
	  case 5: Sub_HMI();          break; // 5 人机接口
	  case 6: Sub_WireComm();     break; // 6 有线通讯(UART,CAN,LIN)  
	  case 7: Sub_Wireless();     break; // 7 无线通讯
	  case 8: Sub_Record();       break; // 8 记录再存贮  
	  case 9: Sub_Reserved();     break; // 9 预留
      default:break;
   }
   if(++s_subTaskNum > 9) s_subTaskNum = 0;     
}



static void TaskAndStaChg_InSlefChk(void) //函数作用域本文件可与其他文件中函数重名
{
   MosSelfChk();   
   
   #if 0
   if(g_sFlagError.bBusVdcUnder)	
   { g_eSysState = cSTA_ERR; }    //有故障切换为故障状态             
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
   { g_eSysState = cSTA_ERR; }     //有故障切换为故障状态                    
   
   else
   { 
      #if 1   
	  if(g_sMotor.uWorkFlag.bRecvFinish)   //成功接收到串口数据            
	  {
         g_sMotor.uWorkFlag.bRecvFinish = 0;   
		 if(g_sMotor.u8MotorRecvBuf[2] != 0) //传递速度不为0置工作状态         
	     { g_eSysState = cSTA_RUN; RstVarBeforInRunSta(); } //复位工作状态前变量
	  }
	  #endif
   } 
   #endif
}

void RstVarBeforInRunSta(void) //恢复进入工作状态前的变量初始值
{
   g_sMotor.eAct = eSTANDBY_ACT;   //进入工作状态前置电机动作为待机动作
   g_sMotor.uStaCnt.u8Standby = 0; //待机状态计数清0 
}

static void TaskAndStaChg_InRun(void)      
{
}

void RstVarBeforInStopSta(void) //恢复进入刹车停机状态前的变量初始值
{
   g_sMotor.u16BrakeTime = 0;    
}

static void TaskAndStaChg_InStop(void)    
{
   #if 0
   if(g_sFlagError.bBusVdcUnder)
   { g_eSysState = cSTA_ERR; }          //有故障切换为故障状态                
   else 
   {
      if(++g_sMotor.u8BrakeTime >= 20)  //刹车200ms转待机状态   
      { g_sMotor.u8BrakeTime = 0; g_eSysState = cSTA_WAIT; }
   }
   #endif
}

static void TaskAndStaChg_InErr(void)
{
   //if(!g_sFlagError.bBusVdcUnder)
   //{ g_eSysState = cSTA_WAIT; }
}

static void Sub_MotorDrive(void)  //驱动子任务 10ms/time           
{
   if(g_eSysState == cSTA_RUN && g_sMotor.eAct == eRUN_ACT)
   {
      if(++g_u16BlockCnt >= 200)  //持续2s无换相信号,报堵转故障    
      { 
         g_u16BlockCnt = 0; 
		 //g_sFlagErr.Bits.bMotorBlock = 1;   
	  }                                                  
   }
   else
   { g_u16BlockCnt = 0; }       
}
void Sub_Current(void)  // 10ms/time 放到STM32芯片中运算                        
{
}

//电压处理
static void Sub_Voltage(void)  // 10ms/time  放到STM32芯片中运算           
{
}

//温度处理:60℃(605)->3.0043K,70℃(498)->2.2212K,80℃(406)->1.6669K,90℃(330)->1.2676K,分压电阻4.7K上拉+5V   
static void Sub_Temperature(void) //放到STM32芯片中运算             
{

}

void MosTmpErrChk(S_FILT_DATA_T *psMosTmp,U_ERROR_FLAG_T *psflag)
{

}


static void Sub_HMI(void)
{
   static u8 DirFlashCnt = 0;     
  
   
   if(++DirFlashCnt >= 50)             //有故障灯灭     
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
              主从机串口通讯模块   
u8MotorRecvBuf[0]:头码0x54
u8MotorRecvBuf[1]:从机地址
u8MotorRecvBuf[2]:速度(Bit5-Bit0)+方向(Bit7)+割草电机位(Bit6)  
----------------------------------------------------------------*/
static void Sub_WireComm(void)  //有线通信程序              
{
   if(g_sFlagErr.Bits.bShortCurrent)     //电机短路状态   
   { g_sMotor.u8MotorStatus = 0x01; }
   else if(g_sFlagErr.Bits.bMotorBlock)  //电机堵转故障   
   { g_sMotor.u8MotorStatus = 0x02; }       
#if 0
   //设置发送缓存 
   g_sMotor.u8MotorSendBuf[0] = 0x54;  
   g_sMotor.u8MotorSendBuf[1] = g_sMotor.u8MotorRecvBuf[1]; //设置发送从机地址  
   g_sMotor.u8MotorSendBuf[2] = g_sMotor.u8MotorStatus;     //电机运行状态 g_sBusIdc.RealValue
   g_sMotor.u8MotorSendBuf[3] = 0xff; 
 #endif 
 #if 1
   g_sMotor.u8MotorSendBuf[0] = 0x54;  
   g_sMotor.u8MotorSendBuf[1] = g_sMotor.u8MotorRecvBuf[1]; //设置发送从机地址
  
 //  g_sMotor.u8MotorSendBuf[2] = (g_sMotor.u16Speed <<2)|(g_sMotor.u8MotorStatus);     //电机运行状态 g_sBusIdc.RealValue
  //  g_sMotor.u8MotorSendBuf[2] = ((g_sMotor.u16Speed>>6) <<2)|(g_sMotor.u8MotorStatus);     //电机运行状态 g_sBusIdc.RealValue//
   
   g_sMotor.u8MotorSendBuf[2] = (g_sMotor.u8MotorStatus);     //电机运行状态 g_sBusIdc.RealValue//
   g_sMotor.u8MotorSendBuf[3] = 0xff; 
   #endif 
#if 0
   if(g_sFlagErr.Bits.bShortCurrent)     //电机短路状态   
   { g_sMotor.u8MotorStatus = 0x02; }
   
   else if(g_sFlagErr.Bits.bMosOverTmp)  //mos过温故障
   { g_sMotor.u8MotorStatus = 0x03;}
   
   else if(g_sFlagErr.Bits.bAvgIdcOver)  //电机过流状态       
   { g_sMotor.u8MotorStatus = 0x04; }
   
   else if(g_sFlagErr.Bits.bBusVdcUnder) //母线欠压状态   
   { g_sMotor.u8MotorStatus = 0x05; }
   
   else if(g_sFlagErr.Bits.bObstacles)   //电机遇障状态        
   { g_sMotor.u8MotorStatus = 0x06; }
   
   else                                  //电机正常发送状态0x0   
   { g_sMotor.u8MotorStatus = 0x0;}  

   //UartSendData(g_sMotor.u8MotorStatus);  
   
   //g_sMotor.u8MotorStatus = 0x0;
   //设置发送缓存    
   g_sMotor.u8MotorSendBuf[0] = 0x54;     
   g_sMotor.u8MotorSendBuf[1] = g_sMotor.u8MotorRecvBuf[1]; //设置发送从机地址  
   g_sMotor.u8MotorSendBuf[2] = g_sMotor.u8MotorStatus;     //电机运行状态 g_sBusIdc.RealValue
   //g_sMotor.u8MotorSendBuf[2] = g_sBusIdc.RealValue;        //发送电机电流(16ms更新1次)   
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
















